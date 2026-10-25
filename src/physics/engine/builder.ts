// ===========================================================================
// MMD physics engine (TypeScript port) – PMX → PhysicsWorld builder
// Port of PmxPhysicsBuilder.cs: builds bodies / joints, bone links (bind offsets),
// kinematic driving, FK-rest reset and the [physics + bone alignment] write-back helper.
// ===========================================================================

import { f, Quat, RigidTransform, Vec3 } from './math';
import { Joint, JointType } from './constraints';
import { PmxPhysicsModel, PmxRigidBody } from './pmx';
import { PhysicsMode, RigidBody } from './rigidBody';
import { BoxShape, CapsuleShape, CollisionShape, SphereShape } from './shapes';
import { PhysicsWorld } from './world';

/** Body ↔ bone link. bodyOffsetFromBone = bone^-1 * body at bind time. */
export interface BoneLink {
  body: RigidBody;
  boneIndex: number;
  bodyOffsetFromBone: RigidTransform;
  mode: PhysicsMode;
}

/** Bone index → world pose of that bone (null = unavailable). */
export type BoneWorldFn = (boneIndex: number) => RigidTransform | null;

export class PmxPhysicsBuilder {
  readonly world = new PhysicsWorld();
  readonly boneLinks: BoneLink[] = [];
  readonly bodies: RigidBody[] = [];
  private model!: PmxPhysicsModel;
  private _hasBoneMerge = false;

  /** Mass clamp for dynamic bodies (0 = disabled, the default). */
  static maxDynamicMass = 0;
  static clampedMassCount = 0;

  static build(model: PmxPhysicsModel): PmxPhysicsBuilder {
    const b = new PmxPhysicsBuilder();
    b.model = model;
    b.buildBodies(model);
    b.buildJoints(model);
    return b;
  }

  private buildBodies(model: PmxPhysicsModel) {
    for (const rb of model.rigidBodies) {
      const shape = createShape(rb);
      const body = new RigidBody(shape);
      body.name = rb.name;
      body.boneIndex = rb.boneIndex;
      body.group = rb.group;
      body.collisionMask = rb.nonCollisionGroup;
      body.mode = rb.physicsMode as PhysicsMode;
      body.linearDamping = rb.linearDamping;
      body.angularDamping = rb.angularDamping;
      body.restitution = rb.restitution;
      body.friction = rb.friction;
      body.worldTransform = RigidTransform.fromEuler(rb.position, rb.rotation);
      body.kinematicTarget = body.worldTransform;
      body.setMassProps(body.mode === PhysicsMode.BoneFollow ? 0 : clampMass(rb.mass));

      this.world.addBody(body);
      this.bodies.push(body);

      const link: BoneLink = {
        body,
        boneIndex: rb.boneIndex,
        mode: body.mode,
        bodyOffsetFromBone: computeOffset(model, rb),
      };
      this.boneLinks.push(link);
      if (link.mode === PhysicsMode.DynamicBoneMerge) this._hasBoneMerge = true;
    }
  }

  /** Places every body at boneWorld * offset, zeroes velocities and clears contacts. */
  resetBodiesToBonePose(getBoneWorld: BoneWorldFn) {
    for (const link of this.boneLinks) {
      const body = link.body;
      if (link.boneIndex >= 0) {
        const bw = getBoneWorld(link.boneIndex);
        if (bw) {
          body.worldTransform = bw.mul(link.bodyOffsetFromBone);
          body.kinematicTarget = body.worldTransform;
          body.kinematicStepTarget = body.worldTransform;
        }
      }
      body.linearVelocity = Vec3.Zero;
      body.angularVelocity = Vec3.Zero;
      body.updateInertiaWorld();
    }
    this.world.clearContacts();
  }

  /** FK-rest physics reset: physics bones are rebuilt from the parent chain, driven bones use getDrivenBoneWorld. */
  resetBodiesToBonePoseFk(getDrivenBoneWorld: BoneWorldFn) {
    const model = this.model;
    const n = model.boneNames.length;
    const isPhysics = new Array<boolean>(n).fill(false);
    for (const link of this.boneLinks)
      if (link.boneIndex >= 0 && link.boneIndex < n && link.mode !== PhysicsMode.BoneFollow) isPhysics[link.boneIndex] = true;

    const world: (RigidTransform | null)[] = new Array(n).fill(null);
    const fk = (i: number, depth: number): RigidTransform => {
      const w = world[i];
      if (w) return w;
      if (depth > 512) return (world[i] = new RigidTransform(Quat.Identity, model.bonePositions[i]));
      const driven = isPhysics[i] ? null : getDrivenBoneWorld(i);
      let res: RigidTransform;
      if (driven) res = driven;
      else {
        const p = i < model.boneParents.length ? model.boneParents[i] : -1;
        if (p < 0 || p >= n) res = new RigidTransform(Quat.Identity, model.bonePositions[i]);
        else {
          const pw = fk(p, depth + 1);
          const localOff = model.bonePositions[i].sub(model.bonePositions[p]);
          res = new RigidTransform(pw.rotation, pw.rotation.rotate(localOff).add(pw.origin));
        }
      }
      world[i] = res;
      return res;
    };
    for (let i = 0; i < n; i++) if (!world[i]) fk(i, 0);

    this.resetBodiesToBonePose((i) => (i >= 0 && i < n ? world[i] : null));
  }

  /** Sets KinematicTarget = boneWorld * offset for every BoneFollow body (null keeps the previous target). */
  applyKinematicTargets(getDrivenBoneWorld: BoneWorldFn) {
    for (const link of this.boneLinks) {
      if (link.mode !== PhysicsMode.BoneFollow || link.boneIndex < 0) continue;
      const bw = getDrivenBoneWorld(link.boneIndex);
      if (bw) link.body.kinematicTarget = bw.mul(link.bodyOffsetFromBone);
    }
  }

  get hasBoneMergeBodies() {
    return this._hasBoneMerge;
  }

  /**
   * [physics + bone alignment] reproduction (PmxEditor correction layer).
   * Returns boneIndex → corrected world pose (null for bones the helper does not produce).
   * @param rotClampAlpha 0 = disabled; >0 pulls the parent-joint relative Euler back toward its limits.
   * @param alignAllPositions true = rebuild every physics bone position from the parent chain.
   */
  computeAlignedBonePoses(getDrivenBoneWorld: BoneWorldFn, rotClampAlpha = 0, alignAllPositions = false): (RigidTransform | null)[] {
    rotClampAlpha = f(rotClampAlpha);
    const model = this.model;
    const n = model.boneNames.length;
    const physRot: (Quat | null)[] = new Array(n).fill(null);
    const linkOf = new Map<RigidBody, BoneLink>();
    const linkOfBone: (BoneLink | null)[] = new Array(n).fill(null);
    for (const link of this.boneLinks) {
      linkOf.set(link.body, link);
      if (link.boneIndex >= 0 && link.boneIndex < n && link.mode !== PhysicsMode.BoneFollow) {
        physRot[link.boneIndex] = link.body.worldTransform.mul(link.bodyOffsetFromBone.inverse()).rotation;
        linkOfBone[link.boneIndex] = link;
      }
    }
    let parentJoint: Map<number, Joint> | null = null;
    if (rotClampAlpha > 0) {
      parentJoint = new Map();
      for (const j of this.world.joints) {
        if (!j.bodyB || !j.bodyA || !linkOf.has(j.bodyB) || !linkOf.has(j.bodyA)) continue;
        const bi = linkOf.get(j.bodyB)!.boneIndex;
        if (bi < 0 || bi >= n || !physRot[bi]) continue;
        const hierParent = bi < model.boneParents.length ? model.boneParents[bi] : -1;
        const isHier = linkOf.get(j.bodyA)!.boneIndex === hierParent;
        const cur = parentJoint.get(bi);
        if (!cur) parentJoint.set(bi, j);
        else if (isHier && linkOf.get(cur.bodyA!)!.boneIndex !== hierParent) parentJoint.set(bi, j);
      }
    }
    const eulerQ = (e: Vec3) => {
      const sx = f(Math.sin(f(e.x * 0.5))), cx = f(Math.cos(f(e.x * 0.5)));
      const sy = f(Math.sin(f(e.y * 0.5))), cy = f(Math.cos(f(e.y * 0.5)));
      const sz = f(Math.sin(f(e.z * 0.5))), cz = f(Math.cos(f(e.z * 0.5)));
      return new Quat(sx, 0, 0, cx).mul(new Quat(0, sy, 0, cy)).mul(new Quat(0, 0, sz, cz));
    };

    const world: (RigidTransform | null)[] = new Array(n).fill(null);
    const align = (i: number, depth: number): RigidTransform => {
      const w = world[i];
      if (w) return w;
      if (depth > 512) return (world[i] = new RigidTransform(Quat.Identity, model.bonePositions[i]));
      let res: RigidTransform;
      const p = i < model.boneParents.length ? model.boneParents[i] : -1;
      const pr = physRot[i];
      if (!pr) {
        const driven = getDrivenBoneWorld(i);
        if (driven) res = driven;
        else if (p < 0 || p >= n) res = new RigidTransform(Quat.Identity, model.bonePositions[i]);
        else {
          const pw = align(p, depth + 1);
          res = new RigidTransform(pw.rotation, pw.rotation.rotate(model.bonePositions[i].sub(model.bonePositions[p])).add(pw.origin));
        }
      } else {
        const myLink = linkOfBone[i];
        if (!alignAllPositions && myLink && myLink.mode !== PhysicsMode.DynamicBoneMerge) {
          return (world[i] = myLink.body.worldTransform.mul(myLink.bodyOffsetFromBone.inverse()));
        }
        let rot = pr;
        const pj = parentJoint ? parentJoint.get(i) : undefined;
        if (pj) {
          const aLink = linkOf.get(pj.bodyA!)!;
          let aBonePose: RigidTransform;
          if (aLink.boneIndex >= 0 && aLink.boneIndex < n) aBonePose = align(aLink.boneIndex, depth + 1);
          else aBonePose = pj.bodyA!.worldTransform.mul(aLink.bodyOffsetFromBone.inverse());
          const qA = aBonePose.mul(aLink.bodyOffsetFromBone).rotation.mul(pj.frameInA.rotation).normalized;
          const bLink = linkOf.get(pj.bodyB!)!;
          const qB = rot.mul(bLink.bodyOffsetFromBone.rotation).mul(pj.frameInB.rotation).normalized;
          const eu = Joint.toEulerXYZ(qA.conjugated().mul(qB).normalized);
          let ec = eu;
          let changed = false;
          for (let d = 0; d < 3; d++) {
            const lo = pj.angularLowerLimit.get(d), hi = pj.angularUpperLimit.get(d);
            if (lo > hi) continue;
            const e = ec.get(d);
            if (e < lo) {
              ec = ec.with(d, f(e + f(rotClampAlpha * f(lo - e))));
              changed = true;
            } else if (e > hi) {
              ec = ec.with(d, f(e + f(rotClampAlpha * f(hi - e))));
              changed = true;
            }
          }
          if (changed) {
            const qBnew = qA.mul(eulerQ(ec));
            rot = qBnew.mul(pj.frameInB.rotation.conjugated()).mul(bLink.bodyOffsetFromBone.rotation.conjugated());
          }
        }
        if (p < 0 || p >= n) res = new RigidTransform(rot, model.bonePositions[i]);
        else {
          const pw = align(p, depth + 1);
          res = new RigidTransform(rot, pw.rotation.rotate(model.bonePositions[i].sub(model.bonePositions[p])).add(pw.origin));
        }
      }
      world[i] = res;
      return res;
    };
    for (let i = 0; i < n; i++) if (!world[i]) align(i, 0);
    return world;
  }

  private buildJoints(model: PmxPhysicsModel) {
    for (const pj of model.joints) {
      const a = this.validBody(pj.rigidBodyAIndex);
      const b = this.validBody(pj.rigidBodyBIndex);
      if (!a || !b) continue;
      const worldFrame = RigidTransform.fromEuler(pj.position, pj.rotation);
      const joint = Joint.fromPmx(
        pj.jointType as JointType, a, b, worldFrame,
        pj.linearLowerLimit, pj.linearUpperLimit,
        pj.angularLowerLimit, pj.angularUpperLimit,
        pj.springLinear, pj.springAngular,
      );
      joint.name = pj.name;
      this.world.addJoint(joint);
    }
  }

  private validBody(index: number) {
    return index >= 0 && index < this.bodies.length ? this.bodies[index] : null;
  }
}

function clampMass(mass: number) {
  const max = PmxPhysicsBuilder.maxDynamicMass;
  if (max <= 0) return mass;
  if (!(mass > max)) return mass;
  PmxPhysicsBuilder.clampedMassCount++;
  return f(max);
}

function computeOffset(model: PmxPhysicsModel, rb: PmxRigidBody): RigidTransform {
  const bodyWorld = RigidTransform.fromEuler(rb.position, rb.rotation);
  if (rb.boneIndex < 0 || rb.boneIndex >= model.bonePositions.length) return bodyWorld;
  const boneWorld = new RigidTransform(Quat.Identity, model.bonePositions[rb.boneIndex]);
  return boneWorld.inverseTimes(bodyWorld);
}

function createShape(rb: PmxRigidBody): CollisionShape {
  switch (rb.shapeType) {
    case 0: return new SphereShape(rb.size.x);
    case 1: return new BoxShape(rb.size);
    case 2: return new CapsuleShape(rb.size.x, rb.size.y);
    default: return new SphereShape(rb.size.x);
  }
}
