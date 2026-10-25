// ===========================================================================
// MMD physics engine (TypeScript port) – physics world
// Port of PhysicsWorld.cs (btDiscreteDynamicsWorld equivalent): gravity, integration,
// collision, joints; sequential impulses + Baumgarte. float32 rules: see math.ts.
// Profiling counters / diagnostic row hooks of the C# source are not ported
// (debugContacts is kept because the engine harness uses it).
// ===========================================================================

import { f, clamp, Quat, RigidTransform, Vec3 } from './math';
import { ContactPoint, GjkEpa, PersistentManifold } from './collision';
import { Joint } from './constraints';
import { RigidBody } from './rigidBody';

/** One contact constraint (normal + friction). Mutated in place (the C# code always writes the copy back). */
class ContactConstraint {
  a!: RigidBody;
  b!: RigidBody;
  relA: Vec3 = Vec3.Zero;
  relB: Vec3 = Vec3.Zero;
  normal: Vec3 = Vec3.Zero;
  tangent1: Vec3 = Vec3.Zero;
  tangent2: Vec3 = Vec3.Zero;
  normalMass = 0;
  tangentMass1 = 0;
  tangentMass2 = 0;
  normalBias = 0;
  pushBias = 0;
  friction = 0;
  normalImpulse = 0;
  tangentImpulse1 = 0;
  tangentImpulse2 = 0;
  pushImpulse = 0;
  manifold: PersistentManifold | null = null;
  pointRef = 0;
  useTangent2 = false;
  // scalar copies for the solver loop
  rax = 0; ray = 0; raz = 0;
  rbx = 0; rby = 0; rbz = 0;
  nx = 0; ny = 0; nz = 0;
}

const F_098 = f(f(-9.8) * 10);
const F_0005 = f(0.005);
const F_02 = f(0.2);
const F_085 = f(0.85);
const F_M002 = f(-0.02);
const F_08 = f(0.8);
const F_1EM4 = f(1e-4);
const F_1EM3 = f(1e-3);
const F_1EM6 = f(1e-6);
const F_1EM9 = f(1e-9);
const F_0001 = f(0.001);
const F_FLT_EPSILON = f(1.1920929e-7);
const F_0577 = f(0.577);
const F_COEF = f(0.020833333333);
const F_PI = f(Math.PI);
const F_2PI = f(2 * F_PI);
const ANGULAR_MOTION_THRESHOLD = f(0.5 * f(Math.PI * 0.5));

/** C# PairKey: ((long)min << 32) | max — kept as an exact JS number (indices < 2^21). */
function pairKey(a: number, b: number) {
  if (a > b) {
    const t = a;
    a = b;
    b = t;
  }
  return a * 4294967296 + b;
}

export class PhysicsWorld {
  gravity: Vec3 = new Vec3(0, F_098, 0);

  solverIterations = 10;
  jointVelocityIterations = 0;
  jointMaxCorrectionVel = 0;
  subSteps = 4;
  private _fixedTimeStep = f(1 / 30);
  /** Rounded to float32 on assignment (C# field is float). */
  get fixedTimeStep() {
    return this._fixedTimeStep;
  }
  set fixedTimeStep(v: number) {
    this._fixedTimeStep = f(v);
  }

  penetrationSlop = F_0005;
  baumgarteFactor = F_02;
  contactRhsBullet = true;
  restitutionThreshold = 1;

  useSplitImpulse = false;
  useJointSplitImpulse = false;
  useJointWarmStart = false;
  useJointWarmStartAngular = false;
  splitImpulsePenetrationThreshold = F_M002;

  solveJointsFirst = false;

  /** Number of internal steps run by the last stepSimulation (0 = accumulate only). */
  lastStepsRun = 0;

  contactWarmStartFactor = F_085;
  contactNormalBeforeFriction = false;
  contactPoolOrder = false;
  frictionVelocityAligned = true;
  frictionCombineMultiply = true;

  readonly bodies: RigidBody[] = [];
  readonly joints: Joint[] = [];

  private readonly manifolds = new Map<number, PersistentManifold>();
  private readonly contacts: ContactConstraint[] = [];
  private readonly detectBuffer: ContactPoint[] = [];
  private accumulator = 0;

  /** Optional hook (default null): (bodyA name, bodyB name, distance, normal impulse) per stored contact. */
  debugContacts: { a: string; b: string; dist: number; ni: number }[] | null = null;
  debugContactCount = 0;

  addBody(b: RigidBody) {
    b.index = this.bodies.length;
    if (b.isStaticOrKinematic) b.kinematicTarget = b.worldTransform;
    this.bodies.push(b);
    this.pairCount = -1;
  }

  addJoint(j: Joint) {
    this.joints.push(j);
  }

  /** Clears contact manifolds (incl. accumulated impulses) and the time accumulator. */
  clearContacts() {
    this.manifolds.clear();
    this.contacts.length = 0;
    this.accumulator = 0;
  }

  private frameKinStart: RigidTransform[] | null = null;

  stepSimulation(deltaTime: number) {
    deltaTime = f(deltaTime);
    if (this.jointMaxCorrectionVel > 0) Joint.maxCorrectionVel = this.jointMaxCorrectionVel;

    this.accumulator = f(this.accumulator + deltaTime);
    const fts = this._fixedTimeStep;

    let stepsToRun = 0;
    {
      let rem = this.accumulator;
      while (rem >= fts && stepsToRun < 8) {
        rem = f(rem - fts);
        stepsToRun++;
      }
    }
    this.lastStepsRun = stepsToRun;
    if (stepsToRun === 0) return;

    const n = this.bodies.length;
    if (!this.frameKinStart || this.frameKinStart.length < n) {
      const arr: RigidTransform[] = new Array(n);
      // C# `new RigidTransform[n]` is zero-initialised (default struct: rotation all zero).
      for (let i = 0; i < n; i++) arr[i] = ZERO_TRANSFORM;
      this.frameKinStart = arr;
    }
    for (let i = 0; i < n; i++) if (this.bodies[i].isKinematic) this.frameKinStart[i] = this.bodies[i].worldTransform;

    const totalSub = stepsToRun * this.subSteps;
    for (let k = 0; k < stepsToRun; k++) {
      this.internalStep(fts, k * this.subSteps, totalSub);
      this.accumulator = f(this.accumulator - fts);
    }
  }

  private internalStep(dt: number, gBase: number, totalSub: number) {
    const sub = f(dt / this.subSteps);
    for (let s = 0; s < this.subSteps; s++) this.subStep(sub, f(f(gBase + s + 1) / f(totalSub)));
  }

  private subStep(dt: number, frac: number) {
    const bodies = this.bodies;
    const kinStart = this.frameKinStart!;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i];
      if (!b.isKinematic) continue;
      b.kinematicStepTarget = interpTransform(kinStart[i], b.kinematicTarget, frac);
    }

    this.integrateVelocities(dt);
    this.broadphaseNarrowphase();
    this.buildContactConstraints(dt);

    Joint.solverIterationsForSpring = this.solverIterations;
    for (const j of this.joints) j.prepare(dt, this.useJointSplitImpulse, this.useJointWarmStart, this.useJointWarmStartAngular);
    for (const j of this.joints) j.applySprings(dt);

    this.warmStart();

    const jointIters = this.jointVelocityIterations > 0 ? this.jointVelocityIterations : this.solverIterations;
    const totalIters = Math.max(this.solverIterations, jointIters);
    for (let it = 0; it < totalIters; it++) {
      const doContacts = it < this.solverIterations;
      const doJoints = it < jointIters;
      if (this.solveJointsFirst) {
        if (doJoints) for (const j of this.joints) j.solveVelocity();
        if (doContacts) this.solveContacts();
      } else {
        if (doContacts) this.solveContacts();
        if (doJoints) for (const j of this.joints) j.solveVelocity();
      }
    }

    if (this.useSplitImpulse || this.useJointSplitImpulse || Joint.splitCrossOnly || Joint.splitChainOnly) {
      for (let i = 0; i < bodies.length; i++) {
        bodies[i].pseudoLinearVelocity = Vec3.Zero;
        bodies[i].pseudoAngularVelocity = Vec3.Zero;
      }
      for (let it = 0; it < this.solverIterations; it++) {
        if (this.useSplitImpulse) this.solveSplitImpulse();
        if (this.useJointSplitImpulse || Joint.splitCrossOnly || Joint.splitChainOnly)
          for (const j of this.joints) j.solveSplitPosition();
      }
    }

    this.storeImpulses();
    this.integratePositions(dt);
    if (this.enableSleeping) this.updateSleeping(dt);
  }

  // ---- sleeping (Bullet deactivation, island based; default OFF) ----
  enableSleeping = false;
  linearSleepThreshold = F_08;
  angularSleepThreshold = 1;
  deactivationTime = 2;
  kinematicMotionEpsilon = F_1EM4;
  sleepingBodyCount = 0;

  private uf: Int32Array | null = null;
  private wantsSleep: boolean[] = [];
  private islandBlocked: boolean[] = [];
  private islandWants: boolean[] = [];

  private ufFind(x: number) {
    const uf = this.uf!;
    while (uf[x] !== x) {
      uf[x] = uf[uf[x]];
      x = uf[x];
    }
    return x;
  }

  private ufUnion(a: number, b: number) {
    a = this.ufFind(a);
    b = this.ufFind(b);
    if (a !== b) this.uf![a] = b;
  }

  private updateSleeping(dt: number) {
    const bodies = this.bodies;
    const n = bodies.length;
    if (!this.uf || this.uf.length < n) {
      this.uf = new Int32Array(n);
      this.wantsSleep = new Array(n).fill(false);
      this.islandBlocked = new Array(n).fill(false);
      this.islandWants = new Array(n).fill(false);
    }
    const lt2 = f(this.linearSleepThreshold * this.linearSleepThreshold);
    const at2 = f(this.angularSleepThreshold * this.angularSleepThreshold);

    for (let i = 0; i < n; i++) {
      const b = bodies[i];
      this.uf[i] = i;
      this.islandBlocked[i] = false;
      this.islandWants[i] = true;
      this.wantsSleep[i] = false;
      if (b.isStaticOrKinematic) continue;
      if (b.linearVelocity.lengthSquared < lt2 && b.angularVelocity.lengthSquared < at2) b.sleepTimer = f(b.sleepTimer + dt);
      else {
        b.sleepTimer = 0;
        b.isActive = true;
      }
      this.wantsSleep[i] = b.sleepTimer > this.deactivationTime;
    }

    for (const j of this.joints) {
      if (!j.bodyA || !j.bodyB) continue;
      if (j.bodyA.isStaticOrKinematic || j.bodyB.isStaticOrKinematic) continue;
      this.ufUnion(j.bodyA.index, j.bodyB.index);
    }
    for (const c of this.contacts) {
      if (c.a.isStaticOrKinematic || c.b.isStaticOrKinematic) continue;
      this.ufUnion(c.a.index, c.b.index);
    }

    const eps2 = f(this.kinematicMotionEpsilon * this.kinematicMotionEpsilon);
    const kinMoving = (k: RigidBody) => k.linearVelocity.lengthSquared > eps2 || k.angularVelocity.lengthSquared > eps2;

    for (const j of this.joints) {
      if (!j.bodyA || !j.bodyB) continue;
      if (j.bodyA.isStaticOrKinematic && !j.bodyB.isStaticOrKinematic) {
        if (kinMoving(j.bodyA)) this.islandBlocked[this.ufFind(j.bodyB.index)] = true;
      } else if (j.bodyB.isStaticOrKinematic && !j.bodyA.isStaticOrKinematic) {
        if (kinMoving(j.bodyB)) this.islandBlocked[this.ufFind(j.bodyA.index)] = true;
      }
    }
    for (const c of this.contacts) {
      const a = c.a, b2 = c.b;
      if (a.isStaticOrKinematic && !b2.isStaticOrKinematic) {
        if (kinMoving(a)) this.islandBlocked[this.ufFind(b2.index)] = true;
      } else if (b2.isStaticOrKinematic && !a.isStaticOrKinematic) {
        if (kinMoving(b2)) this.islandBlocked[this.ufFind(a.index)] = true;
      }
    }

    for (let i = 0; i < n; i++) {
      if (bodies[i].isStaticOrKinematic) continue;
      if (!this.wantsSleep[i]) this.islandWants[this.ufFind(i)] = false;
    }

    let sleeping = 0;
    for (let i = 0; i < n; i++) {
      const b = bodies[i];
      if (b.isStaticOrKinematic) continue;
      const r = this.ufFind(i);
      const sleep = this.islandWants[r] && !this.islandBlocked[r];
      if (sleep) {
        b.isActive = false;
        b.linearVelocity = Vec3.Zero;
        b.angularVelocity = Vec3.Zero;
        sleeping++;
      } else if (!b.isActive) {
        b.isActive = true;
        b.sleepTimer = 0;
      }
    }
    this.sleepingBodyCount = sleeping;
  }

  private integrateVelocities(dt: number) {
    for (const b of this.bodies) {
      if (b.isStaticOrKinematic) {
        if (b.isKinematic) {
          const cur = b.worldTransform;
          const tgt = b.kinematicStepTarget;
          b.linearVelocity = tgt.origin.sub(cur.origin).div(dt);
          const dq = tgt.rotation.mul(cur.rotation.conjugated());
          b.angularVelocity = quatToAngularVelocity(dq, dt);
        }
        continue;
      }
      if (this.enableSleeping && !b.isActive) {
        b.clearForces();
        continue;
      }

      b.linearVelocity = b.linearVelocity.add(this.gravity.add(b.totalForce.mul(b.inverseMass)).mul(dt));
      b.angularVelocity = b.angularVelocity.add(b.inverseInertiaWorld.mulVec(b.totalTorque).mul(dt));

      b.linearVelocity = b.linearVelocity.mul(dampingFactor(b.linearDamping, dt));
      b.angularVelocity = b.angularVelocity.mul(dampingFactor(b.angularDamping, dt));

      b.clearForces();
    }
  }

  /** Damping clamp upper bound (Bullet [0,1]). */
  static dampingClampMax = 1;
  /** Task 34: Bullet 2.75 exponential-map orientation integration (default true). */
  static bulletRotationIntegration = true;

  private integratePositions(dt: number) {
    const split = this.useSplitImpulse || this.useJointSplitImpulse;
    for (const b of this.bodies) {
      if (b.isStaticOrKinematic) {
        if (b.isKinematic) {
          b.worldTransform = b.kinematicStepTarget;
          b.updateInertiaWorld();
        }
        continue;
      }
      if (this.enableSleeping && !b.isActive) continue;

      const t = b.worldTransform;
      let vlin = b.linearVelocity;
      let vang = b.angularVelocity;
      if (split) {
        vlin = vlin.add(b.pseudoLinearVelocity);
        vang = vang.add(b.pseudoAngularVelocity);
      }
      const origin = t.origin.add(vlin.mul(dt));

      const w = vang;
      let rotation: Quat;
      if (PhysicsWorld.bulletRotationIntegration) {
        let fAngle = f(Math.sqrt(f(f(f(w.x * w.x) + f(w.y * w.y)) + f(w.z * w.z))));
        if (f(fAngle * dt) > ANGULAR_MOTION_THRESHOLD) fAngle = f(ANGULAR_MOTION_THRESHOLD / dt);
        const sc =
          fAngle < F_0001
            ? f(f(0.5 * dt) - f(f(f(f(f(f(dt * dt) * dt) * F_COEF) * fAngle) * fAngle)))
            : f(f(Math.sin(f(f(0.5 * fAngle) * dt))) / fAngle);
        const dorn = new Quat(f(w.x * sc), f(w.y * sc), f(w.z * sc), f(Math.cos(f(f(fAngle * dt) * 0.5))));
        rotation = dorn.mul(t.rotation).normalized;
      } else {
        const r = t.rotation;
        const spin = new Quat(w.x, w.y, w.z, 0).mul(r);
        rotation = new Quat(
          f(r.x + f(f(spin.x * 0.5) * dt)),
          f(r.y + f(f(spin.y * 0.5) * dt)),
          f(r.z + f(f(spin.z * 0.5) * dt)),
          f(r.w + f(f(spin.w * 0.5) * dt)),
        ).normalized;
      }

      b.worldTransform = new RigidTransform(rotation, origin);
      b.updateInertiaWorld();
    }
  }

  // ---- broadphase + narrowphase ----
  private pairA: Int32Array | null = null;
  private pairB: Int32Array | null = null;
  private pairCount = -1;
  private pairBuiltForCount = -1;
  private readonly seenScratch = new Set<number>();
  private bbScratch: Float64Array | null = null;
  private orgScratch: Float64Array | null = null;
  private sphRScratch: Float64Array | null = null;
  private thrScratch: Float64Array | null = null;

  /** Call after changing Group / CollisionMask / Mode at runtime. */
  invalidateCollisionPairs() {
    this.pairCount = -1;
  }

  get debugCollisionPairCount() {
    if (this.pairCount < 0 || this.pairBuiltForCount !== this.bodies.length) this.buildCollisionPairs();
    return this.pairCount;
  }

  private buildCollisionPairs() {
    const n = this.bodies.length;
    const cap = (n * (n - 1)) / 2;
    if (!this.pairA || this.pairA.length < cap) {
      this.pairA = new Int32Array(cap);
      this.pairB = new Int32Array(cap);
    }
    let c = 0;
    for (let i = 0; i < n; i++)
      for (let k = i + 1; k < n; k++) {
        const a = this.bodies[i], b = this.bodies[k];
        if (a.isStaticOrKinematic && b.isStaticOrKinematic) continue;
        if (!PhysicsWorld.shouldCollide(a, b)) continue;
        this.pairA[c] = i;
        this.pairB![c] = k;
        c++;
      }
    this.pairCount = c;
    this.pairBuiltForCount = n;
  }

  /** Issue #2 broadphase early rejection (bit-identical to the plain path). */
  useBroadphasePrefilter = true;
  static readonly broadphaseSphereSlack = F_1EM3;

  private broadphaseNarrowphase() {
    const bodies = this.bodies;
    const n = bodies.length;
    if (this.pairCount < 0 || this.pairBuiltForCount !== n) this.buildCollisionPairs();

    // Scratch arrays (C#: _aabbScratch / _sphereRadiusScratch / _acceptThrScratch).
    if (!this.bbScratch || this.bbScratch.length < n * 6) {
      this.bbScratch = new Float64Array(n * 6);
      this.orgScratch = new Float64Array(n * 3);
      this.sphRScratch = new Float64Array(n);
      this.thrScratch = new Float64Array(n);
    }
    const bb = this.bbScratch, org = this.orgScratch!, sphR = this.sphRScratch!, thr = this.thrScratch!;
    const extra = f(GjkEpa.speculativeMargin - GjkEpa.speculativeMarginDefault);
    for (let i = 0; i < n; i++) {
      // RigidBody.ComputeAabb (+ Aabb.Expand(extra) when the speculative band is widened).
      const body = bodies[i];
      const c = body.worldTransform.origin;
      const r = f(body.shape.boundingRadius + body.shape.margin);
      let x0 = f(c.x - r), y0 = f(c.y - r), z0 = f(c.z - r);
      let x1 = f(c.x + r), y1 = f(c.y + r), z1 = f(c.z + r);
      if (extra > 0) {
        x0 = f(x0 - extra); y0 = f(y0 - extra); z0 = f(z0 - extra);
        x1 = f(x1 + extra); y1 = f(y1 + extra); z1 = f(z1 + extra);
      }
      const o = i * 6;
      bb[o] = x0; bb[o + 1] = y0; bb[o + 2] = z0; bb[o + 3] = x1; bb[o + 4] = y1; bb[o + 5] = z1;
      org[i * 3] = c.x; org[i * 3 + 1] = c.y; org[i * 3 + 2] = c.z;
    }

    const prefilter = this.useBroadphasePrefilter;
    if (prefilter)
      for (let i = 0; i < n; i++) {
        const s = bodies[i].shape;
        sphR[i] = s.boundingRadius;
        thr[i] = GjkEpa.acceptThresholdOf(s);
      }

    const seen = this.seenScratch;
    seen.clear();
    const pairA = this.pairA!, pairB = this.pairB!;
    const detectBuffer = this.detectBuffer;
    const slack = PhysicsWorld.broadphaseSphereSlack;
    for (let p = 0; p < this.pairCount; p++) {
      const i = pairA[p], k = pairB[p];
      if (prefilter) {
        const lim = f(f(f(sphR[i] + sphR[k]) + Math.min(thr[i], thr[k])) + slack);
        const dx = f(org[k * 3] - org[i * 3]), dy = f(org[k * 3 + 1] - org[i * 3 + 1]), dz = f(org[k * 3 + 2] - org[i * 3 + 2]);
        if (f(f(f(dx * dx) + f(dy * dy)) + f(dz * dz)) >= f(lim * lim)) continue;
      }
      // Aabb.Intersects
      const oi = i * 6, ok = k * 6;
      if (!(bb[oi] <= bb[ok + 3] && bb[oi + 3] >= bb[ok] &&
            bb[oi + 1] <= bb[ok + 4] && bb[oi + 4] >= bb[ok + 1] &&
            bb[oi + 2] <= bb[ok + 5] && bb[oi + 5] >= bb[ok + 2])) continue;
      const a = bodies[i], b = bodies[k];

      const key = pairKey(a.index, b.index);
      let m = this.manifolds.get(key);
      detectBuffer.length = 0;
      GjkEpa.detect(a, b, detectBuffer);
      if (!m) {
        if (prefilter && detectBuffer.length === 0) continue;
        m = new PersistentManifold(a, b);
        this.manifolds.set(key, m);
      }
      seen.add(key);
      m.refresh();
      for (let di = 0; di < detectBuffer.length; di++) m.addPoint(detectBuffer[di]);
      m.pruneStaleDeep();
    }
    if (this.manifolds.size > seen.size) {
      const dead: number[] = [];
      for (const key of this.manifolds.keys()) if (!seen.has(key)) dead.push(key);
      for (const key of dead) this.manifolds.delete(key);
    }
  }

  /** PMX collision filter: (maskB & bit(groupA)) && (maskA & bit(groupB)). */
  static shouldCollide(a: RigidBody, b: RigidBody) {
    return (b.collisionMask & (1 << a.group)) !== 0 && (a.collisionMask & (1 << b.group)) !== 0;
  }

  private buildContactConstraints(dt: number) {
    const contacts = this.contacts;
    contacts.length = 0;

    // Keys are unique, so sorting by PairKey gives the same order as the C# (unstable) List.Sort.
    const sorted: [number, PersistentManifold][] = [];
    for (const [key, m] of this.manifolds) sorted.push([key, m]);
    sorted.sort((x, y) => x[0] - y[0]);

    for (const [, m] of sorted) {
      for (let p = 0; p < m.points.length; p++) {
        const cp = m.points[p];
        const a = m.bodyA, b = m.bodyB;

        const rA = cp.positionWorldA.sub(a.centerOfMass);
        const rB = cp.positionWorldB.sub(b.centerOfMass);
        const n = cp.normal;

        let [t1, t2] = buildTangentBasis(n);
        let useT2 = true;
        if (this.frictionVelocityAligned) {
          const vrel = b.velocityAtPoint(cp.positionWorldB).sub(a.velocityAtPoint(cp.positionWorldA));
          const lat = vrel.sub(n.mul(vrel.dot(n)));
          const l2 = lat.lengthSquared;
          if (l2 > F_FLT_EPSILON) {
            t1 = lat.div(f(Math.sqrt(l2)));
            t2 = Vec3.cross(t1, n);
          }
          useT2 = false;
        }

        const cc = new ContactConstraint();
        cc.a = a;
        cc.b = b;
        cc.relA = rA;
        cc.relB = rB;
        cc.normal = n;
        cc.tangent1 = t1;
        cc.tangent2 = t2;
        const fa = Math.max(0, a.friction), fb = Math.max(0, b.friction);
        cc.friction = this.frictionCombineMultiply ? f(fa * fb) : f(Math.sqrt(f(fa * fb)));
        cc.normalMass = effectiveMass(a, b, rA, rB, n);
        cc.tangentMass1 = effectiveMass(a, b, rA, rB, t1);
        cc.tangentMass2 = effectiveMass(a, b, rA, rB, t2);
        cc.normalImpulse = cp.normalImpulse;
        cc.tangentImpulse1 = cp.tangentImpulse1;
        cc.tangentImpulse2 = cp.tangentImpulse2;
        cc.manifold = m;
        cc.pointRef = p;
        cc.useTangent2 = useT2;
        cc.rax = rA.x; cc.ray = rA.y; cc.raz = rA.z;
        cc.rbx = rB.x; cc.rby = rB.y; cc.rbz = rB.z;
        cc.nx = n.x; cc.ny = n.y; cc.nz = n.z;

        const relN = b.velocityAtPoint(cp.positionWorldB).sub(a.velocityAtPoint(cp.positionWorldA)).dot(n);
        const rest = f(Math.sqrt(f(Math.max(0, a.restitution) * Math.max(0, b.restitution))));
        const restBias = -relN > this.restitutionThreshold ? f(rest * -relN) : 0;

        if (this.contactRhsBullet) {
          const penB = cp.distance;
          const posErr = f(f(-penB * this.baumgarteFactor) / dt);
          if (this.useSplitImpulse && penB <= this.splitImpulsePenetrationThreshold) {
            cc.normalBias = restBias;
            cc.pushBias = posErr;
          } else {
            cc.normalBias = f(posErr + restBias);
            cc.pushBias = 0;
          }
        } else if (cp.distance <= 0) {
          const pen = f(-cp.distance - this.penetrationSlop);
          const biasVel = pen > 0 ? f(f(this.baumgarteFactor * pen) / dt) : 0;
          if (this.useSplitImpulse && cp.distance <= this.splitImpulsePenetrationThreshold) {
            cc.normalBias = restBias;
            cc.pushBias = biasVel;
          } else {
            cc.normalBias = Math.max(biasVel, restBias);
            cc.pushBias = 0;
          }
        } else {
          const speculative = f(-cp.distance / dt);
          cc.normalBias = Math.min(speculative, restBias);
        }
        contacts.push(cc);
      }
    }
    this.debugContactCount = contacts.length;
  }

  private storeImpulses() {
    const contacts = this.contacts;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i];
      if (!c.manifold || c.pointRef >= c.manifold.points.length) continue;
      const cp = c.manifold.points[c.pointRef];
      cp.normalImpulse = c.normalImpulse;
      if (!this.frictionVelocityAligned) {
        cp.tangentImpulse1 = c.tangentImpulse1;
        cp.tangentImpulse2 = c.tangentImpulse2;
      }
      this.debugContacts?.push({ a: c.a.name, b: c.b.name, dist: cp.distance, ni: c.normalImpulse });
    }
  }

  private warmStart() {
    const wf = this.contactWarmStartFactor;
    const contacts = this.contacts;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i];
      if (this.frictionVelocityAligned) {
        c.tangentImpulse1 = 0;
        c.tangentImpulse2 = 0;
      }
      if (wf !== 1) {
        c.normalImpulse = f(c.normalImpulse * wf);
        c.tangentImpulse1 = f(c.tangentImpulse1 * wf);
        c.tangentImpulse2 = f(c.tangentImpulse2 * wf);
      }
      const P = c.normal.mul(c.normalImpulse).add(c.tangent1.mul(c.tangentImpulse1)).add(c.tangent2.mul(c.tangentImpulse2));
      c.a.applyImpulse(P.neg(), c.relA);
      c.b.applyImpulse(P, c.relB);
    }
  }

  private solveContacts() {
    const contacts = this.contacts;
    if (this.contactPoolOrder) {
      for (let i = 0; i < contacts.length; i++) solveNormal(contacts[i]);
      for (let i = 0; i < contacts.length; i++) {
        const c = contacts[i];
        c.tangentImpulse1 = solveFriction(c, c.tangent1, c.tangentMass1, c.tangentImpulse1);
        if (c.useTangent2) c.tangentImpulse2 = solveFriction(c, c.tangent2, c.tangentMass2, c.tangentImpulse2);
      }
      return;
    }
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i];
      if (!this.contactNormalBeforeFriction) {
        c.tangentImpulse1 = solveFriction(c, c.tangent1, c.tangentMass1, c.tangentImpulse1);
        if (c.useTangent2) c.tangentImpulse2 = solveFriction(c, c.tangent2, c.tangentMass2, c.tangentImpulse2);
        solveNormal(c);
      } else {
        solveNormal(c);
        c.tangentImpulse1 = solveFriction(c, c.tangent1, c.tangentMass1, c.tangentImpulse1);
        if (c.useTangent2) c.tangentImpulse2 = solveFriction(c, c.tangent2, c.tangentMass2, c.tangentImpulse2);
      }
    }
  }

  private solveSplitImpulse() {
    const contacts = this.contacts;
    for (let i = 0; i < contacts.length; i++) {
      const c = contacts[i];
      if (c.pushBias <= 0) continue;
      const a = c.a, b = c.b;
      const pA = a.centerOfMass.add(c.relA);
      const pB = b.centerOfMass.add(c.relB);
      const relN = b.pseudoVelocityAtPoint(pB).sub(a.pseudoVelocityAtPoint(pA)).dot(c.normal);
      let dP = f(f(c.pushBias - relN) * c.normalMass);
      const old = c.pushImpulse;
      c.pushImpulse = Math.max(0, f(old + dP));
      dP = f(c.pushImpulse - old);
      const P = c.normal.mul(dP);
      a.applyPushImpulse(P.neg(), c.relA);
      b.applyPushImpulse(P, c.relB);
    }
  }
}

/** default(RigidTransform): all-zero rotation and origin. */
const ZERO_TRANSFORM = new RigidTransform(Quat.Zero, Vec3.Zero);

/**
 * (b.VelocityAtPoint(b.com + relB) - a.VelocityAtPoint(a.com + relA)) in scalar form, written to REL.
 * Note VelocityAtPoint recomputes r = (com + rel) - com, which is not always bit-equal to rel.
 */
const REL = new Float64Array(3);
/** Impulse P = IMP[0..2] applied as A: -P at relA, B: +P at relB (RigidBody.applyImpulse twice, args via scratch to avoid boxing). */
const IMP = new Float64Array(3);
function applyPair(c: ContactConstraint) {
  const px = IMP[0], py = IMP[1], pz = IMP[2];
  applyImp(c.a, -px, -py, -pz, c.rax, c.ray, c.raz);
  applyImp(c.b, px, py, pz, c.rbx, c.rby, c.rbz);
}
function applyImp(B: RigidBody, px: number, py: number, pz: number, rx: number, ry: number, rz: number) {
  const im = B.inverseMass;
  if (im === 0) return;
  B.lvx = f(B.lvx + f(px * im));
  B.lvy = f(B.lvy + f(py * im));
  B.lvz = f(B.lvz + f(pz * im));
  const cx = f(f(ry * pz) - f(rz * py));
  const cy = f(f(rz * px) - f(rx * pz));
  const cz = f(f(rx * py) - f(ry * px));
  B.avx = f(B.avx + f(f(f(B.i00 * cx) + f(B.i01 * cy)) + f(B.i02 * cz)));
  B.avy = f(B.avy + f(f(f(B.i10 * cx) + f(B.i11 * cy)) + f(B.i12 * cz)));
  B.avz = f(B.avz + f(f(f(B.i20 * cx) + f(B.i21 * cy)) + f(B.i22 * cz)));
}
function relVelocity(c: ContactConstraint) {
  const a = c.a, b = c.b;
  const ao = a.worldTransform.origin, bo = b.worldTransform.origin;
  const rax = f(f(ao.x + c.rax) - ao.x), ray = f(f(ao.y + c.ray) - ao.y), raz = f(f(ao.z + c.raz) - ao.z);
  const rbx = f(f(bo.x + c.rbx) - bo.x), rby = f(f(bo.y + c.rby) - bo.y), rbz = f(f(bo.z + c.rbz) - bo.z);
  const vbx = f(b.lvx + f(f(b.avy * rbz) - f(b.avz * rby)));
  const vby = f(b.lvy + f(f(b.avz * rbx) - f(b.avx * rbz)));
  const vbz = f(b.lvz + f(f(b.avx * rby) - f(b.avy * rbx)));
  const vax = f(a.lvx + f(f(a.avy * raz) - f(a.avz * ray)));
  const vay = f(a.lvy + f(f(a.avz * rax) - f(a.avx * raz)));
  const vaz = f(a.lvz + f(f(a.avx * ray) - f(a.avy * rax)));
  REL[0] = f(vbx - vax);
  REL[1] = f(vby - vay);
  REL[2] = f(vbz - vaz);
}

function solveNormal(c: ContactConstraint) {
  relVelocity(c);
  const nx = c.nx, ny = c.ny, nz = c.nz;
  const relN = f(f(f(REL[0] * nx) + f(REL[1] * ny)) + f(REL[2] * nz));
  let dPn = f(f(c.normalBias - relN) * c.normalMass);
  const oldN = c.normalImpulse;
  c.normalImpulse = Math.max(0, f(oldN + dPn));
  dPn = f(c.normalImpulse - oldN);
  IMP[0] = f(nx * dPn); IMP[1] = f(ny * dPn); IMP[2] = f(nz * dPn);
  applyPair(c);
}

/** Returns the new accumulated impulse (C# `ref float accum`). */
function solveFriction(c: ContactConstraint, tangent: Vec3, mass: number, accum: number) {
  relVelocity(c);
  const tx = tangent.x, ty = tangent.y, tz = tangent.z;
  const relT = f(f(f(REL[0] * tx) + f(REL[1] * ty)) + f(REL[2] * tz));
  let dPt = f(-relT * mass);
  const maxF = f(c.friction * c.normalImpulse);
  const old = accum;
  accum = Math.max(-maxF, Math.min(maxF, f(old + dPt)));
  dPt = f(accum - old);
  IMP[0] = f(tx * dPt); IMP[1] = f(ty * dPt); IMP[2] = f(tz * dPt);
  applyPair(c);
  return accum;
}

function effectiveMass(a: RigidBody, b: RigidBody, rA: Vec3, rB: Vec3, dir: Vec3) {
  const rAxn = Vec3.cross(rA, dir);
  const rBxn = Vec3.cross(rB, dir);
  const k = f(
    f(f(a.inverseMass + b.inverseMass) + rAxn.dot(a.inverseInertiaWorld.mulVec(rAxn))) + rBxn.dot(b.inverseInertiaWorld.mulVec(rBxn)),
  );
  return k > 0 ? f(1 / k) : 0;
}

function buildTangentBasis(n: Vec3): [Vec3, Vec3] {
  const t1 = Math.abs(n.x) >= F_0577 ? new Vec3(n.y, -n.x, 0).normalized : new Vec3(0, n.z, -n.y).normalized;
  return [t1, Vec3.cross(n, t1)];
}

function dampingFactor(damping: number, dt: number) {
  const d = clamp(damping, 0, PhysicsWorld.dampingClampMax);
  return f(Math.pow(f(1 - d), dt));
}

function quatToAngularVelocity(dq: Quat, dt: number): Vec3 {
  dq = dq.normalized;
  let angle = f(2 * f(Math.acos(clamp(dq.w, -1, 1))));
  if (angle < F_1EM6) return Vec3.Zero;
  if (angle > Math.PI) angle = f(angle - F_2PI);
  const axis = new Vec3(dq.x, dq.y, dq.z);
  const len = axis.length;
  if (len < F_1EM9) return Vec3.Zero;
  return axis.div(len).mul(f(angle / dt));
}

function interpTransform(from: RigidTransform, to: RigidTransform, frac: number) {
  return new RigidTransform(Quat.slerp(from.rotation, to.rotation, frac), from.origin.add(to.origin.sub(from.origin).mul(frac)));
}
