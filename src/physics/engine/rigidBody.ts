// ===========================================================================
// MMD physics engine (TypeScript port) – rigid body
// Port of RigidBody.cs (btRigidBody equivalent). float32 emulation rules: see math.ts.
// ===========================================================================

import { f, Aabb, Matrix3x3, RigidTransform, Vec3 } from './math';
import { CollisionShape } from './shapes';

/** PMX rigid body physics mode. */
export enum PhysicsMode {
  BoneFollow = 0,
  Dynamic = 1,
  DynamicBoneMerge = 2,
}

export class RigidBody {
  name = '';
  index = -1;
  boneIndex = -1;

  /** PMX group (0..15) and collision mask (bit = 1: collides with that group). */
  group = 0;
  collisionMask = 0;

  shape: CollisionShape;
  mode: PhysicsMode = PhysicsMode.Dynamic;

  mass = 0;
  linearDamping = 0;
  angularDamping = 0;
  restitution = 0;
  friction = 0;

  worldTransform: RigidTransform = RigidTransform.Identity;

  // Velocities are stored as float32-valued scalars so the solver hot loops do not allocate.
  // linearVelocity / angularVelocity are value-semantics views (a fresh Vec3 per read).
  /** @internal */ lvx = 0;
  /** @internal */ lvy = 0;
  /** @internal */ lvz = 0;
  /** @internal */ avx = 0;
  /** @internal */ avy = 0;
  /** @internal */ avz = 0;
  get linearVelocity(): Vec3 {
    return new Vec3(this.lvx, this.lvy, this.lvz);
  }
  set linearVelocity(v: Vec3) {
    this.lvx = v.x;
    this.lvy = v.y;
    this.lvz = v.z;
  }
  get angularVelocity(): Vec3 {
    return new Vec3(this.avx, this.avy, this.avz);
  }
  set angularVelocity(v: Vec3) {
    this.avx = v.x;
    this.avy = v.y;
    this.avz = v.z;
  }

  pseudoLinearVelocity: Vec3 = Vec3.Zero;
  pseudoAngularVelocity: Vec3 = Vec3.Zero;

  kinematicTarget: RigidTransform = RigidTransform.Identity;
  kinematicStepTarget: RigidTransform = RigidTransform.Identity;

  localInertiaDiag: Vec3 = Vec3.Zero;

  inverseMass = 0;
  private _inverseInertiaWorld: Matrix3x3 = Matrix3x3.Zero;
  get inverseInertiaWorld() {
    return this._inverseInertiaWorld;
  }
  // Row-major scalar copy of inverseInertiaWorld for the solver hot loops.
  /** @internal */ i00 = 0;
  /** @internal */ i01 = 0;
  /** @internal */ i02 = 0;
  /** @internal */ i10 = 0;
  /** @internal */ i11 = 0;
  /** @internal */ i12 = 0;
  /** @internal */ i20 = 0;
  /** @internal */ i21 = 0;
  /** @internal */ i22 = 0;
  private setInertiaWorld(m: Matrix3x3) {
    this._inverseInertiaWorld = m;
    this.i00 = m.row0.x; this.i01 = m.row0.y; this.i02 = m.row0.z;
    this.i10 = m.row1.x; this.i11 = m.row1.y; this.i12 = m.row1.z;
    this.i20 = m.row2.x; this.i21 = m.row2.y; this.i22 = m.row2.z;
  }
  private inverseInertiaLocal: Vec3 = Vec3.Zero;

  totalForce: Vec3 = Vec3.Zero;
  totalTorque: Vec3 = Vec3.Zero;

  impulseLinearLocal: Vec3 = Vec3.Zero;
  impulseAngularLocal: Vec3 = Vec3.Zero;
  impulseLinearGlobal: Vec3 = Vec3.Zero;
  impulseAngularGlobal: Vec3 = Vec3.Zero;
  impulseResetFlag = false;

  isActive = true;
  sleepTimer = 0;

  get isStaticOrKinematic() {
    return this.mode === PhysicsMode.BoneFollow || this.mass <= 0;
  }

  get isKinematic() {
    return this.mode === PhysicsMode.BoneFollow;
  }

  constructor(shape: CollisionShape) {
    this.shape = shape;
    this.setMassProps(0);
  }

  setMassProps(mass: number) {
    mass = f(mass);
    this.mass = mass;
    if (mass > 0 && !this.isKinematic) {
      this.inverseMass = f(1 / mass);
      this.localInertiaDiag = this.shape.calculateLocalInertia(mass);
      const d = this.localInertiaDiag;
      this.inverseInertiaLocal = new Vec3(d.x > 0 ? f(1 / d.x) : 0, d.y > 0 ? f(1 / d.y) : 0, d.z > 0 ? f(1 / d.z) : 0);
    } else {
      this.inverseMass = 0;
      this.localInertiaDiag = Vec3.Zero;
      this.inverseInertiaLocal = Vec3.Zero;
    }
    this.updateInertiaWorld();
  }

  updateInertiaWorld() {
    if (this.inverseMass === 0) {
      this.setInertiaWorld(Matrix3x3.Zero);
      return;
    }
    const basis = Matrix3x3.fromQuat(this.worldTransform.rotation);
    this.setInertiaWorld(basis.scaled(this.inverseInertiaLocal));
  }

  get centerOfMass() {
    return this.worldTransform.origin;
  }

  velocityAtPoint(worldPoint: Vec3) {
    const r = worldPoint.sub(this.centerOfMass);
    return this.linearVelocity.add(Vec3.cross(this.angularVelocity, r));
  }

  applyForce(force: Vec3) {
    this.totalForce = this.totalForce.add(force);
  }

  applyTorque(torque: Vec3) {
    this.totalTorque = this.totalTorque.add(torque);
  }

  applyCentralImpulse(impulse: Vec3) {
    if (this.inverseMass === 0) return;
    this.linearVelocity = this.linearVelocity.add(impulse.mul(this.inverseMass));
  }

  applyTorqueImpulse(torque: Vec3) {
    this.applyTorqueImpulseXYZ(torque.x, torque.y, torque.z);
  }

  applyImpulse(impulse: Vec3, rel: Vec3) {
    this.applyImpulseXYZ(impulse.x, impulse.y, impulse.z, rel.x, rel.y, rel.z);
  }

  applyPushImpulse(impulse: Vec3, rel: Vec3) {
    if (this.inverseMass === 0) return;
    this.pseudoLinearVelocity = this.pseudoLinearVelocity.add(impulse.mul(this.inverseMass));
    this.pseudoAngularVelocity = this.pseudoAngularVelocity.add(this.inverseInertiaWorld.mulVec(Vec3.cross(rel, impulse)));
  }

  applyPushTorqueImpulse(torque: Vec3) {
    if (this.inverseMass === 0) return;
    this.pseudoAngularVelocity = this.pseudoAngularVelocity.add(this.inverseInertiaWorld.mulVec(torque));
  }

  /** applyImpulse(new Vec3(px,py,pz), new Vec3(rx,ry,rz)) without allocations (same float32 operations). */
  applyImpulseXYZ(px: number, py: number, pz: number, rx: number, ry: number, rz: number) {
    const im = this.inverseMass;
    if (im === 0) return;
    this.lvx = f(this.lvx + f(px * im));
    this.lvy = f(this.lvy + f(py * im));
    this.lvz = f(this.lvz + f(pz * im));
    const cx = f(f(ry * pz) - f(rz * py));
    const cy = f(f(rz * px) - f(rx * pz));
    const cz = f(f(rx * py) - f(ry * px));
    this.avx = f(this.avx + f(f(f(this.i00 * cx) + f(this.i01 * cy)) + f(this.i02 * cz)));
    this.avy = f(this.avy + f(f(f(this.i10 * cx) + f(this.i11 * cy)) + f(this.i12 * cz)));
    this.avz = f(this.avz + f(f(f(this.i20 * cx) + f(this.i21 * cy)) + f(this.i22 * cz)));
  }

  /** applyTorqueImpulse(new Vec3(tx,ty,tz)) without allocations. */
  applyTorqueImpulseXYZ(tx: number, ty: number, tz: number) {
    if (this.inverseMass === 0) return;
    this.avx = f(this.avx + f(f(f(this.i00 * tx) + f(this.i01 * ty)) + f(this.i02 * tz)));
    this.avy = f(this.avy + f(f(f(this.i10 * tx) + f(this.i11 * ty)) + f(this.i12 * tz)));
    this.avz = f(this.avz + f(f(f(this.i20 * tx) + f(this.i21 * ty)) + f(this.i22 * tz)));
  }

  pseudoVelocityAtPoint(worldPoint: Vec3) {
    const r = worldPoint.sub(this.centerOfMass);
    return this.pseudoLinearVelocity.add(Vec3.cross(this.pseudoAngularVelocity, r));
  }

  clearForces() {
    this.totalForce = Vec3.Zero;
    this.totalTorque = Vec3.Zero;
  }

  computeAabb(): Aabb {
    const c = this.worldTransform.origin;
    const r = f(this.shape.boundingRadius + this.shape.margin);
    const rv = Vec3.splat(r);
    return new Aabb(c.sub(rv), c.add(rv));
  }
}
