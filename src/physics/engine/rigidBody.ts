// ===========================================================================
// MMD physics engine (TypeScript port) – rigid body
// Port of RigidBody.cs (btRigidBody equivalent). float32 emulation rules: see math.ts.
// ===========================================================================

import { f, Aabb, Matrix3x3, RigidTransform, Vec3 } from './math';
import { CollisionShape } from './shapes';

/** PMX rigid body physics mode. */
const SCR_R = new Float64Array(9);
const SCR_D = new Float64Array(9);
const SCR_T = new Float64Array(9);

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

  /**
   * @internal Solver state as float32-valued numbers (no allocation in the hot loops):
   * [0..2] linear velocity, [3..5] angular velocity, [6..14] inverseInertiaWorld (row-major), [15] inverseMass.
   * linearVelocity / angularVelocity are value-semantics views (a fresh Vec3 per read).
   */
  readonly st = new Float64Array(16);
  get linearVelocity(): Vec3 {
    const s = this.st;
    return new Vec3(s[0], s[1], s[2]);
  }
  set linearVelocity(v: Vec3) {
    const s = this.st;
    s[0] = v.x;
    s[1] = v.y;
    s[2] = v.z;
  }
  get angularVelocity(): Vec3 {
    const s = this.st;
    return new Vec3(s[3], s[4], s[5]);
  }
  set angularVelocity(v: Vec3) {
    const s = this.st;
    s[3] = v.x;
    s[4] = v.y;
    s[5] = v.z;
  }

  pseudoLinearVelocity: Vec3 = Vec3.Zero;
  pseudoAngularVelocity: Vec3 = Vec3.Zero;

  kinematicTarget: RigidTransform = RigidTransform.Identity;
  kinematicStepTarget: RigidTransform = RigidTransform.Identity;

  localInertiaDiag: Vec3 = Vec3.Zero;

  private _inverseMass = 0;
  get inverseMass() {
    return this._inverseMass;
  }
  private set inverseMass(v: number) {
    this._inverseMass = v;
    this.st[15] = v;
  }
  private _inverseInertiaWorld: Matrix3x3 | null = Matrix3x3.Zero;
  /** World inverse inertia tensor (built lazily from the solver state). */
  get inverseInertiaWorld(): Matrix3x3 {
    let m = this._inverseInertiaWorld;
    if (!m) {
      const s = this.st;
      m = this._inverseInertiaWorld = new Matrix3x3(new Vec3(s[6], s[7], s[8]), new Vec3(s[9], s[10], s[11]), new Vec3(s[12], s[13], s[14]));
    }
    return m;
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
    const s = this.st;
    if (this._inverseMass === 0) {
      for (let i = 6; i < 15; i++) s[i] = 0;
      this._inverseInertiaWorld = Matrix3x3.Zero;
      return;
    }
    // basis.Scaled(invI) = (R * Diagonal(invI)) * R^T with R = Matrix3x3.FromQuat(rotation); scalar form of
    // the same Matrix3x3 products (zero entries of the diagonal included, like the C# code).
    const q = this.worldTransform.rotation;
    const xx = f(q.x * q.x), yy = f(q.y * q.y), zz = f(q.z * q.z);
    const xy = f(q.x * q.y), xz = f(q.x * q.z), yz = f(q.y * q.z);
    const wx = f(q.w * q.x), wy = f(q.w * q.y), wz = f(q.w * q.z);
    const R = SCR_R;
    R[0] = f(1 - f(2 * f(yy + zz))); R[1] = f(2 * f(xy - wz)); R[2] = f(2 * f(xz + wy));
    R[3] = f(2 * f(xy + wz)); R[4] = f(1 - f(2 * f(xx + zz))); R[5] = f(2 * f(yz - wx));
    R[6] = f(2 * f(xz - wy)); R[7] = f(2 * f(yz + wx)); R[8] = f(1 - f(2 * f(xx + yy)));
    const il = this.inverseInertiaLocal;
    const D = SCR_D;
    D[0] = il.x; D[1] = 0; D[2] = 0; D[3] = 0; D[4] = il.y; D[5] = 0; D[6] = 0; D[7] = 0; D[8] = il.z;
    const T = SCR_T;
    for (let i = 0; i < 9; i += 3)
      for (let j = 0; j < 3; j++) T[i + j] = f(f(f(R[i] * D[j]) + f(R[i + 1] * D[3 + j])) + f(R[i + 2] * D[6 + j]));
    // (T * R^T)[i][j] = row_i(T) · column_j(R^T) = row_i(T) · row_j(R)
    for (let i = 0; i < 3; i++)
      for (let j = 0; j < 3; j++)
        s[6 + i * 3 + j] = f(f(f(T[i * 3] * R[j * 3]) + f(T[i * 3 + 1] * R[j * 3 + 1])) + f(T[i * 3 + 2] * R[j * 3 + 2]));
    this._inverseInertiaWorld = null;
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
    applyImpulseSt(this.st, px, py, pz, rx, ry, rz);
  }

  /** applyTorqueImpulse(new Vec3(tx,ty,tz)) without allocations. */
  applyTorqueImpulseXYZ(tx: number, ty: number, tz: number) {
    const s = this.st;
    if (s[15] === 0) return;
    s[3] = f(s[3] + f(f(f(s[6] * tx) + f(s[7] * ty)) + f(s[8] * tz)));
    s[4] = f(s[4] + f(f(f(s[9] * tx) + f(s[10] * ty)) + f(s[11] * tz)));
    s[5] = f(s[5] + f(f(f(s[12] * tx) + f(s[13] * ty)) + f(s[14] * tz)));
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

/** @internal RigidBody.ApplyImpulse on a solver state array: v += P*im; w += I*(r x P). */
export function applyImpulseSt(s: Float64Array, px: number, py: number, pz: number, rx: number, ry: number, rz: number) {
  const im = s[15];
  if (im === 0) return;
  s[0] = f(s[0] + f(px * im));
  s[1] = f(s[1] + f(py * im));
  s[2] = f(s[2] + f(pz * im));
  const cx = f(f(ry * pz) - f(rz * py));
  const cy = f(f(rz * px) - f(rx * pz));
  const cz = f(f(rx * py) - f(ry * px));
  s[3] = f(s[3] + f(f(f(s[6] * cx) + f(s[7] * cy)) + f(s[8] * cz)));
  s[4] = f(s[4] + f(f(f(s[9] * cx) + f(s[10] * cy)) + f(s[11] * cz)));
  s[5] = f(s[5] + f(f(f(s[12] * cx) + f(s[13] * cy)) + f(s[14] * cz)));
}
