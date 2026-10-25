// ===========================================================================
// MMD physics engine (TypeScript port) – joint constraints
// Port of Constraints.cs: all PMX joint types solved as a Bullet 2.75-compatible 6DOF
// sequential-impulse constraint. float32 emulation rules: see math.ts.
// Diagnostic hooks (Debug* lists, probes, counters) of the C# source are not ported.
// ===========================================================================

import { f, FLT_MAX, Matrix3x3, Quat, RigidTransform, Vec3 } from './math';
import { RigidBody } from './rigidBody';

export enum JointType {
  Spring6Dof = 0,
  Generic6Dof = 1,
  Point2Point = 2,
  ConeTwist = 3,
  Slider = 4,
  Hinge = 5,
}

// ---- packed row store used by the velocity solver (one per world, or a joint-private one) ----
// Row layout (Float64Array, stride ROW): axis 0..2, relA 3..5, relB 6..8, lower 9, upper 10, targetVel 11,
// effMass 12, accumulated 13, angular 14 (0/1), dof 15, warmStartable 16, positionBias 17, pseudoAccumulated 18.
/** @internal */
export const ROW = 19;
/** @internal */
export class RowStore {
  d = new Float64Array(ROW * 256);
  bodyA: RigidBody[] = [];
  bodyB: RigidBody[] = [];
  joint: Joint[] = [];
  count = 0;
  alloc() {
    if ((this.count + 1) * ROW > this.d.length) {
      const nd = new Float64Array(this.d.length * 2);
      nd.set(this.d);
      this.d = nd;
    }
    return this.count++;
  }
}

/**
 * @internal Joint.SolveVelocity for rows [start, end) of a store, in order (scalar form of the C# Vec3
 * expressions, identical float32 operation order). The impulse application is inlined on purpose.
 */
export function solveRowRange(store: RowStore, start: number, end: number) {
  const d = store.d, bA = store.bodyA, bB = store.bodyB;
  for (let r = start; r < end; r++) {
    const o = r * ROW;
    const sa = bA[r].st, sb = bB[r].st;
    const ax = d[o], ay = d[o + 1], az = d[o + 2];
    const angular = d[o + 14] !== 0;
    let relVel: number;
    if (angular) {
      // (wB - wA).Dot(axis)
      relVel = f(f(f(f(sb[3] - sa[3]) * ax) + f(f(sb[4] - sa[4]) * ay)) + f(f(sb[5] - sa[5]) * az));
    } else {
      // ((vB + wB x relB) - (vA + wA x relA)).Dot(axis)
      const rax = d[o + 3], ray = d[o + 4], raz = d[o + 5], rbx = d[o + 6], rby = d[o + 7], rbz = d[o + 8];
      const wbx = sb[3], wby = sb[4], wbz = sb[5], wax = sa[3], way = sa[4], waz = sa[5];
      const bx = f(sb[0] + f(f(wby * rbz) - f(wbz * rby)));
      const by = f(sb[1] + f(f(wbz * rbx) - f(wbx * rbz)));
      const bz = f(sb[2] + f(f(wbx * rby) - f(wby * rbx)));
      const qx = f(sa[0] + f(f(way * raz) - f(waz * ray)));
      const qy = f(sa[1] + f(f(waz * rax) - f(wax * raz)));
      const qz = f(sa[2] + f(f(wax * ray) - f(way * rax)));
      relVel = f(f(f(f(bx - qx) * ax) + f(f(by - qy) * ay)) + f(f(bz - qz) * az));
    }

    let dImpulse = f(f(d[o + 11] - relVel) * d[o + 12]);
    const old = d[o + 13];
    const acc = Math.max(d[o + 9], Math.min(d[o + 10], f(old + dImpulse)));
    d[o + 13] = acc;
    dImpulse = f(acc - old);

    if (dImpulse !== 0) {
      const px = f(ax * dImpulse), py = f(ay * dImpulse), pz = f(az * dImpulse);
      const nx = -px, ny = -py, nz = -pz;
      if (angular) {
        // ApplyTorqueImpulse(-L) on A, (L) on B
        if (sa[15] !== 0) {
          sa[3] = f(sa[3] + f(f(f(sa[6] * nx) + f(sa[7] * ny)) + f(sa[8] * nz)));
          sa[4] = f(sa[4] + f(f(f(sa[9] * nx) + f(sa[10] * ny)) + f(sa[11] * nz)));
          sa[5] = f(sa[5] + f(f(f(sa[12] * nx) + f(sa[13] * ny)) + f(sa[14] * nz)));
        }
        if (sb[15] !== 0) {
          sb[3] = f(sb[3] + f(f(f(sb[6] * px) + f(sb[7] * py)) + f(sb[8] * pz)));
          sb[4] = f(sb[4] + f(f(f(sb[9] * px) + f(sb[10] * py)) + f(sb[11] * pz)));
          sb[5] = f(sb[5] + f(f(f(sb[12] * px) + f(sb[13] * py)) + f(sb[14] * pz)));
        }
      } else {
        // ApplyImpulse(-P, relA) on A, (P, relB) on B
        const ima = sa[15];
        if (ima !== 0) {
          const rx = d[o + 3], ry = d[o + 4], rz = d[o + 5];
          sa[0] = f(sa[0] + f(nx * ima));
          sa[1] = f(sa[1] + f(ny * ima));
          sa[2] = f(sa[2] + f(nz * ima));
          const cx = f(f(ry * nz) - f(rz * ny));
          const cy = f(f(rz * nx) - f(rx * nz));
          const cz = f(f(rx * ny) - f(ry * nx));
          sa[3] = f(sa[3] + f(f(f(sa[6] * cx) + f(sa[7] * cy)) + f(sa[8] * cz)));
          sa[4] = f(sa[4] + f(f(f(sa[9] * cx) + f(sa[10] * cy)) + f(sa[11] * cz)));
          sa[5] = f(sa[5] + f(f(f(sa[12] * cx) + f(sa[13] * cy)) + f(sa[14] * cz)));
        }
        const imb = sb[15];
        if (imb !== 0) {
          const rx = d[o + 6], ry = d[o + 7], rz = d[o + 8];
          sb[0] = f(sb[0] + f(px * imb));
          sb[1] = f(sb[1] + f(py * imb));
          sb[2] = f(sb[2] + f(pz * imb));
          const cx = f(f(ry * pz) - f(rz * py));
          const cy = f(f(rz * px) - f(rx * pz));
          const cz = f(f(rx * py) - f(ry * px));
          sb[3] = f(sb[3] + f(f(f(sb[6] * cx) + f(sb[7] * cy)) + f(sb[8] * cz)));
          sb[4] = f(sb[4] + f(f(f(sb[9] * cx) + f(sb[10] * cy)) + f(sb[11] * cz)));
          sb[5] = f(sb[5] + f(f(f(sb[12] * cx) + f(sb[13] * cy)) + f(sb[14] * cz)));
        }
      }
    }
    if (d[o + 16] !== 0) {
      const j = store.joint[r];
      if (angular) j.warmAng[d[o + 15]] = acc;
      else j.warmLin[d[o + 15]] = acc;
    }
  }
}

const F_01 = f(0.1);
const F_02 = f(0.2);
const F_085 = f(0.85);
const F_1EM9 = f(1e-9);
const F_1E18 = f(1e18);
const F_1EM6 = f(1e-6);
const F_ONE_MINUS_1EM6 = f(1 - F_1EM6);
const F_MINUS_ONE_PLUS_1EM6 = f(-1 + F_1EM6);
const F_HALF_PI = f(Math.PI / 2);

function testLimitValue(v: number, lo: number, hi: number) {
  if (lo > hi) return 0;
  if (v < lo) return 2;
  if (v > hi) return 1;
  return 0;
}

const isLocked = (lo: number, hi: number) => lo === hi;
const isFree = (lo: number, hi: number) => lo > hi;

function clampToLimit(v: number, lo: number, hi: number) {
  if (lo > hi) return v;
  return Math.max(lo, Math.min(hi, v));
}

export class Joint {
  name = '';
  type: JointType = JointType.Spring6Dof;

  bodyA: RigidBody | null = null;
  bodyB: RigidBody | null = null;

  frameInA: RigidTransform = RigidTransform.Identity;
  frameInB: RigidTransform = RigidTransform.Identity;

  linearLowerLimit: Vec3 = Vec3.Zero;
  linearUpperLimit: Vec3 = Vec3.Zero;
  angularLowerLimit: Vec3 = Vec3.Zero;
  angularUpperLimit: Vec3 = Vec3.Zero;

  springLinear: Vec3 = Vec3.Zero;
  springAngular: Vec3 = Vec3.Zero;

  springDamping = F_01;
  /** Baumgarte position correction factor (Bullet erp). */
  beta = F_02;

  // ---- static tunables (same names / defaults as the C# engine) ----
  static angularBetaScale = 1;
  static errDeadband = 0;
  static biasDeadband = 0;
  static splitCrossOnly = false;
  static splitChainOnly = false;
  static freezeCrossAxes = false;
  static maxCorrectionVel = 10;
  static angularMixedAxes = true;
  static bulletAngleConvention = true;
  static springAsMotorRow = true;
  static bulletLimitRowGating = true;
  static readonly bulletSpringDamping = 1;
  static linearLeverMode = 1;
  static leverArmGate = 5;
  static warmStartFactor = F_085;
  static lockedRowImpulseBound = FLT_MAX;
  /** Iteration count for the spring motor velFactor; PhysicsWorld copies SolverIterations here each substep. */
  static solverIterationsForSpring = 0;
  static disableSpringClamp = false;

  get isCrossTypeJoint() {
    for (let i = 0; i < 3; i++) if (this.linearLowerLimit.get(i) !== this.linearUpperLimit.get(i)) return true;
    return false;
  }

  private bindAxes: Vec3[] | null = null;

  private rowCount = 0;

  private readonly ownStore = new RowStore();
  private store: RowStore = this.ownStore;
  private rowStart = 0;

  /** Appends one row (C# AddRow(new ConstraintRow{...})) to the packed store. */
  private addRow(axis: Vec3, rA: Vec3, rB: Vec3, angular: boolean, lo: number, hi: number, targetVel: number,
                 positionBias: number, effMass: number, acc: number, dof: number, warm: boolean) {
    this.rowCount++;
    const st = this.store;
    const idx = st.alloc();
    const d = st.d, o = idx * ROW;
    d[o] = axis.x; d[o + 1] = axis.y; d[o + 2] = axis.z;
    d[o + 3] = rA.x; d[o + 4] = rA.y; d[o + 5] = rA.z;
    d[o + 6] = rB.x; d[o + 7] = rB.y; d[o + 8] = rB.z;
    d[o + 9] = lo; d[o + 10] = hi; d[o + 11] = targetVel;
    d[o + 12] = effMass; d[o + 13] = acc; d[o + 14] = angular ? 1 : 0;
    d[o + 15] = dof; d[o + 16] = warm ? 1 : 0; d[o + 17] = positionBias; d[o + 18] = 0;
    st.bodyA[idx] = this.bodyA!;
    st.bodyB[idx] = this.bodyB!;
    st.joint[idx] = this;
  }

  private worldA: RigidTransform = RigidTransform.Identity;
  private worldB: RigidTransform = RigidTransform.Identity;
  private anchorA: Vec3 = Vec3.Zero;
  private anchorB: Vec3 = Vec3.Zero;

  private leverLenRA = 0;
  private leverLenRB = 0;
  private leverLenABReady = false;
  private leverLenAnchorDelta = 0;
  private leverLenAnchorReady = false;

  private leverAnchorDelta() {
    if (!this.leverLenAnchorReady) {
      this.leverLenAnchorDelta = this.anchorB.sub(this.anchorA).length;
      this.leverLenAnchorReady = true;
    }
    return this.leverLenAnchorDelta;
  }

  private axesA: Vec3[] = [Vec3.Zero, Vec3.Zero, Vec3.Zero];
  private splitBias = false;
  private warmStart = false;
  /** @internal */ readonly warmLin = [0, 0, 0];
  private readonly warmLinSeen = [false, false, false];
  private warmStartAng = false;
  /** @internal */ readonly warmAng = [0, 0, 0];
  private readonly warmAngPrevSide = [-1, -1, -1];
  private readonly warmAngSeen = [false, false, false];

  /** Builds a joint from raw PMX parameters (frames are converted into each body's local space). */
  static fromPmx(
    type: JointType, a: RigidBody | null, b: RigidBody | null,
    worldFrame: RigidTransform,
    linLo: Vec3, linHi: Vec3, angLo: Vec3, angHi: Vec3,
    springLin: Vec3, springAng: Vec3,
  ): Joint {
    const j = new Joint();
    j.type = type;
    j.bodyA = a;
    j.bodyB = b;
    j.linearLowerLimit = linLo;
    j.linearUpperLimit = linHi;
    j.angularLowerLimit = angLo;
    j.angularUpperLimit = angHi;
    j.springLinear = springLin;
    j.springAngular = springAng;

    j.frameInA = a ? a.worldTransform.inverseTimes(worldFrame) : worldFrame;
    j.frameInB = b ? b.worldTransform.inverseTimes(worldFrame) : worldFrame;

    switch (type) {
      case JointType.Point2Point:
        j.angularLowerLimit = Vec3.splat(1);
        j.angularUpperLimit = Vec3.splat(-1);
        j.linearLowerLimit = j.linearUpperLimit = Vec3.Zero;
        j.springLinear = j.springAngular = Vec3.Zero;
        break;
      case JointType.Hinge:
        j.linearLowerLimit = j.linearUpperLimit = Vec3.Zero;
        j.angularLowerLimit = new Vec3(angLo.x, 0, 0);
        j.angularUpperLimit = new Vec3(angHi.x, 0, 0);
        break;
      case JointType.Slider:
        j.linearLowerLimit = new Vec3(linLo.x, 0, 0);
        j.linearUpperLimit = new Vec3(linHi.x, 0, 0);
        j.angularLowerLimit = new Vec3(angLo.x, 0, 0);
        j.angularUpperLimit = new Vec3(angHi.x, 0, 0);
        break;
      case JointType.ConeTwist:
        j.linearLowerLimit = j.linearUpperLimit = Vec3.Zero;
        break;
      case JointType.Generic6Dof:
        j.springLinear = j.springAngular = Vec3.Zero;
        break;
      default:
        break;
    }
    return j;
  }

  /** Prepares frames / anchors / rows for one substep. */
  prepare(dt: number, splitBias = false, warmStart = false, warmStartAng = false, store?: RowStore) {
    // Rows are appended to the world's packed store (or a private one when called standalone).
    if (store) this.store = store;
    else {
      this.store = this.ownStore;
      this.ownStore.count = 0;
    }
    this.rowStart = this.store.count;
    this.splitBias = Joint.splitCrossOnly ? this.isCrossTypeJoint : Joint.splitChainOnly ? !this.isCrossTypeJoint : splitBias;
    this.warmStart = warmStart;
    this.warmStartAng = warmStartAng;
    this.warmLinSeen[0] = this.warmLinSeen[1] = this.warmLinSeen[2] = false;
    this.warmAngSeen[0] = this.warmAngSeen[1] = this.warmAngSeen[2] = false;
    this.rowCount = 0;
    this.leverLenABReady = false;
    this.leverLenAnchorReady = false;
    const bodyA = this.bodyA, bodyB = this.bodyB;
    if (!bodyA || !bodyB) return;

    if (
      Joint.bulletLimitRowGating && Joint.angularMixedAxes && Joint.bulletAngleConvention && Joint.linearLeverMode === 1 &&
      Joint.springAsMotorRow && !Joint.freezeCrossAxes && !warmStart && !warmStartAng && this.bindAxes
    ) {
      this.prepareFast(dt, bodyA, bodyB);
      return;
    }

    this.worldA = bodyA.worldTransform.mul(this.frameInA);
    this.worldB = bodyB.worldTransform.mul(this.frameInB);
    this.anchorA = this.worldA.origin;
    this.anchorB = this.worldB.origin;

    const rA = this.anchorA.sub(bodyA.centerOfMass);
    const rB = this.anchorB.sub(bodyB.centerOfMass);

    // Matrix3x3.FromQuat(worldA.rotation).Column(0..2), via scalar scratch (same operations).
    m3FromQuat(this.worldA.rotation, S_BASIS);
    const axesA = this.axesA;
    axesA[0] = new Vec3(S_BASIS[0], S_BASIS[3], S_BASIS[6]);
    axesA[1] = new Vec3(S_BASIS[1], S_BASIS[4], S_BASIS[7]);
    axesA[2] = new Vec3(S_BASIS[2], S_BASIS[5], S_BASIS[8]);
    if (!this.bindAxes) this.bindAxes = [axesA[0], axesA[1], axesA[2]];
    const freezeLin = Joint.freezeCrossAxes && this.isCrossTypeJoint;

    const invDt = dt > 0 ? f(1 / dt) : 0;
    const linDelta = this.anchorB.sub(this.anchorA);

    let euler: Vec3;
    let linDiffBt = Vec3.Zero;
    if (Joint.bulletLimitRowGating) {
      // Same chain as the C# Matrix3x3 code (FromQuatBullet / * / BulletInverse), on scalar scratch buffers.
      m3FromQuatBullet(bodyA.worldTransform.rotation, S_BMA);
      m3FromQuatBullet(bodyB.worldTransform.rotation, S_BMB);
      m3FromQuatBullet(this.frameInA.rotation, S_T);
      m3Mul(S_BMA, S_T, S_BA);
      m3FromQuatBullet(this.frameInB.rotation, S_T);
      m3Mul(S_BMB, S_T, S_BB);
      const fa = this.frameInA.origin, fb = this.frameInB.origin;
      const oa = bodyA.worldTransform.origin, ob = bodyB.worldTransform.origin;
      const oAx = f(m3Row(S_BMA, 0, fa.x, fa.y, fa.z) + oa.x), oAy = f(m3Row(S_BMA, 3, fa.x, fa.y, fa.z) + oa.y), oAz = f(m3Row(S_BMA, 6, fa.x, fa.y, fa.z) + oa.z);
      const oBx = f(m3Row(S_BMB, 0, fb.x, fb.y, fb.z) + ob.x), oBy = f(m3Row(S_BMB, 3, fb.x, fb.y, fb.z) + ob.y), oBz = f(m3Row(S_BMB, 6, fb.x, fb.y, fb.z) + ob.z);
      m3BulletInverse(S_BA, S_INV);
      const dx = f(oBx - oAx), dy = f(oBy - oAy), dz = f(oBz - oAz);
      linDiffBt = new Vec3(m3Row(S_INV, 0, dx, dy, dz), m3Row(S_INV, 3, dx, dy, dz), m3Row(S_INV, 6, dx, dy, dz));
      m3Mul(S_INV, S_BB, S_T);
      euler = toEulerXYZBulletArr(S_T, Joint.bulletAngleConvention);
    } else {
      const qRel = this.worldA.rotation.conjugated().mul(this.worldB.rotation);
      euler = Joint.toEulerXYZ(Joint.bulletAngleConvention ? qRel.conjugated().normalized : qRel.normalized);
    }

    // LinearLeverMode = 2 precomputation.
    let hasStatic = false;
    let factA = 0.5, factB = 0.5;
    const angActive = [false, false, false];
    if (Joint.linearLeverMode === 2) {
      const miA = bodyA.inverseMass, miB = bodyB.inverseMass;
      hasStatic = miA < F_1EM9 || miB < F_1EM9;
      const miS = f(miA + miB);
      factA = miS > 0 ? f(miB / miS) : 0.5;
      factB = f(1 - factA);
      for (let i = 0; i < 3; i++) {
        const alo = this.angularLowerLimit.get(i), ahi = this.angularUpperLimit.get(i);
        const e = euler.get(i);
        angActive[i] = !isFree(alo, ahi) && (isLocked(alo, ahi) ? e !== alo : e < alo || e > ahi);
      }
    }

    // --- linear 3 axes ---
    for (let i = 0; i < 3; i++) {
      const axis = freezeLin ? this.bindAxes[i] : axesA[i];
      const lo = this.linearLowerLimit.get(i), hi = this.linearUpperLimit.get(i);
      const curF = Joint.bulletLimitRowGating && !freezeLin ? linDiffBt.get(i) : linDelta.dot(axis);
      if (isFree(lo, hi)) {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(false, i, axis, curF, lo, hi, rA, rB, invDt);
        continue;
      }

      const cur = curF;
      let err: number, lower: number, upper: number;
      if (Joint.bulletLimitRowGating && isLocked(lo, hi) && testLimitValue(cur, lo, hi) === 0) {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(false, i, axis, cur, lo, hi, rA, rB, invDt);
        continue;
      }
      const BND = Joint.lockedRowImpulseBound;
      if (isLocked(lo, hi)) { err = f(lo - cur); lower = -BND; upper = BND; }
      else if (cur < lo) { err = f(lo - cur); lower = 0; upper = BND; }
      else if (cur > hi) { err = f(hi - cur); lower = -BND; upper = 0; }
      else {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(false, i, axis, cur, lo, hi, rA, rB, invDt);
        continue;
      }
      if (Joint.errDeadband > 0 && Math.abs(err) < Joint.errDeadband) continue;

      let armA = rA, armB = rB;
      if (Joint.linearLeverMode === 1) {
        armA = this.anchorB.sub(bodyA.centerOfMass);
        armB = rB;
        if (Joint.leverArmGate > 0) {
          if (!this.leverLenABReady) {
            this.leverLenRA = rA.length;
            this.leverLenRB = rB.length;
            this.leverLenABReady = true;
          }
          const scale = f(this.leverLenRA + this.leverLenRB);
          const ratio = scale > F_1EM9 ? f(this.leverAnchorDelta() / scale) : 0;
          if (ratio > Joint.leverArmGate) armA = rA;
        }
      } else if (Joint.linearLeverMode === 2) {
        const pA = rA.dot(axis), pB = rB.dot(axis);
        const orthoA = rA.sub(axis.mul(pA));
        const orthoB = rB.sub(axis.mul(pB));
        const desiredOffs = f(cur + err);
        const totalDist = axis.mul(f(f(pA + desiredOffs) - pB));
        armA = orthoA.add(totalDist.mul(factA));
        armB = orthoB.sub(totalDist.mul(factB));
        const rotAllowed = !(angActive[(i + 1) % 3] && angActive[(i + 2) % 3]);
        if (hasStatic && !rotAllowed) {
          armA = armA.mul(factA);
          armB = armB.mul(factB);
        }
      }

      let linTarget = Joint.clampCorr(f(f(err * this.beta) * invDt));
      if (Joint.biasDeadband > 0 && Math.abs(err) < Joint.biasDeadband) linTarget = 0;
      this.addLinearRow(axis, armA, armB, linTarget, lower, upper, i, isLocked(lo, hi));
    }

    // --- angular 3 axes ---
    let mix0 = Vec3.Zero, mix1 = Vec3.Zero, mix2 = Vec3.Zero;
    if (Joint.angularMixedAxes) {
      m3FromQuat(this.worldB.rotation, S_BASIS);
      const axis0 = new Vec3(S_BASIS[0], S_BASIS[3], S_BASIS[6]);
      const axis2 = axesA[2];
      const m1 = Vec3.cross(axis2, axis0);
      const m0 = Vec3.cross(m1, axis2);
      const m2 = Vec3.cross(axis0, m1);
      mix0 = m0.normalized;
      mix1 = m1.normalized;
      mix2 = m2.normalized;
    }
    for (let i = 0; i < 3; i++) {
      let axis = Joint.angularMixedAxes ? (i === 0 ? mix0 : i === 1 ? mix1 : mix2) : axesA[i];
      if (Joint.bulletAngleConvention) axis = axis.neg();
      const lo = this.angularLowerLimit.get(i), hi = this.angularUpperLimit.get(i);
      if (isFree(lo, hi)) {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(true, i, axis, euler.get(i), lo, hi, Vec3.Zero, Vec3.Zero, invDt);
        continue;
      }

      const cur = euler.get(i);
      let err: number, lower: number, upper: number, sideCode: number;
      if (Joint.bulletLimitRowGating && isLocked(lo, hi) && testLimitValue(cur, lo, hi) === 0) {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(true, i, axis, cur, lo, hi, Vec3.Zero, Vec3.Zero, invDt);
        continue;
      }
      if (isLocked(lo, hi)) { err = f(lo - cur); lower = -F_1E18; upper = F_1E18; sideCode = 0; }
      else if (cur < lo) { err = f(lo - cur); lower = 0; upper = F_1E18; sideCode = 1; }
      else if (cur > hi) { err = f(hi - cur); lower = -F_1E18; upper = 0; sideCode = 2; }
      else {
        if (Joint.springAsMotorRow) this.addSpringMotorRow(true, i, axis, cur, lo, hi, Vec3.Zero, Vec3.Zero, invDt);
        continue;
      }
      if (Joint.errDeadband > 0 && Math.abs(err) < Joint.errDeadband) continue;

      const angBeta = f(this.beta * Joint.angularBetaScale);
      this.addAngularRow(axis, Joint.clampCorr(f(f(err * angBeta) * invDt)), lower, upper, i, sideCode);
    }

    if (this.warmStart) for (let i = 0; i < 3; i++) if (!this.warmLinSeen[i]) this.warmLin[i] = 0;
    if (this.warmStartAng)
      for (let i = 0; i < 3; i++)
        if (!this.warmAngSeen[i]) {
          this.warmAng[i] = 0;
          this.warmAngPrevSide[i] = -1;
        }

    if (this.warmStart || this.warmStartAng) {
      const d = this.store.d;
      for (let r = this.rowStart; r < this.rowStart + this.rowCount; r++) {
        const o = r * ROW;
        if (d[o + 16] === 0) continue;
        const axis = new Vec3(d[o], d[o + 1], d[o + 2]);
        if (d[o + 14] !== 0) {
          const L = axis.mul(d[o + 13]);
          bodyA.applyTorqueImpulse(L.neg());
          bodyB.applyTorqueImpulse(L);
        } else {
          const P = axis.mul(d[o + 13]);
          bodyA.applyImpulse(P.neg(), new Vec3(d[o + 3], d[o + 4], d[o + 5]));
          bodyB.applyImpulse(P, new Vec3(d[o + 6], d[o + 7], d[o + 8]));
        }
      }
    }
  }


  /**
   * Prepare() for the default flag set (BulletLimitRowGating, AngularMixedAxes, BulletAngleConvention,
   * LinearLeverMode=1, SpringAsMotorRow, no joint warm start, no FreezeCrossAxes) in scalar form:
   * the same float32 operations in the same order as the generic path above, without allocations.
   */
  private prepareFast(dt: number, bodyA: RigidBody, bodyB: RigidBody) {
    const S = PF;
    const ta = bodyA.worldTransform, tb = bodyB.worldTransform;
    const fA = this.frameInA, fB = this.frameInB;
    // worldA = A.wt * frameInA ; worldB = B.wt * frameInB  (RigidTransform operator *)
    qMulNormalized(ta.rotation, fA.rotation, S, 0);
    qRotate(ta.rotation, fA.origin.x, fA.origin.y, fA.origin.z, S, 8);
    const aAx = f(S[8] + ta.origin.x), aAy = f(S[9] + ta.origin.y), aAz = f(S[10] + ta.origin.z);
    qMulNormalized(tb.rotation, fB.rotation, S, 4);
    qRotate(tb.rotation, fB.origin.x, fB.origin.y, fB.origin.z, S, 8);
    const aBx = f(S[8] + tb.origin.x), aBy = f(S[9] + tb.origin.y), aBz = f(S[10] + tb.origin.z);

    const ca = ta.origin, cb = tb.origin;
    const rAx = f(aAx - ca.x), rAy = f(aAy - ca.y), rAz = f(aAz - ca.z);
    const rBx = f(aBx - cb.x), rBy = f(aBy - cb.y), rBz = f(aBz - cb.z);

    // basisA = FromQuat(worldA.rotation); axesA[i] = column i
    m3FromQuat4(S, 0, S_BASIS);
    const BA = S_BASIS;
    const invDt = dt > 0 ? f(1 / dt) : 0;

    // Bullet calculateLinearInfo / calculateAngleInfo chain (see the generic path)
    m3FromQuatBullet(ta.rotation, S_BMA);
    m3FromQuatBullet(tb.rotation, S_BMB);
    m3FromQuatBullet(fA.rotation, S_T);
    m3Mul(S_BMA, S_T, S_BA);
    m3FromQuatBullet(fB.rotation, S_T);
    m3Mul(S_BMB, S_T, S_BB);
    const fa = fA.origin, fb = fB.origin;
    const oAx = f(m3Row(S_BMA, 0, fa.x, fa.y, fa.z) + ca.x), oAy = f(m3Row(S_BMA, 3, fa.x, fa.y, fa.z) + ca.y), oAz = f(m3Row(S_BMA, 6, fa.x, fa.y, fa.z) + ca.z);
    const oBx = f(m3Row(S_BMB, 0, fb.x, fb.y, fb.z) + cb.x), oBy = f(m3Row(S_BMB, 3, fb.x, fb.y, fb.z) + cb.y), oBz = f(m3Row(S_BMB, 6, fb.x, fb.y, fb.z) + cb.z);
    m3BulletInverse(S_BA, S_INV);
    const dx = f(oBx - oAx), dy = f(oBy - oAy), dz = f(oBz - oAz);
    const ld0 = m3Row(S_INV, 0, dx, dy, dz), ld1 = m3Row(S_INV, 3, dx, dy, dz), ld2 = m3Row(S_INV, 6, dx, dy, dz);
    m3Mul(S_INV, S_BB, S_T);
    eulerBulletInto(S_T, true, S, 12); // euler -> S[12..14]

    const lin = this.linearLowerLimit, linU = this.linearUpperLimit;
    const sa = bodyA.st, sb = bodyB.st;
    const beta = this.beta;
    const gate = Joint.leverArmGate;
    // armA for LinearLeverMode=1: anchorB - comA
    const pAx = f(aBx - ca.x), pAy = f(aBy - ca.y), pAz = f(aBz - ca.z);
    let lenRA = -1, lenRB = -1, anchorDelta = -1;
    const anchorDeltaF = () => {
      if (anchorDelta < 0) {
        const ex = f(aBx - aAx), ey = f(aBy - aAy), ez = f(aBz - aAz);
        anchorDelta = f(Math.sqrt(f(f(f(ex * ex) + f(ey * ey)) + f(ez * ez))));
      }
      return anchorDelta;
    };
    for (let i = 0; i < 3; i++) {
      const ax = BA[i], ay = BA[3 + i], az = BA[6 + i];
      const lo = i === 0 ? lin.x : i === 1 ? lin.y : lin.z;
      const hi = i === 0 ? linU.x : i === 1 ? linU.y : linU.z;
      const cur = i === 0 ? ld0 : i === 1 ? ld1 : ld2;
      let err: number, lower: number, upper: number;
      if (lo > hi || (lo === hi && testLimitValue(cur, lo, hi) === 0) || (lo !== hi && !(cur < lo) && !(cur > hi))) {
        // free / locked exactly at the limit / inside the range -> spring motor row only
        const k = i === 0 ? this.springLinear.x : i === 1 ? this.springLinear.y : this.springLinear.z;
        if (!(k <= 0)) this.motorRowFast(false, i, ax, ay, az, cur, lo, hi, k, invDt, rAx, rAy, rAz, rBx, rBy, rBz, pAx, pAy, pAz, anchorDeltaF, sa, sb);
        continue;
      }
      const BND = Joint.lockedRowImpulseBound;
      if (lo === hi) { err = f(lo - cur); lower = -BND; upper = BND; }
      else if (cur < lo) { err = f(lo - cur); lower = 0; upper = BND; }
      else { err = f(hi - cur); lower = -BND; upper = 0; }
      if (Joint.errDeadband > 0 && Math.abs(err) < Joint.errDeadband) continue;

      let armAx = pAx, armAy = pAy, armAz = pAz;
      if (gate > 0) {
        if (lenRA < 0) {
          lenRA = f(Math.sqrt(f(f(f(rAx * rAx) + f(rAy * rAy)) + f(rAz * rAz))));
          lenRB = f(Math.sqrt(f(f(f(rBx * rBx) + f(rBy * rBy)) + f(rBz * rBz))));
        }
        const scale = f(lenRA + lenRB);
        const ratio = scale > F_1EM9 ? f(anchorDeltaF() / scale) : 0;
        if (ratio > gate) { armAx = rAx; armAy = rAy; armAz = rAz; }
      }
      let linTarget = Joint.clampCorr(f(f(err * beta) * invDt));
      if (Joint.biasDeadband > 0 && Math.abs(err) < Joint.biasDeadband) linTarget = 0;
      this.linearRowFast(sa, sb, ax, ay, az, armAx, armAy, armAz, rBx, rBy, rBz, linTarget, lower, upper, i);
    }

    // --- angular: Bullet mixed axes  axis0 = B col0, axis2 = A col2, m1 = axis2 x axis0, m0 = m1 x axis2, m2 = axis0 x m1
    m3FromQuat4(S, 4, S_T);
    const b0x = S_T[0], b0y = S_T[3], b0z = S_T[6];
    const a2x = BA[2], a2y = BA[5], a2z = BA[8];
    const m1x = f(f(a2y * b0z) - f(a2z * b0y)), m1y = f(f(a2z * b0x) - f(a2x * b0z)), m1z = f(f(a2x * b0y) - f(a2y * b0x));
    const m0x = f(f(m1y * a2z) - f(m1z * a2y)), m0y = f(f(m1z * a2x) - f(m1x * a2z)), m0z = f(f(m1x * a2y) - f(m1y * a2x));
    const m2x = f(f(b0y * m1z) - f(b0z * m1y)), m2y = f(f(b0z * m1x) - f(b0x * m1z)), m2z = f(f(b0x * m1y) - f(b0y * m1x));
    normalizeInto(m0x, m0y, m0z, S, 16);
    normalizeInto(m1x, m1y, m1z, S, 19);
    normalizeInto(m2x, m2y, m2z, S, 22);
    const angL = this.angularLowerLimit, angU = this.angularUpperLimit;
    const angBeta = f(beta * Joint.angularBetaScale);
    for (let i = 0; i < 3; i++) {
      // BulletAngleConvention: axis = -mix_i
      const ax = -S[16 + i * 3], ay = -S[17 + i * 3], az = -S[18 + i * 3];
      const lo = i === 0 ? angL.x : i === 1 ? angL.y : angL.z;
      const hi = i === 0 ? angU.x : i === 1 ? angU.y : angU.z;
      const cur = S[12 + i];
      if (lo > hi || (lo === hi && testLimitValue(cur, lo, hi) === 0) || (lo !== hi && !(cur < lo) && !(cur > hi))) {
        const k = i === 0 ? this.springAngular.x : i === 1 ? this.springAngular.y : this.springAngular.z;
        if (!(k <= 0)) this.motorRowFast(true, i, ax, ay, az, cur, lo, hi, k, invDt, 0, 0, 0, 0, 0, 0, 0, 0, 0, anchorDeltaF, sa, sb);
        continue;
      }
      let err: number, lower: number, upper: number;
      if (lo === hi) { err = f(lo - cur); lower = -F_1E18; upper = F_1E18; }
      else if (cur < lo) { err = f(lo - cur); lower = 0; upper = F_1E18; }
      else { err = f(hi - cur); lower = -F_1E18; upper = 0; }
      if (Joint.errDeadband > 0 && Math.abs(err) < Joint.errDeadband) continue;
      this.angularRowFast(sa, sb, ax, ay, az, Joint.clampCorr(f(f(err * angBeta) * invDt)), lower, upper, i);
    }
  }

  /** AddSpringMotorRow (scalar). k > 0 is checked by the caller. */
  private motorRowFast(angular: boolean, i: number, ax: number, ay: number, az: number, cur: number, lo: number, hi: number,
    k: number, invDt: number, rAx: number, rAy: number, rAz: number, rBx: number, rBy: number, rBz: number,
    pAx: number, pAy: number, pAz: number, anchorDelta: () => number, sa: Float64Array, sb: Float64Array) {
    const fps = invDt;
    if (fps <= 0) return;
    const eq = clampToLimit(0, lo, hi);
    const delta = f(cur - eq);
    const iters = Joint.solverIterationsForSpring > 0 ? Joint.solverIterationsForSpring : 10;
    const velFactor = f(f(fps * Joint.bulletSpringDamping) / iters);
    const force = f(delta * k);
    let targetVel = f(-velFactor * force);
    const maxMotorForce = f(Math.abs(force) / fps);
    const motFact = Joint.motorFactor(cur, lo, hi, targetVel, f(fps * this.beta));
    targetVel = f(targetVel * motFact);
    if (angular) {
      this.angularRowFast(sa, sb, ax, ay, az, targetVel, -maxMotorForce, maxMotorForce, i);
      return;
    }
    let armAx = pAx, armAy = pAy, armAz = pAz;
    if (Joint.leverArmGate > 0) {
      const gscale = f(f(Math.sqrt(f(f(f(rAx * rAx) + f(rAy * rAy)) + f(rAz * rAz)))) + f(Math.sqrt(f(f(f(rBx * rBx) + f(rBy * rBy)) + f(rBz * rBz)))));
      if (anchorDelta() > f(Joint.leverArmGate * gscale)) { armAx = rAx; armAy = rAy; armAz = rAz; }
    }
    this.linearRowFast(sa, sb, ax, ay, az, armAx, armAy, armAz, rBx, rBy, rBz, targetVel, -maxMotorForce, maxMotorForce, i);
  }

  /** AddLinearRow (scalar, no warm start). */
  private linearRowFast(sa: Float64Array, sb: Float64Array, ax: number, ay: number, az: number,
    rax: number, ray: number, raz: number, rbx: number, rby: number, rbz: number,
    targetVel: number, lo: number, hi: number, dof: number) {
    const cAx = f(f(ray * az) - f(raz * ay)), cAy = f(f(raz * ax) - f(rax * az)), cAz = f(f(rax * ay) - f(ray * ax));
    const cBx = f(f(rby * az) - f(rbz * ay)), cBy = f(f(rbz * ax) - f(rbx * az)), cBz = f(f(rbx * ay) - f(rby * ax));
    const k = f(f(f(sa[15] + sb[15]) + inertiaDot(sa, cAx, cAy, cAz)) + inertiaDot(sb, cBx, cBy, cBz));
    this.pushRow(ax, ay, az, rax, ray, raz, rbx, rby, rbz, false, lo, hi, targetVel, k > 0 ? f(1 / k) : 0, dof);
  }

  /** AddAngularRow (scalar, no warm start). */
  private angularRowFast(sa: Float64Array, sb: Float64Array, ax: number, ay: number, az: number, targetVel: number, lo: number, hi: number, dof: number) {
    const k = f(inertiaDot(sa, ax, ay, az) + inertiaDot(sb, ax, ay, az));
    this.pushRow(ax, ay, az, 0, 0, 0, 0, 0, 0, true, lo, hi, targetVel, k > 0 ? f(1 / k) : 0, dof);
  }

  private pushRow(ax: number, ay: number, az: number, rax: number, ray: number, raz: number, rbx: number, rby: number, rbz: number,
    angular: boolean, lo: number, hi: number, targetVel: number, effMass: number, dof: number) {
    this.rowCount++;
    const st = this.store;
    const idx = st.alloc();
    const d = st.d, o = idx * ROW;
    d[o] = ax; d[o + 1] = ay; d[o + 2] = az;
    d[o + 3] = rax; d[o + 4] = ray; d[o + 5] = raz;
    d[o + 6] = rbx; d[o + 7] = rby; d[o + 8] = rbz;
    d[o + 9] = lo; d[o + 10] = hi;
    d[o + 11] = this.splitBias ? 0 : targetVel;
    d[o + 12] = effMass; d[o + 13] = 0; d[o + 14] = angular ? 1 : 0;
    d[o + 15] = dof; d[o + 16] = 0; d[o + 17] = this.splitBias ? targetVel : 0; d[o + 18] = 0;
    st.bodyA[idx] = this.bodyA!;
    st.bodyB[idx] = this.bodyB!;
    st.joint[idx] = this;
  }

  /** Spring as a Bullet 2.75 limit-motor row (only for axes without a limit row). */
  private addSpringMotorRow(angular: boolean, i: number, axis: Vec3, cur: number, lo: number, hi: number, rA: Vec3, rB: Vec3, invDt: number) {
    const k = angular ? this.springAngular.get(i) : this.springLinear.get(i);
    if (k <= 0) return;
    const fps = invDt;
    if (fps <= 0) return;
    const eq = clampToLimit(0, lo, hi);
    const delta = f(cur - eq);
    const iters = Joint.solverIterationsForSpring > 0 ? Joint.solverIterationsForSpring : 10;
    const velFactor = f(f(fps * Joint.bulletSpringDamping) / iters);
    const force = f(delta * k);
    let targetVel = f(-velFactor * force);
    const maxMotorForce = f(Math.abs(force) / fps);
    const motFact = Joint.motorFactor(cur, lo, hi, targetVel, f(fps * this.beta));
    targetVel = f(targetVel * motFact);
    let armA = rA;
    const armB = rB;
    if (Joint.linearLeverMode === 1) {
      armA = this.anchorB.sub(this.bodyA!.centerOfMass);
      if (Joint.leverArmGate > 0) {
        const gscale = f(rA.length + rB.length);
        if (this.leverAnchorDelta() > f(Joint.leverArmGate * gscale)) armA = rA;
      }
    }
    if (angular) {
      this.addAngularRow(axis, targetVel, -maxMotorForce, maxMotorForce, i, 5);
      return;
    }
    this.addLinearRow(axis, armA, armB, targetVel, -maxMotorForce, maxMotorForce, i, false);
  }

  /** Bullet 2.75 btTypedConstraint::getMotorFactor. */
  static motorFactor(pos: number, lowLim: number, uppLim: number, vel: number, timeFact: number) {
    if (lowLim > uppLim) return 1;
    if (lowLim === uppLim) return 0;
    let limFact = 1;
    const deltaMax = timeFact !== 0 ? f(vel / timeFact) : 0;
    if (deltaMax < 0) {
      if (pos >= lowLim && pos < f(lowLim - deltaMax)) limFact = f(f(lowLim - pos) / deltaMax);
      else if (pos < lowLim) limFact = 0;
      else limFact = 1;
    } else if (deltaMax > 0) {
      if (pos <= uppLim && pos > f(uppLim - deltaMax)) limFact = f(f(uppLim - pos) / deltaMax);
      else if (pos > uppLim) limFact = 0;
      else limFact = 1;
    } else limFact = 0;
    return limFact;
  }

  private static clampCorr(v: number) {
    const m = Joint.maxCorrectionVel;
    return Math.max(-m, Math.min(m, v));
  }

  private addLinearRow(axis: Vec3, rA: Vec3, rB: Vec3, targetVel: number, lo: number, hi: number, dof: number, locked: boolean) {
    // k = imA + imB + (rA x n)·(IA (rA x n)) + (rB x n)·(IB (rB x n))   (scalar form, same operation order)
    const sa = this.bodyA!.st, sb = this.bodyB!.st;
    const k = f(f(f(sa[15] + sb[15]) + crossInertiaDot(sa, rA, axis)) + crossInertiaDot(sb, rB, axis));
    const warm = this.warmStart && locked;
    let acc = 0;
    if (warm) {
      this.warmLinSeen[dof] = true;
      acc = Math.max(lo, Math.min(hi, f(this.warmLin[dof] * Joint.warmStartFactor)));
    }
    this.addRow(axis, rA, rB, false, lo, hi, this.splitBias ? 0 : targetVel, this.splitBias ? targetVel : 0,
      k > 0 ? f(1 / k) : 0, acc, dof, warm);
  }

  private addAngularRow(axis: Vec3, targetVel: number, lo: number, hi: number, dof: number, sideCode: number) {
    const sa = this.bodyA!.st, sb = this.bodyB!.st;
    const k = f(inertiaDot(sa, axis.x, axis.y, axis.z) + inertiaDot(sb, axis.x, axis.y, axis.z));
    let warm = false;
    let acc = 0;
    if (this.warmStartAng) {
      warm = true;
      this.warmAngSeen[dof] = true;
      const sameSide = this.warmAngPrevSide[dof] === sideCode;
      acc = sameSide ? Math.max(lo, Math.min(hi, f(this.warmAng[dof] * Joint.warmStartFactor))) : 0;
      this.warmAngPrevSide[dof] = sideCode;
    }
    this.addRow(axis, Vec3.Zero, Vec3.Zero, true, lo, hi, this.splitBias ? 0 : targetVel, this.splitBias ? targetVel : 0,
      k > 0 ? f(1 / k) : 0, acc, dof, warm);
  }

  /** Explicit spring impulses (only when springAsMotorRow is false; the default returns immediately). */
  applySprings(dt: number) {
    const bodyA = this.bodyA, bodyB = this.bodyB;
    if (!bodyA || !bodyB) return;
    const hasLin = this.springLinear.lengthSquared > 0;
    const hasAng = this.springAngular.lengthSquared > 0;
    if (!hasLin && !hasAng) return;
    if (Joint.springAsMotorRow) return; // (the C# code computes rA/rB/linDelta first; they are pure)
    const invDtSpring = dt > 0 ? f(1 / dt) : 0;

    const rA = this.anchorA.sub(bodyA.centerOfMass);
    const rB = this.anchorB.sub(bodyB.centerOfMass);
    const linDelta = this.anchorB.sub(this.anchorA);

    if (Joint.springAsMotorRow) return;

    if (hasLin) {
      for (let i = 0; i < 3; i++) {
        const k = this.springLinear.get(i);
        if (k <= 0) continue;
        const axis = this.axesA[i];
        const eq = clampToLimit(0, this.linearLowerLimit.get(i), this.linearUpperLimit.get(i));
        const err = f(linDelta.dot(axis) - eq);
        let impulse = f(f(-k * err) * dt);
        const rAxn = Vec3.cross(rA, axis);
        const rBxn = Vec3.cross(rB, axis);
        const invMEff = f(
          f(f(bodyA.inverseMass + bodyB.inverseMass) + rAxn.dot(bodyA.inverseInertiaWorld.mulVec(rAxn))) +
            rBxn.dot(bodyB.inverseInertiaWorld.mulVec(rBxn)),
        );
        impulse = clampSpringImpulse(impulse, err, invMEff, invDtSpring);
        const P = axis.mul(impulse);
        bodyA.applyImpulse(P.neg(), rA);
        bodyB.applyImpulse(P, rB);
      }
    }

    if (hasAng) {
      const qRel = this.worldA.rotation.conjugated().mul(this.worldB.rotation);
      const euler = Joint.toEulerXYZ(qRel.normalized);
      for (let i = 0; i < 3; i++) {
        const k = this.springAngular.get(i);
        if (k <= 0) continue;
        const axis = this.axesA[i];
        const eq = clampToLimit(0, this.angularLowerLimit.get(i), this.angularUpperLimit.get(i));
        const err = f(euler.get(i) - eq);
        let impulse = f(f(-k * err) * dt);
        const invMEff = f(axis.dot(bodyA.inverseInertiaWorld.mulVec(axis)) + axis.dot(bodyB.inverseInertiaWorld.mulVec(axis)));
        impulse = clampSpringImpulse(impulse, err, invMEff, invDtSpring);
        const L = axis.mul(impulse);
        bodyA.applyTorqueImpulse(L.neg());
        bodyB.applyTorqueImpulse(L);
      }
    }
  }

  /** @internal True once the first Prepare captured the bind axes (needed before the kernel path). */
  get hasBindAxes() {
    return this.bindAxes !== null;
  }
  /** @internal Upper bound of rows one Prepare can create. */
  get maxRows() {
    return 6;
  }
  /** @internal Rows of this joint were written by the world's kernel into `store` at [start, start+count). */
  adoptRows(store: RowStore, start: number, count: number) {
    this.store = store;
    this.rowStart = start;
    this.rowCount = count;
  }

  /** One velocity iteration over this joint's rows. */
  solveVelocity() {
    if (this.rowCount > 0) solveRowRange(this.store, this.rowStart, this.rowStart + this.rowCount);
  }

  /** Split-impulse position iteration (pseudo velocities). */
  solveSplitPosition() {
    const bodyA = this.bodyA!, bodyB = this.bodyB!;
    const d = this.store.d;
    for (let r = this.rowStart; r < this.rowStart + this.rowCount; r++) {
      const o = r * ROW;
      const axis = new Vec3(d[o], d[o + 1], d[o + 2]);
      const relA = new Vec3(d[o + 3], d[o + 4], d[o + 5]), relB = new Vec3(d[o + 6], d[o + 7], d[o + 8]);
      const angular = d[o + 14] !== 0;
      const relVel = angular
        ? bodyB.pseudoAngularVelocity.sub(bodyA.pseudoAngularVelocity).dot(axis)
        : bodyB.pseudoLinearVelocity
            .add(Vec3.cross(bodyB.pseudoAngularVelocity, relB))
            .sub(bodyA.pseudoLinearVelocity.add(Vec3.cross(bodyA.pseudoAngularVelocity, relA)))
            .dot(axis);

      let dImpulse = f(f(d[o + 17] - relVel) * d[o + 12]);
      const old = d[o + 18];
      const acc = Math.max(d[o + 9], Math.min(d[o + 10], f(old + dImpulse)));
      d[o + 18] = acc;
      dImpulse = f(acc - old);

      if (angular) {
        const L = axis.mul(dImpulse);
        bodyA.applyPushTorqueImpulse(L.neg());
        bodyB.applyPushTorqueImpulse(L);
      } else {
        const P = axis.mul(dImpulse);
        bodyA.applyPushImpulse(P.neg(), relA);
        bodyB.applyPushImpulse(P, relB);
      }
    }
  }

  /** Bullet 2.75 matrixToEulerXYZ (matrix version). bulletElem = read the transpose like btGetMatrixElem. */
  static toEulerXYZBullet(m: Matrix3x3, bulletElem: boolean): Vec3 {
    const e02 = bulletElem ? m.row2.x : m.row0.z;
    const e12 = bulletElem ? m.row2.y : m.row1.z;
    const e22 = m.row2.z;
    const e01 = bulletElem ? m.row1.x : m.row0.y;
    const e00 = m.row0.x;
    const e10 = bulletElem ? m.row0.y : m.row1.x;
    const e11 = m.row1.y;

    let x: number, y: number, z: number;
    if (e02 < 1) {
      if (e02 > -1) {
        x = f(Math.atan2(-e12, e22));
        y = f(Math.asin(e02));
        z = f(Math.atan2(-e01, e00));
      } else {
        x = -f(Math.atan2(e10, e11));
        y = -F_HALF_PI;
        z = 0;
      }
    } else {
      x = f(Math.atan2(e10, e11));
      y = F_HALF_PI;
      z = 0;
    }
    return new Vec3(x, y, z);
  }

  /** XYZ Euler extraction from a quaternion (the engine's own convention). */
  static toEulerXYZ(q: Quat): Vec3 {
    const m = Matrix3x3.fromQuat(q);
    const m02 = m.row0.z;
    let y: number, x: number, z: number;
    if (m02 < F_ONE_MINUS_1EM6) {
      if (m02 > F_MINUS_ONE_PLUS_1EM6) {
        y = f(Math.asin(Math.max(-1, Math.min(1, m02))));
        x = f(Math.atan2(-m.row1.z, m.row2.z));
        z = f(Math.atan2(-m.row0.y, m.row0.x));
      } else {
        y = -F_HALF_PI;
        x = -f(Math.atan2(m.row1.x, m.row1.y));
        z = 0;
      }
    } else {
      y = F_HALF_PI;
      x = f(Math.atan2(m.row1.x, m.row1.y));
      z = 0;
    }
    return new Vec3(x, y, z);
  }
}

/** v·(I v) with I = inverseInertiaWorld in solver state s (Vec3.Dot(Matrix3x3 * v) order). */
function inertiaDot(s: Float64Array, x: number, y: number, z: number) {
  const ix = f(f(f(s[6] * x) + f(s[7] * y)) + f(s[8] * z));
  const iy = f(f(f(s[9] * x) + f(s[10] * y)) + f(s[11] * z));
  const iz = f(f(f(s[12] * x) + f(s[13] * y)) + f(s[14] * z));
  return f(f(f(x * ix) + f(y * iy)) + f(z * iz));
}
/** c·(I c) with c = r x n. */
function crossInertiaDot(s: Float64Array, r: Vec3, n: Vec3) {
  const cx = f(f(r.y * n.z) - f(r.z * n.y));
  const cy = f(f(r.z * n.x) - f(r.x * n.z));
  const cz = f(f(r.x * n.y) - f(r.y * n.x));
  return inertiaDot(s, cx, cy, cz);
}

const PF = new Float64Array(32); // prepareFast scratch: q worldA 0..3, q worldB 4..7, tmp 8..10, euler 12..14, mix 16..24

/** out[o..o+3] = (a * b).Normalized  (Quat operator * then Normalized) */
function qMulNormalized(a: Quat, b: Quat, out: Float64Array, o: number) {
  const x = f(f(f(f(a.w * b.x) + f(a.x * b.w)) + f(a.y * b.z)) - f(a.z * b.y));
  const y = f(f(f(f(a.w * b.y) - f(a.x * b.z)) + f(a.y * b.w)) + f(a.z * b.x));
  const z = f(f(f(f(a.w * b.z) + f(a.x * b.y)) - f(a.y * b.x)) + f(a.z * b.w));
  const w = f(f(f(f(a.w * b.w) - f(a.x * b.x)) - f(a.y * b.y)) - f(a.z * b.z));
  const len = f(Math.sqrt(f(f(f(f(x * x) + f(y * y)) + f(z * z)) + f(w * w))));
  if (len > 0) {
    const i = f(1 / len);
    out[o] = f(x * i); out[o + 1] = f(y * i); out[o + 2] = f(z * i); out[o + 3] = f(w * i);
  } else {
    out[o] = 0; out[o + 1] = 0; out[o + 2] = 0; out[o + 3] = 1;
  }
}

/** out[o..o+2] = q * v  (Quat operator *(Quat, Vec3)) */
function qRotate(q: Quat, ix: number, iy: number, iz: number, out: Float64Array, o: number) {
  const qx = q.x, qy = q.y, qz = q.z, qw = q.w;
  const t = f(2 / f(f(f(f(qx * qx) + f(qy * qy)) + f(qz * qz)) + f(qw * qw)));
  out[o] = f(f(f(f(1 - f(t * f(f(qy * qy) + f(qz * qz)))) * ix) + f(f(t * f(f(qx * qy) - f(qw * qz))) * iy)) + f(f(t * f(f(qx * qz) + f(qw * qy))) * iz));
  out[o + 1] = f(f(f(f(t * f(f(qx * qy) + f(qw * qz))) * ix) + f(f(1 - f(t * f(f(qx * qx) + f(qz * qz)))) * iy)) + f(f(t * f(f(qy * qz) - f(qw * qx))) * iz));
  out[o + 2] = f(f(f(f(t * f(f(qx * qz) - f(qw * qy))) * ix) + f(f(t * f(f(qy * qz) + f(qw * qx))) * iy)) + f(f(1 - f(t * f(f(qx * qx) + f(qy * qy)))) * iz));
}

/** Matrix3x3.FromQuat of the quaternion stored at q[o..o+3]. */
function m3FromQuat4(q: Float64Array, o: number, out: Float64Array) {
  const x = q[o], y = q[o + 1], z = q[o + 2], w = q[o + 3];
  const xx = f(x * x), yy = f(y * y), zz = f(z * z);
  const xy = f(x * y), xz = f(x * z), yz = f(y * z);
  const wx = f(w * x), wy = f(w * y), wz = f(w * z);
  out[0] = f(1 - f(2 * f(yy + zz))); out[1] = f(2 * f(xy - wz)); out[2] = f(2 * f(xz + wy));
  out[3] = f(2 * f(xy + wz)); out[4] = f(1 - f(2 * f(xx + zz))); out[5] = f(2 * f(yz - wx));
  out[6] = f(2 * f(xz - wy)); out[7] = f(2 * f(yz + wx)); out[8] = f(1 - f(2 * f(xx + yy)));
}

/** Vec3.Normalized into out[o..o+2]. */
function normalizeInto(x: number, y: number, z: number, out: Float64Array, o: number) {
  const len = f(Math.sqrt(f(f(f(x * x) + f(y * y)) + f(z * z))));
  if (len > 0) {
    const i = f(1 / len);
    out[o] = f(x * i); out[o + 1] = f(y * i); out[o + 2] = f(z * i);
  } else {
    out[o] = 0; out[o + 1] = 0; out[o + 2] = 0;
  }
}

/** Joint.ToEulerXYZBullet on a row-major array, result into out[o..o+2]. */
function eulerBulletInto(m: Float64Array, bulletElem: boolean, out: Float64Array, o: number) {
  const e02 = bulletElem ? m[6] : m[2];
  const e12 = bulletElem ? m[7] : m[5];
  const e22 = m[8];
  const e01 = bulletElem ? m[3] : m[1];
  const e00 = m[0];
  const e10 = bulletElem ? m[1] : m[3];
  const e11 = m[4];
  if (e02 < 1) {
    if (e02 > -1) {
      out[o] = f(Math.atan2(-e12, e22));
      out[o + 1] = f(Math.asin(e02));
      out[o + 2] = f(Math.atan2(-e01, e00));
    } else {
      out[o] = -f(Math.atan2(e10, e11));
      out[o + 1] = -F_HALF_PI;
      out[o + 2] = 0;
    }
  } else {
    out[o] = f(Math.atan2(e10, e11));
    out[o + 1] = F_HALF_PI;
    out[o + 2] = 0;
  }
}

// ---- scalar 3x3 helpers (row-major Float64Array(9)); identical float32 operations to math.ts Matrix3x3 ----
const S_BASIS = new Float64Array(9);
const S_BMA = new Float64Array(9);
const S_BMB = new Float64Array(9);
const S_T = new Float64Array(9);
const S_BA = new Float64Array(9);
const S_BB = new Float64Array(9);
const S_INV = new Float64Array(9);

/** Matrix3x3.FromQuat */
function m3FromQuat(q: Quat, o: Float64Array) {
  const xx = f(q.x * q.x), yy = f(q.y * q.y), zz = f(q.z * q.z);
  const xy = f(q.x * q.y), xz = f(q.x * q.z), yz = f(q.y * q.z);
  const wx = f(q.w * q.x), wy = f(q.w * q.y), wz = f(q.w * q.z);
  o[0] = f(1 - f(2 * f(yy + zz))); o[1] = f(2 * f(xy - wz)); o[2] = f(2 * f(xz + wy));
  o[3] = f(2 * f(xy + wz)); o[4] = f(1 - f(2 * f(xx + zz))); o[5] = f(2 * f(yz - wx));
  o[6] = f(2 * f(xz - wy)); o[7] = f(2 * f(yz + wx)); o[8] = f(1 - f(2 * f(xx + yy)));
}

/** Matrix3x3.FromQuatBullet */
function m3FromQuatBullet(q: Quat, o: Float64Array) {
  const d = f(f(f(f(q.x * q.x) + f(q.y * q.y)) + f(q.z * q.z)) + f(q.w * q.w));
  const sc = f(2 / d);
  const xs = f(q.x * sc), ys = f(q.y * sc), zs = f(q.z * sc);
  const wx = f(q.w * xs), wy = f(q.w * ys), wz = f(q.w * zs);
  const xx = f(q.x * xs), xy = f(q.x * ys), xz = f(q.x * zs);
  const yy = f(q.y * ys), yz = f(q.y * zs), zz = f(q.z * zs);
  o[0] = f(1 - f(yy + zz)); o[1] = f(xy - wz); o[2] = f(xz + wy);
  o[3] = f(xy + wz); o[4] = f(1 - f(xx + zz)); o[5] = f(yz - wx);
  o[6] = f(xz - wy); o[7] = f(yz + wx); o[8] = f(1 - f(xx + yy));
}

/** Row i (offset 0/3/6) of m dotted with v: Vec3.Dot order. */
function m3Row(m: Float64Array, o: number, x: number, y: number, z: number) {
  return f(f(f(m[o] * x) + f(m[o + 1] * y)) + f(m[o + 2] * z));
}

/** out = a * b (Matrix3x3 operator *). out must not alias a or b. */
function m3Mul(a: Float64Array, b: Float64Array, out: Float64Array) {
  for (let i = 0; i < 9; i += 3) {
    const a0 = a[i], a1 = a[i + 1], a2 = a[i + 2];
    out[i] = f(f(f(a0 * b[0]) + f(a1 * b[3])) + f(a2 * b[6]));
    out[i + 1] = f(f(f(a0 * b[1]) + f(a1 * b[4])) + f(a2 * b[7]));
    out[i + 2] = f(f(f(a0 * b[2]) + f(a1 * b[5])) + f(a2 * b[8]));
  }
}

/** Matrix3x3.BulletInverse (cofactors / determinant). */
function m3BulletInverse(m: Float64Array, out: Float64Array) {
  const m00 = m[0], m01 = m[1], m02 = m[2], m10 = m[3], m11 = m[4], m12 = m[5], m20 = m[6], m21 = m[7], m22 = m[8];
  const cofac = (a: number, b: number, c: number, d: number) => f(f(a * b) - f(c * d));
  const cx = cofac(m11, m22, m12, m21), cy = cofac(m12, m20, m10, m22), cz = cofac(m10, m21, m11, m20);
  const det = f(f(f(m00 * cx) + f(m01 * cy)) + f(m02 * cz));
  const sc = f(1 / det);
  out[0] = f(cx * sc); out[1] = f(cofac(m02, m21, m01, m22) * sc); out[2] = f(cofac(m01, m12, m02, m11) * sc);
  out[3] = f(cy * sc); out[4] = f(cofac(m00, m22, m02, m20) * sc); out[5] = f(cofac(m02, m10, m00, m12) * sc);
  out[6] = f(cz * sc); out[7] = f(cofac(m01, m20, m00, m21) * sc); out[8] = f(cofac(m00, m11, m01, m10) * sc);
}

/** Joint.toEulerXYZBullet on a row-major array. */
function toEulerXYZBulletArr(m: Float64Array, bulletElem: boolean) {
  return Joint.toEulerXYZBullet(
    new Matrix3x3(new Vec3(m[0], m[1], m[2]), new Vec3(m[3], m[4], m[5]), new Vec3(m[6], m[7], m[8])),
    bulletElem,
  );
}

function clampSpringImpulse(impulse: number, err: number, invMEff: number, invDt: number) {
  if (invMEff <= 0 || invDt <= 0) return impulse;
  const maxImp = f(f(Math.abs(err) / invMEff) * invDt);
  const clamped = impulse > maxImp ? maxImp : impulse < -maxImp ? -maxImp : impulse;
  return Joint.disableSpringClamp ? impulse : clamped;
}
