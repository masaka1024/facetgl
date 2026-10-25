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

/** One solver row (linear or angular). Created fresh per Prepare, mutated in place by the solver (C# `ref`). */
class ConstraintRow {
  axis: Vec3 = Vec3.Zero;
  angular = false;
  relA: Vec3 = Vec3.Zero;
  relB: Vec3 = Vec3.Zero;
  lowerImpulse = 0;
  upperImpulse = 0;
  targetVel = 0;
  effMass = 0;
  accumulated = 0;
  positionBias = 0;
  pseudoAccumulated = 0;
  dof = 0;
  warmStartable = false;
  // scalar copies of axis / relA / relB for the solver loop
  ax = 0; ay = 0; az = 0;
  rax = 0; ray = 0; raz = 0;
  rbx = 0; rby = 0; rbz = 0;
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

  private rows: ConstraintRow[] = [];
  private rowCount = 0;

  /** Pooled row objects (C# stores rows by value in an array). newRow() resets every field like `new ConstraintRow{...}`. */
  private newRow(): ConstraintRow {
    let row = this.rows[this.rowCount];
    if (!row) row = this.rows[this.rowCount] = new ConstraintRow();
    row.relA = Vec3.Zero;
    row.relB = Vec3.Zero;
    row.accumulated = 0;
    row.positionBias = 0;
    row.pseudoAccumulated = 0;
    return row;
  }

  private addRow(row: ConstraintRow) {
    row.ax = row.axis.x; row.ay = row.axis.y; row.az = row.axis.z;
    row.rax = row.relA.x; row.ray = row.relA.y; row.raz = row.relA.z;
    row.rbx = row.relB.x; row.rby = row.relB.y; row.rbz = row.relB.z;
    this.rows[this.rowCount++] = row;
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
  private readonly warmLin = [0, 0, 0];
  private readonly warmLinSeen = [false, false, false];
  private warmStartAng = false;
  private readonly warmAng = [0, 0, 0];
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
  prepare(dt: number, splitBias = false, warmStart = false, warmStartAng = false) {
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
      for (let r = 0; r < this.rowCount; r++) {
        const row = this.rows[r];
        if (!row.warmStartable) continue;
        if (row.angular) {
          const L = row.axis.mul(row.accumulated);
          bodyA.applyTorqueImpulse(L.neg());
          bodyB.applyTorqueImpulse(L);
        } else {
          const P = row.axis.mul(row.accumulated);
          bodyA.applyImpulse(P.neg(), row.relA);
          bodyB.applyImpulse(P, row.relB);
        }
      }
    }
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
    const bodyA = this.bodyA!, bodyB = this.bodyB!;
    const rAxn = Vec3.cross(rA, axis);
    const rBxn = Vec3.cross(rB, axis);
    const k = f(
      f(f(f(bodyA.inverseMass + bodyB.inverseMass) + rAxn.dot(bodyA.inverseInertiaWorld.mulVec(rAxn))) +
        rBxn.dot(bodyB.inverseInertiaWorld.mulVec(rBxn))),
    );
    const warm = this.warmStart && locked;
    let acc = 0;
    if (warm) {
      this.warmLinSeen[dof] = true;
      acc = Math.max(lo, Math.min(hi, f(this.warmLin[dof] * Joint.warmStartFactor)));
    }
    const row = this.newRow();
    row.axis = axis;
    row.angular = false;
    row.relA = rA;
    row.relB = rB;
    row.lowerImpulse = lo;
    row.upperImpulse = hi;
    row.targetVel = this.splitBias ? 0 : targetVel;
    row.positionBias = this.splitBias ? targetVel : 0;
    row.effMass = k > 0 ? f(1 / k) : 0;
    row.dof = dof;
    row.warmStartable = warm;
    row.accumulated = acc;
    this.addRow(row);
  }

  private addAngularRow(axis: Vec3, targetVel: number, lo: number, hi: number, dof: number, sideCode: number) {
    const bodyA = this.bodyA!, bodyB = this.bodyB!;
    const k = f(axis.dot(bodyA.inverseInertiaWorld.mulVec(axis)) + axis.dot(bodyB.inverseInertiaWorld.mulVec(axis)));
    let warm = false;
    let acc = 0;
    if (this.warmStartAng) {
      warm = true;
      this.warmAngSeen[dof] = true;
      const sameSide = this.warmAngPrevSide[dof] === sideCode;
      acc = sameSide ? Math.max(lo, Math.min(hi, f(this.warmAng[dof] * Joint.warmStartFactor))) : 0;
      this.warmAngPrevSide[dof] = sideCode;
    }
    const row = this.newRow();
    row.axis = axis;
    row.angular = true;
    row.lowerImpulse = lo;
    row.upperImpulse = hi;
    row.targetVel = this.splitBias ? 0 : targetVel;
    row.positionBias = this.splitBias ? targetVel : 0;
    row.effMass = k > 0 ? f(1 / k) : 0;
    row.dof = dof;
    row.warmStartable = warm;
    row.accumulated = acc;
    this.addRow(row);
  }

  /** Explicit spring impulses (only when springAsMotorRow is false; the default returns immediately). */
  applySprings(dt: number) {
    const bodyA = this.bodyA, bodyB = this.bodyB;
    if (!bodyA || !bodyB) return;
    const hasLin = this.springLinear.lengthSquared > 0;
    const hasAng = this.springAngular.lengthSquared > 0;
    if (!hasLin && !hasAng) return;
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

  /** One velocity iteration over this joint's rows (scalar form of the C# Vec3 expressions, same float32 op order). */
  solveVelocity() {
    const A = this.bodyA!, B = this.bodyB!;
    const rows = this.rows;
    for (let r = 0; r < this.rowCount; r++) {
      const row = rows[r];
      const ax = row.ax, ay = row.ay, az = row.az;
      let relVel: number;
      if (row.angular) {
        // (wB - wA).Dot(axis)
        relVel = f(f(f(f(B.avx - A.avx) * ax) + f(f(B.avy - A.avy) * ay)) + f(f(B.avz - A.avz) * az));
      } else {
        // ((vB + wB x relB) - (vA + wA x relA)).Dot(axis)
        const rbx = row.rbx, rby = row.rby, rbz = row.rbz, rax = row.rax, ray = row.ray, raz = row.raz;
        const bx = f(B.lvx + f(f(B.avy * rbz) - f(B.avz * rby)));
        const by = f(B.lvy + f(f(B.avz * rbx) - f(B.avx * rbz)));
        const bz = f(B.lvz + f(f(B.avx * rby) - f(B.avy * rbx)));
        const qx = f(A.lvx + f(f(A.avy * raz) - f(A.avz * ray)));
        const qy = f(A.lvy + f(f(A.avz * rax) - f(A.avx * raz)));
        const qz = f(A.lvz + f(f(A.avx * ray) - f(A.avy * rax)));
        relVel = f(f(f(f(bx - qx) * ax) + f(f(by - qy) * ay)) + f(f(bz - qz) * az));
      }

      let dImpulse = f(f(row.targetVel - relVel) * row.effMass);
      const old = row.accumulated;
      row.accumulated = Math.max(row.lowerImpulse, Math.min(row.upperImpulse, f(old + dImpulse)));
      dImpulse = f(row.accumulated - old);

      if (dImpulse !== 0) {
        // Manually inlined RigidBody.applyTorqueImpulse / applyImpulse (A gets -P, B gets +P).
        // Kept inline on purpose: a non-inlined call boxes every double argument (GC pressure).
        const px = f(ax * dImpulse), py = f(ay * dImpulse), pz = f(az * dImpulse);
        const nx = -px, ny = -py, nz = -pz;
        if (row.angular) {
          if (A.inverseMass !== 0) {
            A.avx = f(A.avx + f(f(f(A.i00 * nx) + f(A.i01 * ny)) + f(A.i02 * nz)));
            A.avy = f(A.avy + f(f(f(A.i10 * nx) + f(A.i11 * ny)) + f(A.i12 * nz)));
            A.avz = f(A.avz + f(f(f(A.i20 * nx) + f(A.i21 * ny)) + f(A.i22 * nz)));
          }
          if (B.inverseMass !== 0) {
            B.avx = f(B.avx + f(f(f(B.i00 * px) + f(B.i01 * py)) + f(B.i02 * pz)));
            B.avy = f(B.avy + f(f(f(B.i10 * px) + f(B.i11 * py)) + f(B.i12 * pz)));
            B.avz = f(B.avz + f(f(f(B.i20 * px) + f(B.i21 * py)) + f(B.i22 * pz)));
          }
        } else {
          const ima = A.inverseMass;
          if (ima !== 0) {
            const rx = row.rax, ry = row.ray, rz = row.raz;
            A.lvx = f(A.lvx + f(nx * ima));
            A.lvy = f(A.lvy + f(ny * ima));
            A.lvz = f(A.lvz + f(nz * ima));
            const cx = f(f(ry * nz) - f(rz * ny));
            const cy = f(f(rz * nx) - f(rx * nz));
            const cz = f(f(rx * ny) - f(ry * nx));
            A.avx = f(A.avx + f(f(f(A.i00 * cx) + f(A.i01 * cy)) + f(A.i02 * cz)));
            A.avy = f(A.avy + f(f(f(A.i10 * cx) + f(A.i11 * cy)) + f(A.i12 * cz)));
            A.avz = f(A.avz + f(f(f(A.i20 * cx) + f(A.i21 * cy)) + f(A.i22 * cz)));
          }
          const imb = B.inverseMass;
          if (imb !== 0) {
            const rx = row.rbx, ry = row.rby, rz = row.rbz;
            B.lvx = f(B.lvx + f(px * imb));
            B.lvy = f(B.lvy + f(py * imb));
            B.lvz = f(B.lvz + f(pz * imb));
            const cx = f(f(ry * pz) - f(rz * py));
            const cy = f(f(rz * px) - f(rx * pz));
            const cz = f(f(rx * py) - f(ry * px));
            B.avx = f(B.avx + f(f(f(B.i00 * cx) + f(B.i01 * cy)) + f(B.i02 * cz)));
            B.avy = f(B.avy + f(f(f(B.i10 * cx) + f(B.i11 * cy)) + f(B.i12 * cz)));
            B.avz = f(B.avz + f(f(f(B.i20 * cx) + f(B.i21 * cy)) + f(B.i22 * cz)));
          }
        }
      }
      if (row.warmStartable) {
        if (row.angular) this.warmAng[row.dof] = row.accumulated;
        else this.warmLin[row.dof] = row.accumulated;
      }
    }
  }

  /** Split-impulse position iteration (pseudo velocities). */
  solveSplitPosition() {
    const bodyA = this.bodyA!, bodyB = this.bodyB!;
    for (let r = 0; r < this.rowCount; r++) {
      const row = this.rows[r];
      const relVel = row.angular
        ? bodyB.pseudoAngularVelocity.sub(bodyA.pseudoAngularVelocity).dot(row.axis)
        : bodyB.pseudoLinearVelocity
            .add(Vec3.cross(bodyB.pseudoAngularVelocity, row.relB))
            .sub(bodyA.pseudoLinearVelocity.add(Vec3.cross(bodyA.pseudoAngularVelocity, row.relA)))
            .dot(row.axis);

      let dImpulse = f(f(row.positionBias - relVel) * row.effMass);
      const old = row.pseudoAccumulated;
      row.pseudoAccumulated = Math.max(row.lowerImpulse, Math.min(row.upperImpulse, f(old + dImpulse)));
      dImpulse = f(row.pseudoAccumulated - old);

      if (row.angular) {
        const L = row.axis.mul(dImpulse);
        bodyA.applyPushTorqueImpulse(L.neg());
        bodyB.applyPushTorqueImpulse(L);
      } else {
        const P = row.axis.mul(dImpulse);
        bodyA.applyPushImpulse(P.neg(), row.relA);
        bodyB.applyPushImpulse(P, row.relB);
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
