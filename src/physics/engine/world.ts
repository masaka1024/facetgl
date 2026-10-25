// ===========================================================================
// MMD physics engine (TypeScript port) – physics world
// Port of PhysicsWorld.cs (btDiscreteDynamicsWorld equivalent): gravity, integration,
// collision, joints; sequential impulses + Baumgarte. float32 rules: see math.ts.
// Profiling counters / diagnostic row hooks of the C# source are not ported
// (debugContacts is kept because the engine harness uses it).
// ===========================================================================

import { f, clamp, Quat, RigidTransform, Vec3 } from './math';
import { ContactPoint, GjkEpa, PersistentManifold } from './collision';
import { Joint, ROW, RowStore, solveRowRange } from './constraints';
import { getSolverKernel, SolverKernel } from './wasmSolver';

function putVec(F: Float32Array, o: number, v: Vec3) {
  F[o] = v.x;
  F[o + 1] = v.y;
  F[o + 2] = v.z;
}

/** Kernel layouts (wasm/solver.c), 32-bit words. */
const K_BODY = 24, K_ROW = 17, K_CON = 26, K_JNT = 35;
import { RigidBody, applyImpulseSt } from './rigidBody';

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

  private readonly rowStore = new RowStore();
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
    const store = this.rowStore;
    store.count = 0;
    const kernelRows = this.prepareKernel(dt);
    if (!kernelRows)
      for (const j of this.joints) j.prepare(dt, this.useJointSplitImpulse, this.useJointWarmStart, this.useJointWarmStartAngular, store);
    for (const j of this.joints) j.applySprings(dt);

    this.warmStart();

    const jointIters = this.jointVelocityIterations > 0 ? this.jointVelocityIterations : this.solverIterations;
    if (!this.solveVelocitiesKernel(store, jointIters, kernelRows)) {
      const totalIters = Math.max(this.solverIterations, jointIters);
      for (let it = 0; it < totalIters; it++) {
        const doContacts = it < this.solverIterations;
        const doJoints = it < jointIters;
        if (this.solveJointsFirst) {
          // Rows of all joints are packed in joint order, so one pass == `foreach (j) j.SolveVelocity()`.
          if (doJoints) solveRowRange(store, 0, store.count);
          if (doContacts) this.solveContacts();
        } else {
          if (doContacts) this.solveContacts();
          if (doJoints) solveRowRange(store, 0, store.count);
        }
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

  // ---- float32 WebAssembly kernel (wasm/solver.c): bit-identical to the TypeScript paths ----
  // Kernel memory layout (32-bit words from heapBase): bodies | rows | contacts | joints | rowCounts
  private kOffR = 0;
  private kOffC = 0;
  private kNRows = 0;
  private readonly kParams = new Float32Array(8);

  private bodiesOwned(): boolean {
    const bodies = this.bodies;
    for (const j of this.joints) {
      const a = j.bodyA, b = j.bodyB;
      if (!a || !b || bodies[a.index] !== a || bodies[b.index] !== b) return false;
    }
    for (const c of this.contacts) if (bodies[c.a.index] !== c.a || bodies[c.b.index] !== c.b) return false;
    return true;
  }

  /** Packs all bodies (velocities, inertia, inverse mass, origin, rotation) into kernel memory. */
  private packBodies(k: SolverKernel) {
    const bodies = this.bodies, F = k.f32, base = k.heapBase >> 2;
    for (let i = 0; i < bodies.length; i++) {
      const b = bodies[i], s = b.st, o = base + i * K_BODY;
      for (let j = 0; j < 16; j++) F[o + j] = s[j];
      const t = b.worldTransform, org = t.origin, q = t.rotation;
      F[o + 16] = org.x; F[o + 17] = org.y; F[o + 18] = org.z;
      F[o + 19] = q.x; F[o + 20] = q.y; F[o + 21] = q.z; F[o + 22] = q.w;
    }
  }

  /**
   * Joint.Prepare of every joint in the kernel (default flag set only, see Joint.prepareFast).
   * Returns false (TypeScript path) when the kernel or the flag set is not applicable.
   */
  private prepareKernel(dt: number): boolean {
    if (!(Joint.bulletLimitRowGating && Joint.angularMixedAxes && Joint.bulletAngleConvention && Joint.linearLeverMode === 1 &&
          Joint.springAsMotorRow && !Joint.freezeCrossAxes && !Joint.splitCrossOnly && !Joint.splitChainOnly &&
          !this.useJointSplitImpulse && !this.useJointWarmStart && !this.useJointWarmStartAngular)) return false;
    const k = getSolverKernel();
    if (!k) return false;
    const joints = this.joints;
    for (const j of joints) if (!j.hasBindAxes) return false; // first Prepare of a joint captures its bind axes (TS path)
    if (!this.bodiesOwned()) return false;
    const nb = this.bodies.length, nj = joints.length, nc = this.contacts.length;
    let maxRows = 0;
    for (const j of joints) maxRows += j.maxRows;
    const offR = nb * K_BODY, offC = offR + maxRows * K_ROW, offJ = offC + nc * K_CON, offN = offJ + nj * K_JNT;
    const offP = offN + nj;
    k.reserve((offP + 8) * 4);
    this.packBodies(k);
    const F = k.f32, I = k.i32, base = k.heapBase >> 2;
    for (let n = 0; n < nj; n++) {
      const j = joints[n], o = base + offJ + n * K_JNT;
      const fa = j.frameInA, fb = j.frameInB;
      F[o] = fa.rotation.x; F[o + 1] = fa.rotation.y; F[o + 2] = fa.rotation.z; F[o + 3] = fa.rotation.w;
      F[o + 4] = fa.origin.x; F[o + 5] = fa.origin.y; F[o + 6] = fa.origin.z;
      F[o + 7] = fb.rotation.x; F[o + 8] = fb.rotation.y; F[o + 9] = fb.rotation.z; F[o + 10] = fb.rotation.w;
      F[o + 11] = fb.origin.x; F[o + 12] = fb.origin.y; F[o + 13] = fb.origin.z;
      putVec(F, o + 14, j.linearLowerLimit);
      putVec(F, o + 17, j.linearUpperLimit);
      putVec(F, o + 20, j.angularLowerLimit);
      putVec(F, o + 23, j.angularUpperLimit);
      putVec(F, o + 26, j.springLinear);
      putVec(F, o + 29, j.springAngular);
      F[o + 32] = j.beta;
      I[o + 33] = j.bodyA!.index;
      I[o + 34] = j.bodyB!.index;
    }
    const P = this.kParams;
    P[0] = dt > 0 ? f(1 / dt) : 0; P[1] = Joint.leverArmGate; P[2] = Joint.maxCorrectionVel; P[3] = Joint.lockedRowImpulseBound;
    P[4] = Joint.angularBetaScale; P[5] = Joint.errDeadband; P[6] = Joint.biasDeadband;
    F.set(P, base + offP);
    const iters = Joint.solverIterationsForSpring > 0 ? Joint.solverIterationsForSpring : 10;
    const B = k.heapBase;
    const nr = k.prepare(nj, B + offJ * 4, B, B + offP * 4, iters, B + offR * 4, B + offN * 4);
    // mirror the rows into the TS row store (joint row ranges stay valid for TS consumers)
    const store = this.rowStore;
    store.count = 0;
    const counts = k.i32;
    for (let n = 0; n < nj; n++) {
      const j = joints[n], c = counts[base + offN + n];
      j.adoptRows(store, store.count, c);
      for (let r = 0; r < c; r++) {
        const idx = store.alloc();
        store.bodyA[idx] = j.bodyA!;
        store.bodyB[idx] = j.bodyB!;
        store.joint[idx] = j;
      }
    }
    this.kOffR = offR;
    this.kOffC = offC;
    this.kNRows = nr;
    return true;
  }

  /**
   * Runs the velocity iterations in the kernel (bit-identical to the TS loop in subStep).
   * kernelRows: the rows were prepared in kernel memory by prepareKernel (otherwise they are packed here).
   * Returns false when the kernel is unavailable or a body is not owned by this world (TS fallback).
   */
  private solveVelocitiesKernel(store: RowStore, jointIters: number, kernelRows: boolean): boolean {
    const k = getSolverKernel();
    if (!k) return false;
    const bodies = this.bodies, nb = bodies.length;
    const contacts = this.contacts, nc = contacts.length;
    let offR: number, offC: number, nr: number;
    if (kernelRows) {
      offR = this.kOffR;
      offC = this.kOffC;
      nr = this.kNRows;
      // velocities changed since prepareKernel (contact warm start)
      const F = k.f32, base = k.heapBase >> 2;
      for (let i = 0; i < nb; i++) {
        const s = bodies[i].st, o = base + i * K_BODY;
        F[o] = s[0]; F[o + 1] = s[1]; F[o + 2] = s[2]; F[o + 3] = s[3]; F[o + 4] = s[4]; F[o + 5] = s[5];
      }
    } else {
      if (!this.bodiesOwned()) return false;
      nr = store.count;
      offR = nb * K_BODY;
      offC = offR + nr * K_ROW;
      k.reserve((offC + nc * K_CON) * 4);
      this.packBodies(k);
      const F = k.f32, I = k.i32, base = k.heapBase >> 2, d = store.d;
      for (let r = 0; r < nr; r++) {
        const o = base + offR + r * K_ROW, q = r * ROW;
        for (let j = 0; j < 15; j++) F[o + j] = d[q + j];
        I[o + 15] = store.bodyA[r].index;
        I[o + 16] = store.bodyB[r].index;
      }
    }
    const F = k.f32, I = k.i32, base = k.heapBase >> 2;
    for (let i = 0; i < nc; i++) {
      const c = contacts[i], o = base + offC + i * K_CON;
      F[o] = c.rax; F[o + 1] = c.ray; F[o + 2] = c.raz;
      F[o + 3] = c.rbx; F[o + 4] = c.rby; F[o + 5] = c.rbz;
      F[o + 6] = c.nx; F[o + 7] = c.ny; F[o + 8] = c.nz;
      const t1 = c.tangent1, t2 = c.tangent2;
      F[o + 9] = t1.x; F[o + 10] = t1.y; F[o + 11] = t1.z;
      F[o + 12] = t2.x; F[o + 13] = t2.y; F[o + 14] = t2.z;
      F[o + 15] = c.normalMass; F[o + 16] = c.tangentMass1; F[o + 17] = c.tangentMass2;
      F[o + 18] = c.normalBias; F[o + 19] = c.friction;
      F[o + 20] = c.normalImpulse; F[o + 21] = c.tangentImpulse1; F[o + 22] = c.tangentImpulse2;
      F[o + 23] = c.useTangent2 ? 1 : 0;
      I[o + 24] = c.a.index;
      I[o + 25] = c.b.index;
    }
    const flags = (this.solveJointsFirst ? 1 : 0) | (this.contactPoolOrder ? 2 : 0) | (this.contactNormalBeforeFriction ? 4 : 0);
    k.solve(k.heapBase, k.heapBase + offR * 4, nr, k.heapBase + offC * 4, nc, this.solverIterations, jointIters, flags);
    // results: velocities, contact impulses, rows (accumulators; full rows when they were prepared in the kernel)
    for (let i = 0; i < nb; i++) {
      const s = bodies[i].st, o = base + i * K_BODY;
      for (let j = 0; j < 6; j++) s[j] = F[o + j];
    }
    for (let i = 0; i < nc; i++) {
      const c = contacts[i], o = base + offC + i * K_CON;
      c.normalImpulse = F[o + 20];
      c.tangentImpulse1 = F[o + 21];
      c.tangentImpulse2 = F[o + 22];
    }
    const d = store.d;
    if (kernelRows) {
      for (let r = 0; r < nr; r++) {
        const o = base + offR + r * K_ROW, q = r * ROW;
        for (let j = 0; j < 15; j++) d[q + j] = F[o + j];
        d[q + 15] = 0; d[q + 16] = 0; d[q + 17] = 0; d[q + 18] = 0; // dof is not tracked (no warm start on this path)
      }
      return true;
    }
    for (let r = 0; r < nr; r++) {
      const q = r * ROW;
      const acc = F[base + offR + r * K_ROW + 13];
      d[q + 13] = acc;
      if (d[q + 16] !== 0) {
        const j = store.joint[r];
        if (d[q + 14] !== 0) j.warmAng[d[q + 15]] = acc;
        else j.warmLin[d[q + 15]] = acc;
      }
    }
    return true;
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

      // v += (g + F*im)*dt;  w += (I*T)*dt;  v *= damp(lin);  w *= damp(ang)   (scalar form, same order)
      const s = b.st, g = this.gravity, F = b.totalForce, T = b.totalTorque, im = s[15];
      s[0] = f(s[0] + f(f(g.x + f(F.x * im)) * dt));
      s[1] = f(s[1] + f(f(g.y + f(F.y * im)) * dt));
      s[2] = f(s[2] + f(f(g.z + f(F.z * im)) * dt));
      const tx = f(f(f(s[6] * T.x) + f(s[7] * T.y)) + f(s[8] * T.z));
      const ty = f(f(f(s[9] * T.x) + f(s[10] * T.y)) + f(s[11] * T.z));
      const tz = f(f(f(s[12] * T.x) + f(s[13] * T.y)) + f(s[14] * T.z));
      s[3] = f(s[3] + f(tx * dt));
      s[4] = f(s[4] + f(ty * dt));
      s[5] = f(s[5] + f(tz * dt));
      const dl = dampingFactor(b.linearDamping, dt), da = dampingFactor(b.angularDamping, dt);
      s[0] = f(s[0] * dl); s[1] = f(s[1] * dl); s[2] = f(s[2] * dl);
      s[3] = f(s[3] * da); s[4] = f(s[4] * da); s[5] = f(s[5] * da);

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
      const s = b.st;
      let vlx = s[0], vly = s[1], vlz = s[2], wx = s[3], wy = s[4], wz = s[5];
      if (split) {
        const pl = b.pseudoLinearVelocity, pa = b.pseudoAngularVelocity;
        vlx = f(vlx + pl.x); vly = f(vly + pl.y); vlz = f(vlz + pl.z);
        wx = f(wx + pa.x); wy = f(wy + pa.y); wz = f(wz + pa.z);
      }
      const to = t.origin;
      const origin = new Vec3(f(to.x + f(vlx * dt)), f(to.y + f(vly * dt)), f(to.z + f(vlz * dt)));

      let rotation: Quat;
      if (PhysicsWorld.bulletRotationIntegration) {
        let fAngle = f(Math.sqrt(f(f(f(wx * wx) + f(wy * wy)) + f(wz * wz))));
        if (f(fAngle * dt) > ANGULAR_MOTION_THRESHOLD) fAngle = f(ANGULAR_MOTION_THRESHOLD / dt);
        const sc =
          fAngle < F_0001
            ? f(f(0.5 * dt) - f(f(f(f(f(f(dt * dt) * dt) * F_COEF) * fAngle) * fAngle)))
            : f(f(Math.sin(f(f(0.5 * fAngle) * dt))) / fAngle);
        // (dorn * rotation).Normalized with dorn = (w*sc, cos(angle*dt/2)) — Quat operator * and Normalized
        const ax = f(wx * sc), ay = f(wy * sc), az = f(wz * sc), aw = f(Math.cos(f(f(fAngle * dt) * 0.5)));
        const r = t.rotation, bx = r.x, by = r.y, bz = r.z, bw = r.w;
        const qx = f(f(f(f(aw * bx) + f(ax * bw)) + f(ay * bz)) - f(az * by));
        const qy = f(f(f(f(aw * by) - f(ax * bz)) + f(ay * bw)) + f(az * bx));
        const qz = f(f(f(f(aw * bz) + f(ax * by)) - f(ay * bx)) + f(az * bw));
        const qw = f(f(f(f(aw * bw) - f(ax * bx)) - f(ay * by)) - f(az * bz));
        const len = f(Math.sqrt(f(f(f(f(qx * qx) + f(qy * qy)) + f(qz * qz)) + f(qw * qw))));
        if (len > 0) {
          const inv = f(1 / len);
          rotation = new Quat(f(qx * inv), f(qy * inv), f(qz * inv), f(qw * inv));
        } else rotation = Quat.Identity;
      } else {
        const w = new Vec3(wx, wy, wz);
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
  private pairOk: Uint32Array | null = null;
  private sapOrder: Int32Array | null = null;
  private sapPack: Float64Array | null = null;
  private candBits: Uint32Array | null = null;
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
    // Bitset of candidate pairs (i < k) for the sweep-and-prune enumeration below.
    const ok = (this.pairOk = new Uint32Array(((n * n) >>> 5) + 1));
    let c = 0;
    for (let i = 0; i < n; i++)
      for (let k = i + 1; k < n; k++) {
        const a = this.bodies[i], b = this.bodies[k];
        if (a.isStaticOrKinematic && b.isStaticOrKinematic) continue;
        if (!PhysicsWorld.shouldCollide(a, b)) continue;
        this.pairA[c] = i;
        this.pairB![c] = k;
        c++;
        const bit = i * n + k;
        ok[bit >>> 5] |= 1 << (bit & 31);
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
    const detectBuffer = this.detectBuffer;
    const slack = PhysicsWorld.broadphaseSphereSlack;

    // ★Sweep and prune only *enumerates* candidates (a superset of the pairs whose AABBs overlap); every
    //   candidate then goes through exactly the C# tests (bounding-sphere prefilter, Aabb.Intersects), and the
    //   surviving pairs come out sorted by (i, k) = the order of the C# pair list.
    const np = this.filterPairs(n, bb, org, sphR, thr, prefilter, slack);
    const pairs = this.pairList;
    for (let p = 0; p < np; p++) {
      const i = pairs[p * 2], k = pairs[p * 2 + 1];
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

  private pairList = new Int32Array(0);

  /** Sweep axis = the axis with the largest variance of AABB centres (only affects the enumeration cost). */
  private static sweepAxis(bb: Float64Array, n: number) {
    let axis = 0, best = -1;
    for (let ax = 0; ax < 3; ax++) {
      let s1 = 0, s2 = 0, m = 0;
      for (let i = 0; i < n; i++) {
        const c = bb[i * 6 + ax] + bb[i * 6 + 3 + ax];
        if (Math.abs(c) < Infinity) { s1 += c; s2 += c * c; m++; }
      }
      const v = m > 0 ? s2 / m - (s1 / m) * (s1 / m) : 0;
      if (v > best) { best = v; axis = ax; }
    }
    return axis;
  }

  /** Writes the (i, k) pairs that pass the C# broadphase tests to pairList (sorted); returns their count. */
  private filterPairs(n: number, bb: Float64Array, org: Float64Array, sphR: Float64Array, thr: Float64Array, prefilter: boolean, slack: number) {
    const okBits = this.pairOk!;
    const words = ((n * n) >>> 5) + 1;
    let order = this.sapOrder;
    if (!order || order.length !== n) {
      order = this.sapOrder = new Int32Array(n);
      for (let i = 0; i < n; i++) order[i] = i;
      this.sapPack = new Float64Array(n * 8);
    }
    const axis = PhysicsWorld.sweepAxis(bb, n);
    const k = getSolverKernel();
    if (k) {
      // float32 WebAssembly kernel (same tests, same result; wasm/solver.c broadphase)
      const offBB = 0, offOrg = offBB + n * 6, offR = offOrg + n * 3, offT = offR + n, offOk = offT + n;
      const offCand = offOk + words, offOrd = offCand + words, offPack = offOrd + n, offOut = offPack + n * 7;
      k.reserve((offOut + this.pairCount * 2 + 2) * 4);
      const F = k.f32, I = k.i32, U = new Uint32Array(k.memory.buffer), base = k.heapBase >> 2;
      F.set(bb.subarray(0, n * 6), base + offBB);
      F.set(org.subarray(0, n * 3), base + offOrg);
      F.set(sphR.subarray(0, n), base + offR);
      F.set(thr.subarray(0, n), base + offT);
      U.set(okBits.subarray(0, words), base + offOk);
      I.set(order, base + offOrd);
      const B = k.heapBase;
      const cnt = k.broadphase(n, B + offBB * 4, B + offOrg * 4, B + offR * 4, B + offT * 4, B + offOk * 4, B + offCand * 4,
        B + offOrd * 4, B + offPack * 4, axis, prefilter ? 1 : 0, slack, B + offOut * 4);
      order.set(I.subarray(base + offOrd, base + offOrd + n));
      if (this.pairList.length < cnt * 2) this.pairList = new Int32Array(Math.max(cnt * 2, this.pairList.length * 2, 256));
      this.pairList.set(I.subarray(base + offOut, base + offOut + cnt * 2));
      return cnt;
    }
    if (!this.candBits || this.candBits.length < words) this.candBits = new Uint32Array(words);
    const candBits = this.candBits;
    candBits.fill(0, 0, words);
    this.sapCandidates(bb, n, okBits, candBits, axis);
    let cnt = 0;
    for (let wi = 0; wi < words; wi++) {
      let word = candBits[wi];
      while (word !== 0) {
        const t = word & -word;
        word ^= t;
        const bit = wi * 32 + (31 - Math.clz32(t));
        const i = Math.floor(bit / n), k = bit - i * n;
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
        if (this.pairList.length < (cnt + 1) * 2) {
          const nl = new Int32Array(Math.max(256, this.pairList.length * 2));
          nl.set(this.pairList);
          this.pairList = nl;
        }
        this.pairList[cnt * 2] = i;
        this.pairList[cnt * 2 + 1] = k;
        cnt++;
      }
    }
    return cnt;
  }

  /**
   * Marks in candBits every candidate pair (i < k, allowed by okBits) whose AABBs overlap, by sweep and
   * prune on the axis with the largest spread. It is only an enumeration: the caller applies the exact
   * float32 tests. Bodies with NaN bounds on the sweep axis are skipped (their exact test can never pass).
   */
  private sapCandidates(bb: Float64Array, n: number, okBits: Uint32Array, candBits: Uint32Array, axis: number) {
    const order = this.sapOrder!;
    const pack = this.sapPack!;
    // sort key per body (sweep-axis minimum; NaN bounds -> +Infinity = excluded)
    const key = pack; // first n slots used as key scratch before packing
    for (let i = 0; i < n; i++) {
      const v = bb[i * 6 + axis], w = bb[i * 6 + 3 + axis];
      key[i] = v === v && w === w ? v : Infinity;
    }
    // insertion sort by key (nearly sorted from the previous substep)
    for (let a = 1; a < n; a++) {
      const id = order[a];
      const kv = key[id];
      let b = a - 1;
      while (b >= 0 && key[order[b]] > kv) {
        order[b + 1] = order[b];
        b--;
      }
      order[b + 1] = id;
    }
    let m = 0;
    while (m < n && key[order[m]] < Infinity) m++;
    // pack bounds in sweep order: [min, max, lo1, hi1, lo2, hi2, id] (sequential inner loop)
    const a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
    for (let a = 0; a < m; a++) {
      const i = order[a], o = i * 6, p = a * 7 + n;
      pack[p] = bb[o + axis]; pack[p + 1] = bb[o + 3 + axis];
      pack[p + 2] = bb[o + a1]; pack[p + 3] = bb[o + 3 + a1];
      pack[p + 4] = bb[o + a2]; pack[p + 5] = bb[o + 3 + a2];
      pack[p + 6] = i;
    }
    for (let a = 0; a < m; a++) {
      const p = a * 7 + n;
      const maxx = pack[p + 1], lo1 = pack[p + 2], hi1 = pack[p + 3], lo2 = pack[p + 4], hi2 = pack[p + 5];
      const i = pack[p + 6];
      for (let q = p + 7, end = m * 7 + n; q < end; q += 7) {
        if (!(pack[q] <= maxx)) break;
        if (!(lo1 <= pack[q + 3] && hi1 >= pack[q + 2] && lo2 <= pack[q + 5] && hi2 >= pack[q + 4])) continue;
        const k = pack[q + 6];
        const bit = i < k ? i * n + k : k * n + i;
        const wi = bit >>> 5, mk = 1 << (bit & 31);
        if ((okBits[wi] & mk) !== 0) candBits[wi] |= mk;
      }
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
  applyImpulseSt(c.a.st, -px, -py, -pz, c.rax, c.ray, c.raz);
  applyImpulseSt(c.b.st, px, py, pz, c.rbx, c.rby, c.rbz);
}
function relVelocity(c: ContactConstraint) {
  const a = c.a, b = c.b;
  const ao = a.worldTransform.origin, bo = b.worldTransform.origin;
  const sa = a.st, sb = b.st;
  const rax = f(f(ao.x + c.rax) - ao.x), ray = f(f(ao.y + c.ray) - ao.y), raz = f(f(ao.z + c.raz) - ao.z);
  const rbx = f(f(bo.x + c.rbx) - bo.x), rby = f(f(bo.y + c.rby) - bo.y), rbz = f(f(bo.z + c.rbz) - bo.z);
  const vbx = f(sb[0] + f(f(sb[4] * rbz) - f(sb[5] * rby)));
  const vby = f(sb[1] + f(f(sb[5] * rbx) - f(sb[3] * rbz)));
  const vbz = f(sb[2] + f(f(sb[3] * rby) - f(sb[4] * rbx)));
  const vax = f(sa[0] + f(f(sa[4] * raz) - f(sa[5] * ray)));
  const vay = f(sa[1] + f(f(sa[5] * rax) - f(sa[3] * raz)));
  const vaz = f(sa[2] + f(f(sa[3] * ray) - f(sa[4] * rax)));
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

// (float)Math.Pow(1 - d, dt) is pure, so the last few results are cached (same value, no recomputation).
const DAMP_CACHE_SIZE = 64;
const dampKeyD = new Float64Array(DAMP_CACHE_SIZE).fill(NaN);
const dampKeyT = new Float64Array(DAMP_CACHE_SIZE).fill(NaN);
const dampVal = new Float64Array(DAMP_CACHE_SIZE);
function dampingFactor(damping: number, dt: number) {
  const d = clamp(damping, 0, PhysicsWorld.dampingClampMax);
  const base = f(1 - d);
  // small direct-mapped cache keyed by the exact (base, dt) pair
  const h = ((base * 1e6) ^ (dt * 1e7)) & (DAMP_CACHE_SIZE - 1);
  if (dampKeyD[h] === base && dampKeyT[h] === dt && !Object.is(base, -0)) return dampVal[h];
  const v = f(Math.pow(base, dt));
  dampKeyD[h] = base;
  dampKeyT[h] = dt;
  dampVal[h] = v;
  return v;
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
