// ===========================================================================
// MMD physics engine (TypeScript port) – narrowphase and contact manifolds
// Port of Collision.cs (analytic sphere/box/capsule pairs + GJK/EPA for box×box,
// btPersistentManifold-compatible point management). float32 rules: see math.ts.
// ===========================================================================

import { f, clamp, FLT_MAX, Vec3 } from './math';
import { RigidBody } from './rigidBody';
import { BoxShape, CapsuleShape, CollisionShape, ShapeType, SphereShape } from './shapes';

/** One contact point (world space). Mutable like the C# struct: copy with clone() where C# copies. */
export class ContactPoint {
  positionWorldA: Vec3 = Vec3.Zero;
  positionWorldB: Vec3 = Vec3.Zero;
  localPointA: Vec3 = Vec3.Zero;
  localPointB: Vec3 = Vec3.Zero;
  /** Unit normal from A to B. */
  normal: Vec3 = Vec3.Zero;
  /** Negative = penetration. */
  distance = 0;
  confirmedThisStep = false;
  normalImpulse = 0;
  tangentImpulse1 = 0;
  tangentImpulse2 = 0;

  clone() {
    const c = new ContactPoint();
    c.positionWorldA = this.positionWorldA;
    c.positionWorldB = this.positionWorldB;
    c.localPointA = this.localPointA;
    c.localPointB = this.localPointB;
    c.normal = this.normal;
    c.distance = this.distance;
    c.confirmedThisStep = this.confirmedThisStep;
    c.normalImpulse = this.normalImpulse;
    c.tangentImpulse1 = this.tangentImpulse1;
    c.tangentImpulse2 = this.tangentImpulse2;
    return c;
  }
}

const F_004 = f(0.04);
const F_002 = f(0.02);
const F_0004SQ = f(F_004 * F_004);
const MERGE_DIST2 = f(F_002 * F_002);

/** Contact point set of one body pair (max 4 points), kept across frames. */
export class PersistentManifold {
  bodyA: RigidBody;
  bodyB: RigidBody;
  readonly points: ContactPoint[] = [];

  constructor(a: RigidBody, b: RigidBody) {
    this.bodyA = a;
    this.bodyB = b;
  }

  /** Task 51: Bullet 2.75 btPersistentManifold point management (default true). */
  static bulletManifoldPoints = true;
  /** Task 67/68: freshness-gated symmetric breaking distance (default true). */
  static symmetricBreakingDistance = true;

  private get breakingThreshold() {
    return Math.min(GjkEpa.bulletBreakingThresholdOf(this.bodyA.shape), GjkEpa.bulletBreakingThresholdOf(this.bodyB.shape));
  }

  private refreshBullet() {
    const thr = this.breakingThreshold;
    const thr2 = f(thr * thr);
    const pts = this.points;
    for (let i = pts.length - 1; i >= 0; i--) {
      // In-place mutation is equivalent to the C# copy here: a dropped point is removed anyway.
      const cp = pts[i];
      const worldA = this.bodyA.worldTransform.transformPoint(cp.localPointA);
      const worldB = this.bodyB.worldTransform.transformPoint(cp.localPointB);
      const d = worldA.sub(worldB).dot(cp.normal);
      cp.positionWorldA = worldA;
      cp.positionWorldB = worldB;
      cp.distance = d;
      if (d > thr) {
        pts.splice(i, 1);
        continue;
      }
      const projected = worldA.sub(cp.normal.mul(d));
      const diff2 = worldB.sub(projected);
      if (diff2.lengthSquared > thr2) {
        pts.splice(i, 1);
        continue;
      }
      cp.confirmedThisStep = false;
    }
  }

  /** Task 68: drop deep points not confirmed by this frame's narrowphase. Call after Refresh → Detect → AddPoint. */
  pruneStaleDeep() {
    if (!PersistentManifold.symmetricBreakingDistance) return;
    const thr = PersistentManifold.bulletManifoldPoints ? this.breakingThreshold : F_004;
    const pts = this.points;
    for (let i = pts.length - 1; i >= 0; i--) {
      const cp = pts[i];
      if (cp.confirmedThisStep || cp.distance >= -thr) continue;
      pts.splice(i, 1);
    }
  }

  private addPointBullet(cp: ContactPoint) {
    const bt = this.breakingThreshold;
    let shortest = f(bt * bt);
    let near = -1;
    const pts = this.points;
    for (let i = 0; i < pts.length; i++) {
      const d2 = pts[i].localPointA.sub(cp.localPointA).lengthSquared;
      if (d2 < shortest) {
        shortest = d2;
        near = i;
      }
    }
    cp.confirmedThisStep = true;
    if (near >= 0) {
      cp.normalImpulse = pts[near].normalImpulse;
      cp.tangentImpulse1 = pts[near].tangentImpulse1;
      cp.tangentImpulse2 = pts[near].tangentImpulse2;
      pts[near] = cp;
      return;
    }
    if (pts.length < 4) {
      pts.push(cp);
      return;
    }
    pts[this.sortCachedPoints(cp)] = cp;
  }

  private sortCachedPoints(pt: ContactPoint) {
    const P = this.points;
    let maxPenetrationIndex = -1;
    let maxPenetration = pt.distance;
    for (let i = 0; i < 4; i++)
      if (P[i].distance < maxPenetration) {
        maxPenetrationIndex = i;
        maxPenetration = P[i].distance;
      }

    let res0 = 0, res1 = 0, res2 = 0, res3 = 0;
    const la = pt.localPointA;
    if (maxPenetrationIndex !== 0)
      res0 = Vec3.cross(la.sub(P[1].localPointA), P[3].localPointA.sub(P[2].localPointA)).lengthSquared;
    if (maxPenetrationIndex !== 1)
      res1 = Vec3.cross(la.sub(P[0].localPointA), P[3].localPointA.sub(P[2].localPointA)).lengthSquared;
    if (maxPenetrationIndex !== 2)
      res2 = Vec3.cross(la.sub(P[0].localPointA), P[3].localPointA.sub(P[1].localPointA)).lengthSquared;
    if (maxPenetrationIndex !== 3)
      res3 = Vec3.cross(la.sub(P[0].localPointA), P[2].localPointA.sub(P[1].localPointA)).lengthSquared;

    let best = 0;
    let bv = res0;
    if (res1 > bv) { bv = res1; best = 1; }
    if (res2 > bv) { bv = res2; best = 2; }
    if (res3 > bv) { bv = res3; best = 3; }
    return best;
  }

  refresh() {
    if (PersistentManifold.bulletManifoldPoints) {
      this.refreshBullet();
      return;
    }
    const pts = this.points;
    for (let i = pts.length - 1; i >= 0; i--) {
      const cp = pts[i];
      const worldA = this.bodyA.worldTransform.transformPoint(cp.localPointA);
      const worldB = this.bodyB.worldTransform.transformPoint(cp.localPointB);
      const diff = worldA.sub(worldB);
      const d = diff.dot(cp.normal);
      const lateral = diff.sub(cp.normal.mul(d));
      if (d > F_004 || lateral.lengthSquared > F_0004SQ) {
        pts.splice(i, 1);
        continue;
      }
      cp.positionWorldA = worldA;
      cp.positionWorldB = worldB;
      cp.distance = d;
      cp.confirmedThisStep = false;
    }
  }

  addPoint(cp: ContactPoint) {
    if (PersistentManifold.bulletManifoldPoints) {
      this.addPointBullet(cp);
      return;
    }
    cp.confirmedThisStep = true;
    const pts = this.points;
    for (let i = 0; i < pts.length; i++) {
      if (pts[i].positionWorldA.sub(cp.positionWorldA).lengthSquared < MERGE_DIST2) {
        cp.normalImpulse = pts[i].normalImpulse;
        cp.tangentImpulse1 = pts[i].tangentImpulse1;
        cp.tangentImpulse2 = pts[i].tangentImpulse2;
        pts[i] = cp;
        return;
      }
    }
    if (pts.length < 4) pts.push(cp);
    else pts[this.worstPointIndex()] = cp;
  }

  private worstPointIndex() {
    const pts = this.points;
    let worst = 0;
    let maxDist = pts[0].distance;
    for (let i = 1; i < pts.length; i++)
      if (pts[i].distance > maxDist) {
        maxDist = pts[i].distance;
        worst = i;
      }
    return worst;
  }
}

// ---------------------------------------------------------------------------
// GJK / EPA and analytic pair tests
// ---------------------------------------------------------------------------

const MAX_ITERATIONS = 32;
const EPSILON = f(1e-7);
const CONTACT_EPS = f(1e-9);
const CAPSULE_PARALLEL_SIN_SQ = f(1e-3);
const MAX_FACES = 128;
const F_1EM12 = f(1e-12);
const F_1EM8 = f(1e-8);
const F_1EM3 = f(1e-3);
const F_1EM5 = f(1e-5);
const F_1EM6 = f(1e-6);
const F_09 = f(0.9);

interface SupportVert {
  v: Vec3;
  a: Vec3;
  b: Vec3;
}

interface Face {
  a: number;
  b: number;
  c: number;
  normal: Vec3;
  dist: number;
  valid: boolean;
}

const INVALID_FACE: Face = { a: 0, b: 0, c: 0, normal: Vec3.Zero, dist: 0, valid: false };

function worldSupport(body: RigidBody, dirWorld: Vec3): Vec3 {
  const localDir = body.worldTransform.inverseTransformDirection(dirWorld);
  const localP = body.shape.localSupportWithMargin(localDir);
  return body.worldTransform.transformPoint(localP);
}

function support(a: RigidBody, b: RigidBody, dir: Vec3): SupportVert {
  const wa = worldSupport(a, dir);
  const wb = worldSupport(b, dir.neg());
  return { v: wa.sub(wb), a: wa, b: wb };
}

/** Per-pair acceptance threshold, decided at the top of Detect (single-threaded). */
let pairThreshold = F_002;

/** `ref Vec3 dir` holder for the GJK helpers. */
interface DirRef {
  dir: Vec3;
}

export class GjkEpa {
  /** Speculative contact margin (keep 0.02). */
  static speculativeMargin = F_002;
  static readonly speculativeMarginDefault = F_002;
  /** Task 38: Bullet 2.75 size-proportional contact threshold (default true). */
  static bulletContactThreshold = true;
  /** Bullet 2.75 gContactThresholdFactor. */
  static readonly contactThresholdFactor = F_002;

  /** Diagnostic counters (kept because the C# harness reads them). */
  static epaIterCapHits = 0;
  static epaFaceCapHits = 0;

  static bulletBreakingThresholdOf(s: CollisionShape) {
    return bulletBreakingThreshold(s);
  }

  /** Acceptance threshold of one side (Detect uses the min of both sides). */
  static acceptThresholdOf(s: CollisionShape) {
    return GjkEpa.bulletContactThreshold ? bulletBreakingThreshold(s) : GjkEpa.speculativeMargin;
  }

  /** Detects contacts between A and B and appends them to outPoints (normal A→B, distance < 0 = penetration). */
  static detect(a: RigidBody, b: RigidBody, outPoints: ContactPoint[]) {
    pairThreshold = Math.min(GjkEpa.acceptThresholdOf(a.shape), GjkEpa.acceptThresholdOf(b.shape));

    const ta = a.shape.type;
    const tb = b.shape.type;

    if (ta === ShapeType.Sphere && tb === ShapeType.Sphere) sphereSphere(a, b, outPoints);
    else if (ta === ShapeType.Sphere && tb === ShapeType.Capsule) sphereCapsule(a, b, true, outPoints);
    else if (ta === ShapeType.Capsule && tb === ShapeType.Sphere) sphereCapsule(b, a, false, outPoints);
    else if (ta === ShapeType.Capsule && tb === ShapeType.Capsule) capsuleCapsule(a, b, outPoints);
    else if (ta === ShapeType.Sphere && tb === ShapeType.Box) sphereBox(a, b, true, outPoints);
    else if (ta === ShapeType.Box && tb === ShapeType.Sphere) sphereBox(b, a, false, outPoints);
    else if (ta === ShapeType.Capsule && tb === ShapeType.Box) capsuleBox(a, b, true, outPoints);
    else if (ta === ShapeType.Box && tb === ShapeType.Capsule) capsuleBox(b, a, false, outPoints);
    else {
      const cp = gjkEpaPenetration(a, b);
      if (cp) outPoints.push(cp);
    }
  }
}

function bulletBreakingThreshold(s: CollisionShape) {
  let h: Vec3;
  if (s instanceof SphereShape) h = new Vec3(s.radius, s.radius, s.radius);
  else if (s instanceof BoxShape) h = s.halfExtents;
  else if (s instanceof CapsuleShape) {
    const m = CollisionShape.bulletConvexDistanceMargin;
    h = new Vec3(f(s.radius + m), f(f(s.halfHeight + s.radius) + m), f(s.radius + m));
  } else {
    const r = (s as CollisionShape).boundingRadius;
    h = new Vec3(r, r, r);
  }
  return f(h.length * GjkEpa.contactThresholdFactor);
}

function emit(A: RigidBody, B: RigidBody, outPoints: ContactPoint[], normalAtoB: Vec3, sep: number, pA: Vec3, pB: Vec3) {
  const cp = new ContactPoint();
  cp.normal = normalAtoB;
  cp.distance = sep;
  cp.positionWorldA = pA;
  cp.positionWorldB = pB;
  cp.localPointA = A.worldTransform.inverseTransformPoint(pA);
  cp.localPointB = B.worldTransform.inverseTransformPoint(pB);
  outPoints.push(cp);
}

function sphereSphere(a: RigidBody, b: RigidBody, outPoints: ContactPoint[]) {
  const cA = a.worldTransform.origin;
  const rA = (a.shape as SphereShape).radius;
  const cB = b.worldTransform.origin;
  const rB = (b.shape as SphereShape).radius;
  const dab = cB.sub(cA);
  const rsum = f(rA + rB);
  const dist2 = dab.lengthSquared;
  const rlim = f(rsum + pairThreshold);
  if (dist2 >= f(rlim * rlim)) return;

  const dist = f(Math.sqrt(dist2));
  const n = dist > CONTACT_EPS ? dab.div(dist) : Vec3.YAxis;
  const sep = f(dist - rsum);
  emit(a, b, outPoints, n, sep, cA.add(n.mul(rA)), cB.sub(n.mul(rB)));
}

function sphereCapsule(sphere: RigidBody, capsule: RigidBody, sphereIsA: boolean, outPoints: ContactPoint[]) {
  const sc = sphere.worldTransform.origin;
  const sr = (sphere.shape as SphereShape).radius;
  const [q0, q1, cr] = capsuleSegment(capsule);

  const cc = closestPtPointSegment(sc, q0, q1);
  const d = cc.sub(sc);
  const rsum = f(sr + cr);
  const dist = d.length;
  if (dist >= f(rsum + pairThreshold)) return;

  const nSphereToCap = dist > CONTACT_EPS ? d.div(dist) : Vec3.YAxis;
  const sep = f(dist - rsum);
  const pSphere = sc.add(nSphereToCap.mul(sr));
  const pCap = cc.sub(nSphereToCap.mul(cr));

  if (sphereIsA) emit(sphere, capsule, outPoints, nSphereToCap, sep, pSphere, pCap);
  else emit(capsule, sphere, outPoints, nSphereToCap.neg(), sep, pCap, pSphere);
}

function capsuleCapsule(a: RigidBody, b: RigidBody, outPoints: ContactPoint[]) {
  const [a0, a1, rA] = capsuleSegment(a);
  const [b0, b1, rB] = capsuleSegment(b);
  const dA = a1.sub(a0);
  const dB = b1.sub(b0);
  const lenA2 = dA.lengthSquared;
  const lenB2 = dB.lengthSquared;
  const rsum = f(rA + rB);

  const cross = Vec3.cross(dA, dB);
  const parallel =
    lenA2 > CONTACT_EPS && lenB2 > CONTACT_EPS && cross.lengthSquared <= f(f(CAPSULE_PARALLEL_SIN_SQ * lenA2) * lenB2);
  if (parallel) {
    const before = outPoints.length;
    const LA = f(Math.sqrt(lenA2));
    const dAn = dA.div(LA);
    const tb0 = b0.sub(a0).dot(dAn);
    const tb1 = b1.sub(a0).dot(dAn);
    const lo = Math.max(0, Math.min(tb0, tb1));
    const hi = Math.min(LA, Math.max(tb0, tb1));
    if (hi >= lo) {
      emitParallelPoint(a, b, outPoints, a0, dAn, lo, b0, b1, rA, rB, rsum);
      if (hi > f(lo + CONTACT_EPS)) emitParallelPoint(a, b, outPoints, a0, dAn, hi, b0, b1, rA, rB, rsum);
      if (outPoints.length > before) return;
    }
  }

  const [c1, c2] = closestPtSegmentSegment(a0, a1, b0, b1);
  const d = c2.sub(c1);
  const dist = d.length;
  if (dist >= f(rsum + pairThreshold)) return;
  const n = dist > CONTACT_EPS ? d.div(dist) : perpVector(dA);
  const sep = f(dist - rsum);
  emit(a, b, outPoints, n, sep, c1.add(n.mul(rA)), c2.sub(n.mul(rB)));
}

function emitParallelPoint(
  a: RigidBody, b: RigidBody, outPoints: ContactPoint[],
  a0: Vec3, dAn: Vec3, param: number, b0: Vec3, b1: Vec3, rA: number, rB: number, rsum: number,
) {
  const cA = a0.add(dAn.mul(param));
  const cB = closestPtPointSegment(cA, b0, b1);
  const d = cB.sub(cA);
  const dist = d.length;
  if (dist >= f(rsum + pairThreshold)) return;
  const n = dist > CONTACT_EPS ? d.div(dist) : perpVector(dAn);
  const sep = f(dist - rsum);
  emit(a, b, outPoints, n, sep, cA.add(n.mul(rA)), cB.sub(n.mul(rB)));
}

interface SphereBoxResult {
  nLocalBoxToSphere: Vec3;
  sep: number;
  boxSurfLocal: Vec3;
}

function solveSphereBoxLocal(local: Vec3, he: Vec3, r: number): SphereBoxResult | null {
  const inside = Math.abs(local.x) <= he.x && Math.abs(local.y) <= he.y && Math.abs(local.z) <= he.z;
  if (!inside) {
    const q = new Vec3(clamp(local.x, -he.x, he.x), clamp(local.y, -he.y, he.y), clamp(local.z, -he.z, he.z));
    const dl = local.sub(q);
    const dist = dl.length;
    if (dist >= f(r + pairThreshold)) return null;
    return {
      nLocalBoxToSphere: dist > CONTACT_EPS ? dl.div(dist) : Vec3.YAxis,
      boxSurfLocal: q,
      sep: f(dist - r),
    };
  }
  let best = FLT_MAX;
  let axis = 0;
  let sign = 1;
  for (let i = 0; i < 3; i++) {
    const toPos = f(he.get(i) - local.get(i));
    const toNeg = f(local.get(i) + he.get(i));
    if (toPos < best) { best = toPos; axis = i; sign = 1; }
    if (toNeg < best) { best = toNeg; axis = i; sign = -1; }
  }
  return {
    nLocalBoxToSphere: Vec3.Zero.with(axis, sign),
    boxSurfLocal: local.with(axis, f(sign * he.get(axis))),
    sep: -f(best + r),
  };
}

function sphereBox(sphere: RigidBody, box: RigidBody, sphereIsA: boolean, outPoints: ContactPoint[]) {
  const sc = sphere.worldTransform.origin;
  const sr = (sphere.shape as SphereShape).radius;
  const he = (box.shape as BoxShape).halfExtents;
  const bt = box.worldTransform;

  const local = bt.inverseTransformPoint(sc);
  const res = solveSphereBoxLocal(local, he, sr);
  if (!res) return;

  const nWorldBoxToSphere = bt.transformDirection(res.nLocalBoxToSphere).normalized;
  const boxSurfWorld = bt.transformPoint(res.boxSurfLocal);
  const sphereSurfWorld = sc.sub(nWorldBoxToSphere.mul(sr));

  if (sphereIsA) emit(sphere, box, outPoints, nWorldBoxToSphere.neg(), res.sep, sphereSurfWorld, boxSurfWorld);
  else emit(box, sphere, outPoints, nWorldBoxToSphere, res.sep, boxSurfWorld, sphereSurfWorld);
}

function capsuleBox(capsule: RigidBody, box: RigidBody, capsuleIsA: boolean, outPoints: ContactPoint[]) {
  const [p0, p1, cr] = capsuleSegment(capsule);
  const he = (box.shape as BoxShape).halfExtents;
  const bt = box.worldTransform;

  const q0 = bt.inverseTransformPoint(p0);
  const q1 = bt.inverseTransformPoint(p1);

  let boxPt = new Vec3(
    clamp(f(f(q0.x + q1.x) * 0.5), -he.x, he.x),
    clamp(f(f(q0.y + q1.y) * 0.5), -he.y, he.y),
    clamp(f(f(q0.z + q1.z) * 0.5), -he.z, he.z),
  );
  let segPt = boxPt;
  for (let k = 0; k < 8; k++) {
    segPt = closestPtPointSegment(boxPt, q0, q1);
    const np = new Vec3(clamp(segPt.x, -he.x, he.x), clamp(segPt.y, -he.y, he.y), clamp(segPt.z, -he.z, he.z));
    if (np.sub(boxPt).lengthSquared <= F_1EM12) {
      boxPt = np;
      break;
    }
    boxPt = np;
  }

  const res = solveSphereBoxLocal(segPt, he, cr);
  if (!res) return;

  const nWorldBoxToCap = bt.transformDirection(res.nLocalBoxToSphere).normalized;
  const boxSurfWorld = bt.transformPoint(res.boxSurfLocal);
  const capSurfWorld = bt.transformPoint(segPt).sub(nWorldBoxToCap.mul(cr));

  if (capsuleIsA) emit(capsule, box, outPoints, nWorldBoxToCap.neg(), res.sep, capSurfWorld, boxSurfWorld);
  else emit(box, capsule, outPoints, nWorldBoxToCap, res.sep, boxSurfWorld, capSurfWorld);
}

function capsuleSegment(body: RigidBody): [Vec3, Vec3, number] {
  const cap = body.shape as CapsuleShape;
  const hh = cap.halfHeight;
  const p0 = body.worldTransform.transformPoint(new Vec3(0, hh, 0));
  const p1 = body.worldTransform.transformPoint(new Vec3(0, -hh, 0));
  return [p0, p1, cap.radius];
}

function closestPtPointSegment(p: Vec3, a: Vec3, b: Vec3): Vec3 {
  const ab = b.sub(a);
  const denom = ab.lengthSquared;
  if (denom < CONTACT_EPS) return a;
  let t = f(p.sub(a).dot(ab) / denom);
  t = clamp(t, 0, 1);
  return a.add(ab.mul(t));
}

/** Ericson ClosestPtSegmentSegment. Returns [c1, c2]. */
function closestPtSegmentSegment(p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3): [Vec3, Vec3] {
  const d1 = q1.sub(p1);
  const d2 = q2.sub(p2);
  const r = p1.sub(p2);
  const a = d1.lengthSquared;
  const e = d2.lengthSquared;
  const ff = d2.dot(r);
  let s: number, t: number;

  if (a <= CONTACT_EPS && e <= CONTACT_EPS) return [p1, p2];
  if (a <= CONTACT_EPS) {
    s = 0;
    t = clamp(f(ff / e), 0, 1);
  } else {
    const c = d1.dot(r);
    if (e <= CONTACT_EPS) {
      t = 0;
      s = clamp(f(-c / a), 0, 1);
    } else {
      const b = d1.dot(d2);
      const denom = f(f(a * e) - f(b * b));
      s = denom !== 0 ? clamp(f(f(f(b * ff) - f(c * e)) / denom), 0, 1) : 0;
      t = f(f(f(b * s) + ff) / e);
      if (t < 0) {
        t = 0;
        s = clamp(f(-c / a), 0, 1);
      } else if (t > 1) {
        t = 1;
        s = clamp(f(f(b - c) / a), 0, 1);
      }
    }
  }
  return [p1.add(d1.mul(s)), p2.add(d2.mul(t))];
}

function perpVector(axis: Vec3): Vec3 {
  const a = axis.normalized;
  const reference = Math.abs(a.x) < F_09 ? Vec3.XAxis : Vec3.YAxis;
  const perp = Vec3.cross(a, reference);
  const len = perp.length;
  return len > CONTACT_EPS ? perp.div(len) : Vec3.YAxis;
}

function gjkEpaPenetration(a: RigidBody, b: RigidBody): ContactPoint | null {
  const simplex: SupportVert[] = [];
  let dir = a.worldTransform.origin.sub(b.worldTransform.origin);
  if (dir.lengthSquared < EPSILON) dir = Vec3.XAxis;

  simplex.push(support(a, b, dir));
  const ref: DirRef = { dir: simplex[0].v.neg() };

  for (let iter = 0; iter < MAX_ITERATIONS; iter++) {
    if (ref.dir.lengthSquared < EPSILON) break;
    const p = support(a, b, ref.dir);
    if (p.v.dot(ref.dir) < 0) return null;
    simplex.push(p);
    if (doSimplex(simplex, ref)) return epa(a, b, simplex);
  }
  return null;
}

function doSimplex(s: SupportVert[], ref: DirRef): boolean {
  if (s.length === 2) return line(s, ref);
  if (s.length === 3) return triangle(s, ref);
  return tetrahedron(s, ref);
}

function line(s: SupportVert[], ref: DirRef): boolean {
  const a = s[1].v;
  const b = s[0].v;
  const ab = b.sub(a);
  const ao = a.neg();
  if (ab.dot(ao) > 0) ref.dir = Vec3.cross(Vec3.cross(ab, ao), ab);
  else {
    s.splice(0, 1);
    ref.dir = ao;
  }
  return false;
}

function triangle(s: SupportVert[], ref: DirRef): boolean {
  const a = s[2].v;
  const b = s[1].v;
  const c = s[0].v;
  const ab = b.sub(a);
  const ac = c.sub(a);
  const ao = a.neg();
  const abc = Vec3.cross(ab, ac);

  if (Vec3.cross(abc, ac).dot(ao) > 0) {
    if (ac.dot(ao) > 0) {
      s.splice(1, 1);
      ref.dir = Vec3.cross(Vec3.cross(ac, ao), ac);
    } else return starLine(s, ref);
  } else if (Vec3.cross(ab, abc).dot(ao) > 0) {
    return starLine(s, ref);
  } else {
    if (abc.dot(ao) > 0) ref.dir = abc;
    else {
      const t = s[0];
      s[0] = s[1];
      s[1] = t;
      ref.dir = abc.neg();
    }
    return false;
  }
  return false;
}

function starLine(s: SupportVert[], ref: DirRef): boolean {
  s.splice(0, 1);
  return line(s, ref);
}

function tetrahedron(s: SupportVert[], ref: DirRef): boolean {
  const a = s[3].v;
  const b = s[2].v;
  const c = s[1].v;
  const d = s[0].v;
  const ao = a.neg();
  const abc = Vec3.cross(b.sub(a), c.sub(a));
  const acd = Vec3.cross(c.sub(a), d.sub(a));
  const adb = Vec3.cross(d.sub(a), b.sub(a));

  if (abc.dot(ao) > 0) { s.splice(0, 1); ref.dir = abc; return triangle(s, ref); }
  if (acd.dot(ao) > 0) { s.splice(2, 1); ref.dir = acd; return triangle(s, ref); }
  if (adb.dot(ao) > 0) { s.splice(1, 1); ref.dir = adb; return triangle(s, ref); }
  return true;
}

function epa(a: RigidBody, b: RigidBody, simplex: SupportVert[]): ContactPoint | null {
  if (simplex.length < 4) {
    if (!expandToTetra(a, b, simplex)) return null;
  }

  const verts = simplex.slice();
  const faces: Face[] = [];
  addIfValid(faces, makeFaceOriented(verts, 0, 1, 2, 3));
  addIfValid(faces, makeFaceOriented(verts, 0, 1, 3, 2));
  addIfValid(faces, makeFaceOriented(verts, 0, 2, 3, 1));
  addIfValid(faces, makeFaceOriented(verts, 1, 2, 3, 0));

  let converged = false;
  let faceCap = false;
  const edges: [number, number][] = [];

  for (let iter = 0; iter < MAX_ITERATIONS; iter++) {
    const closest = tryFindClosestFace(faces);
    if (!closest) break;

    const p = support(a, b, closest.normal);
    const d = p.v.dot(closest.normal);
    if (f(d - closest.dist) < f(f(closest.dist * F_1EM3) + F_1EM5)) {
      converged = true;
      break;
    }

    edges.length = 0;
    for (let i = faces.length - 1; i >= 0; i--) {
      if (faces[i].normal.dot(p.v.sub(verts[faces[i].a].v)) > 0) {
        addEdge(edges, faces[i].a, faces[i].b);
        addEdge(edges, faces[i].b, faces[i].c);
        addEdge(edges, faces[i].c, faces[i].a);
        faces.splice(i, 1);
      }
    }
    const newIndex = verts.length;
    verts.push(p);
    for (const [e0, e1] of edges) addIfValid(faces, makeFace(verts, e0, e1, newIndex));

    if (faces.length === 0) break;
    if (faces.length > MAX_FACES) {
      faceCap = true;
      break;
    }
  }

  if (faceCap) GjkEpa.epaFaceCapHits++;
  else if (!converged) GjkEpa.epaIterCapHits++;

  const closest = tryFindClosestFace(faces);
  if (!closest) {
    const dir = b.worldTransform.origin.sub(a.worldTransform.origin);
    const nf = dir.lengthSquared > EPSILON ? dir.normalized : Vec3.YAxis;
    return fillContact(a, b, nf, -EPSILON, a.worldTransform.origin, b.worldTransform.origin);
  }

  const [baryA, baryB] = barycentricProject(verts, closest);
  return fillContact(a, b, closest.normal, -closest.dist, baryA, baryB);
}

function fillContact(a: RigidBody, b: RigidBody, normal: Vec3, distance: number, baryA: Vec3, baryB: Vec3) {
  const contact = new ContactPoint();
  contact.normal = normal;
  contact.distance = distance;
  contact.positionWorldA = baryA;
  contact.positionWorldB = baryB;
  contact.localPointA = a.worldTransform.inverseTransformPoint(baryA);
  contact.localPointB = b.worldTransform.inverseTransformPoint(baryB);
  return contact;
}

function addIfValid(faces: Face[], fc: Face) {
  if (fc.valid) faces.push(fc);
}

const EXPAND_DIRS = [Vec3.XAxis, Vec3.YAxis, Vec3.ZAxis, Vec3.XAxis.neg(), Vec3.YAxis.neg(), Vec3.ZAxis.neg()];

function expandToTetra(a: RigidBody, b: RigidBody, s: SupportVert[]) {
  for (const d of EXPAND_DIRS) {
    if (s.length >= 4) break;
    const p = support(a, b, d);
    let dup = false;
    for (const e of s)
      if (e.v.sub(p.v).lengthSquared < F_1EM8) {
        dup = true;
        break;
      }
    if (!dup) s.push(p);
  }
  return s.length >= 4;
}

function makeFace(v: SupportVert[], a: number, b: number, c: number): Face {
  let n = Vec3.cross(v[b].v.sub(v[a].v), v[c].v.sub(v[a].v));
  const len = n.length;
  if (len <= EPSILON) return INVALID_FACE;
  n = n.div(len);
  const dist = n.dot(v[a].v);
  return { a, b, c, normal: n, dist, valid: true };
}

function makeFaceOriented(v: SupportVert[], a: number, b: number, c: number, opp: number): Face {
  let n = Vec3.cross(v[b].v.sub(v[a].v), v[c].v.sub(v[a].v));
  const len = n.length;
  if (len <= EPSILON) return INVALID_FACE;
  n = n.div(len);
  if (n.dot(v[opp].v.sub(v[a].v)) > 0) {
    const t = b;
    b = c;
    c = t;
    n = n.neg();
  }
  const dist = n.dot(v[a].v);
  return { a, b, c, normal: n, dist, valid: true };
}

function tryFindClosestFace(faces: Face[]): Face | null {
  let best: Face | null = null;
  let bd = FLT_MAX;
  for (let i = 0; i < faces.length; i++) {
    const fc = faces[i];
    if (!fc.valid || fc.dist < -F_1EM6) continue;
    if (fc.dist < bd) {
      bd = fc.dist;
      best = fc;
    }
  }
  return best;
}

function addEdge(edges: [number, number][], a: number, b: number) {
  for (let i = 0; i < edges.length; i++)
    if (edges[i][0] === b && edges[i][1] === a) {
      edges.splice(i, 1);
      return;
    }
  edges.push([a, b]);
}

function barycentricProject(v: SupportVert[], fc: Face): [Vec3, Vec3] {
  const pa = v[fc.a];
  const pb = v[fc.b];
  const pc = v[fc.c];
  const proj = fc.normal.mul(fc.dist);
  const [u, vv, w] = barycentric(proj, pa.v, pb.v, pc.v);
  const onA = pa.a.mul(u).add(pb.a.mul(vv)).add(pc.a.mul(w));
  const onB = pa.b.mul(u).add(pb.b.mul(vv)).add(pc.b.mul(w));
  return [onA, onB];
}

function barycentric(p: Vec3, a: Vec3, b: Vec3, c: Vec3): [number, number, number] {
  const v0 = b.sub(a);
  const v1 = c.sub(a);
  const v2 = p.sub(a);
  const d00 = v0.dot(v0);
  const d01 = v0.dot(v1);
  const d11 = v1.dot(v1);
  const d20 = v2.dot(v0);
  const d21 = v2.dot(v1);
  const denom = f(f(d00 * d11) - f(d01 * d01));
  if (Math.abs(denom) < EPSILON) return [1, 0, 0];
  const v = f(f(f(d11 * d20) - f(d01 * d21)) / denom);
  const w = f(f(f(d00 * d21) - f(d01 * d20)) / denom);
  const u = f(f(1 - v) - w);
  return [u, v, w];
}
