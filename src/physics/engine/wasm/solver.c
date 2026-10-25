// ===========================================================================
// MMD physics engine – velocity solver kernel (WebAssembly, float32)
//
// The C# engine computes in `float`; WebAssembly has native IEEE-754 binary32 arithmetic, so this
// kernel reproduces it exactly without the per-operation Math.fround of the TypeScript code
// (each JS fround costs two conversions on the critical path; this is the engine's hottest loop).
//
// Rules (do not break them, the TS parity test checks every bit):
//   * every expression is written in exactly the C# evaluation order (left associative);
//   * build with -ffp-contract=off and without -ffast-math (no FMA, no reassociation);
//   * Math.Max / Math.Min are reproduced by fmax32 / fmin32 below (NaN-propagating, -0 < +0).
//
// Build (clang with the wasm32 target, no libc needed):  see build-wasm.mjs next to this file.
// ===========================================================================

typedef unsigned int u32;

// ---- layouts shared with world.ts (strides in 32-bit words) ----
// body:    lv[3] av[3] I[9] (row-major inverseInertiaWorld) im origin[3] rotation[4] pad = 24
// row:     axis[3] relA[3] relB[3] lo hi targetVel effMass acc angular(0/1) bodyA bodyB (as int) = 17
// contact: relA[3] relB[3] n[3] t1[3] t2[3] nMass tMass1 tMass2 nBias friction nImp tImp1 tImp2
//          useT2 bodyA bodyB (as int)                                                          = 26
#define BODY 24
#define ROW 17
#define CON 26

static inline float fmax32(float a, float b) {
  // Math.Max(float, float): NaN if either is NaN; +0 > -0.
  if (a != a) return a;
  if (b != b) return b;
  if (a > b) return a;
  if (b > a) return b;
  return __builtin_signbit(a) ? b : a;
}
static inline float fmin32(float a, float b) {
  if (a != a) return a;
  if (b != b) return b;
  if (a < b) return a;
  if (b < a) return b;
  return __builtin_signbit(a) ? a : b;
}

// RigidBody.ApplyImpulse(P, rel): v += P*im; w += I*(rel x P)
static inline void applyImpulse(float *s, float px, float py, float pz, float rx, float ry, float rz) {
  float im = s[15];
  if (im == 0.0f) return;
  s[0] = s[0] + px * im;
  s[1] = s[1] + py * im;
  s[2] = s[2] + pz * im;
  float cx = ry * pz - rz * py;
  float cy = rz * px - rx * pz;
  float cz = rx * py - ry * px;
  s[3] = s[3] + (s[6] * cx + s[7] * cy + s[8] * cz);
  s[4] = s[4] + (s[9] * cx + s[10] * cy + s[11] * cz);
  s[5] = s[5] + (s[12] * cx + s[13] * cy + s[14] * cz);
}

// RigidBody.ApplyTorqueImpulse(t): w += I*t
static inline void applyTorque(float *s, float tx, float ty, float tz) {
  if (s[15] == 0.0f) return;
  s[3] = s[3] + (s[6] * tx + s[7] * ty + s[8] * tz);
  s[4] = s[4] + (s[9] * tx + s[10] * ty + s[11] * tz);
  s[5] = s[5] + (s[12] * tx + s[13] * ty + s[14] * tz);
}

// Joint.SolveVelocity over rows [0, n) (all joints' rows packed in joint order).
static void solveRows(float *B, float *R, int n) {
  for (int r = 0; r < n; r++) {
    float *row = R + r * ROW;
    float *sa = B + ((int *)row)[15] * BODY;
    float *sb = B + ((int *)row)[16] * BODY;
    float ax = row[0], ay = row[1], az = row[2];
    int angular = row[14] != 0.0f;
    float relVel;
    if (angular) {
      relVel = (sb[3] - sa[3]) * ax + (sb[4] - sa[4]) * ay + (sb[5] - sa[5]) * az;
    } else {
      float rax = row[3], ray = row[4], raz = row[5], rbx = row[6], rby = row[7], rbz = row[8];
      float bx = sb[0] + (sb[4] * rbz - sb[5] * rby);
      float by = sb[1] + (sb[5] * rbx - sb[3] * rbz);
      float bz = sb[2] + (sb[3] * rby - sb[4] * rbx);
      float qx = sa[0] + (sa[4] * raz - sa[5] * ray);
      float qy = sa[1] + (sa[5] * rax - sa[3] * raz);
      float qz = sa[2] + (sa[3] * ray - sa[4] * rax);
      relVel = (bx - qx) * ax + (by - qy) * ay + (bz - qz) * az;
    }
    float dImpulse = (row[11] - relVel) * row[12];
    float old = row[13];
    float acc = fmax32(row[9], fmin32(row[10], old + dImpulse));
    row[13] = acc;
    dImpulse = acc - old;
    if (dImpulse != 0.0f) {
      float px = ax * dImpulse, py = ay * dImpulse, pz = az * dImpulse;
      if (angular) {
        applyTorque(sa, -px, -py, -pz);
        applyTorque(sb, px, py, pz);
      } else {
        applyImpulse(sa, -px, -py, -pz, row[3], row[4], row[5]);
        applyImpulse(sb, px, py, pz, row[6], row[7], row[8]);
      }
    }
  }
}

// (b.VelocityAtPoint(b.com + relB) - a.VelocityAtPoint(a.com + relA)) — note r = (com + rel) - com.
static inline void relVelocity(const float *sa, const float *sb, const float *c, float *out) {
  float rax = (sa[16] + c[0]) - sa[16], ray = (sa[17] + c[1]) - sa[17], raz = (sa[18] + c[2]) - sa[18];
  float rbx = (sb[16] + c[3]) - sb[16], rby = (sb[17] + c[4]) - sb[17], rbz = (sb[18] + c[5]) - sb[18];
  float vbx = sb[0] + (sb[4] * rbz - sb[5] * rby);
  float vby = sb[1] + (sb[5] * rbx - sb[3] * rbz);
  float vbz = sb[2] + (sb[3] * rby - sb[4] * rbx);
  float vax = sa[0] + (sa[4] * raz - sa[5] * ray);
  float vay = sa[1] + (sa[5] * rax - sa[3] * raz);
  float vaz = sa[2] + (sa[3] * ray - sa[4] * rax);
  out[0] = vbx - vax;
  out[1] = vby - vay;
  out[2] = vbz - vaz;
}

static inline void solveNormal(float *B, float *c) {
  float *sa = B + ((int *)c)[24] * BODY;
  float *sb = B + ((int *)c)[25] * BODY;
  float v[3];
  relVelocity(sa, sb, c, v);
  float nx = c[6], ny = c[7], nz = c[8];
  float relN = v[0] * nx + v[1] * ny + v[2] * nz;
  float dPn = (c[18] - relN) * c[15];
  float oldN = c[20];
  c[20] = fmax32(0.0f, oldN + dPn);
  dPn = c[20] - oldN;
  float px = nx * dPn, py = ny * dPn, pz = nz * dPn;
  applyImpulse(sa, -px, -py, -pz, c[0], c[1], c[2]);
  applyImpulse(sb, px, py, pz, c[3], c[4], c[5]);
}

// SolveFriction for tangent t (offset to) with mass c[mo] and accumulator c[ao].
static inline void solveFriction(float *B, float *c, int to, int mo, int ao) {
  float *sa = B + ((int *)c)[24] * BODY;
  float *sb = B + ((int *)c)[25] * BODY;
  float v[3];
  relVelocity(sa, sb, c, v);
  float tx = c[to], ty = c[to + 1], tz = c[to + 2];
  float relT = v[0] * tx + v[1] * ty + v[2] * tz;
  float dPt = -relT * c[mo];
  float maxF = c[19] * c[20];
  float old = c[ao];
  float acc = fmax32(-maxF, fmin32(maxF, old + dPt));
  c[ao] = acc;
  dPt = acc - old;
  float px = tx * dPt, py = ty * dPt, pz = tz * dPt;
  applyImpulse(sa, -px, -py, -pz, c[0], c[1], c[2]);
  applyImpulse(sb, px, py, pz, c[3], c[4], c[5]);
}

#define FRICTION1(c) solveFriction(B, c, 9, 16, 21)
#define FRICTION2(c) solveFriction(B, c, 12, 17, 22)

static void solveContacts(float *B, float *C, int n, int poolOrder, int normalFirst) {
  if (poolOrder) {
    for (int i = 0; i < n; i++) solveNormal(B, C + i * CON);
    for (int i = 0; i < n; i++) {
      float *c = C + i * CON;
      FRICTION1(c);
      if (c[23] != 0.0f) FRICTION2(c);
    }
    return;
  }
  for (int i = 0; i < n; i++) {
    float *c = C + i * CON;
    if (!normalFirst) {
      FRICTION1(c);
      if (c[23] != 0.0f) FRICTION2(c);
      solveNormal(B, c);
    } else {
      solveNormal(B, c);
      FRICTION1(c);
      if (c[23] != 0.0f) FRICTION2(c);
    }
  }
}

// PhysicsWorld.SubStep velocity iterations (contacts + joints).
// flags: bit0 solveJointsFirst, bit1 contactPoolOrder, bit2 contactNormalBeforeFriction
__attribute__((export_name("solve"))) void solve(float *bodies, float *rows, int nRows, float *contacts,
                                                 int nContacts, int contactIters, int jointIters, int flags) {
  int totalIters = contactIters > jointIters ? contactIters : jointIters;
  int jointsFirst = flags & 1, poolOrder = (flags >> 1) & 1, normalFirst = (flags >> 2) & 1;
  for (int it = 0; it < totalIters; it++) {
    int doContacts = it < contactIters;
    int doJoints = it < jointIters;
    if (jointsFirst) {
      if (doJoints) solveRows(bodies, rows, nRows);
      if (doContacts) solveContacts(bodies, contacts, nContacts, poolOrder, normalFirst);
    } else {
      if (doContacts) solveContacts(bodies, contacts, nContacts, poolOrder, normalFirst);
      if (doJoints) solveRows(bodies, rows, nRows);
    }
  }
}

// ---------------------------------------------------------------------------
// Broadphase (PhysicsWorld.BroadphaseNarrowphase pair filtering).
// Enumerates candidate pairs by sweep and prune on `axis` (only an enumeration: a superset of the
// AABB-overlapping pairs), keeps the pairs allowed by okBits (bit i*n+k, i<k), then applies exactly the
// C# tests (bounding-sphere prefilter, Aabb.Intersects) and writes the surviving pairs, sorted by (i, k)
// — the order of the C# pair list — to outPairs as (i, k). Returns the number of pairs.
//   bb: per body min[3] max[3]; org: origin[3]; sphR / thr: per body; order: persistent sweep order (in/out)
//   pack, candBits: scratch (7*n floats, ceil(n*n/32)+1 words)
// ---------------------------------------------------------------------------
__attribute__((export_name("broadphase"))) int broadphase(int n, const float *bb, const float *org, const float *sphR,
    const float *thr, const u32 *okBits, u32 *candBits, int *order, float *pack, int axis, int prefilter,
    float slack, int *outPairs) {
  // sort keys (NaN bounds on the sweep axis -> excluded, their exact test can never pass)
  int m = 0;
  // insertion sort of order[] by bb[min axis]; keys with NaN are moved to the end
  for (int a = 1; a < n; a++) {
    int id = order[a];
    float kv = bb[id * 6 + axis];
    int kvNaN = kv != kv || bb[id * 6 + 3 + axis] != bb[id * 6 + 3 + axis];
    int b = a - 1;
    while (b >= 0) {
      int ob = order[b];
      float kb = bb[ob * 6 + axis];
      int kbNaN = kb != kb || bb[ob * 6 + 3 + axis] != bb[ob * 6 + 3 + axis];
      // ob goes after id if ob is NaN-keyed and id is not, or both finite and kb > kv
      int after = kvNaN ? 0 : (kbNaN ? 1 : kb > kv);
      if (!after) break;
      order[b + 1] = ob;
      b--;
    }
    order[b + 1] = id;
  }
  while (m < n) {
    int id = order[m];
    float v = bb[id * 6 + axis], w = bb[id * 6 + 3 + axis];
    if (v != v || w != w) break;
    m++;
  }
  int a1 = (axis + 1) % 3, a2 = (axis + 2) % 3;
  for (int a = 0; a < m; a++) {
    int i = order[a];
    const float *o = bb + i * 6;
    float *p = pack + a * 7;
    p[0] = o[axis]; p[1] = o[3 + axis]; p[2] = o[a1]; p[3] = o[3 + a1]; p[4] = o[a2]; p[5] = o[3 + a2];
    ((int *)p)[6] = i;
  }
  u32 words = ((u32)n * (u32)n >> 5) + 1;
  for (u32 w = 0; w < words; w++) candBits[w] = 0;
  for (int a = 0; a < m; a++) {
    const float *p = pack + a * 7;
    float maxx = p[1], lo1 = p[2], hi1 = p[3], lo2 = p[4], hi2 = p[5];
    int i = ((const int *)p)[6];
    for (int b = a + 1; b < m; b++) {
      const float *q = pack + b * 7;
      if (!(q[0] <= maxx)) break;
      if (!(lo1 <= q[3] && hi1 >= q[2] && lo2 <= q[5] && hi2 >= q[4])) continue;
      int k = ((const int *)q)[6];
      u32 bit = i < k ? (u32)i * n + k : (u32)k * n + i;
      u32 wi = bit >> 5, mk = 1u << (bit & 31);
      if (okBits[wi] & mk) candBits[wi] |= mk;
    }
  }
  int cnt = 0;
  for (u32 wi = 0; wi < words; wi++) {
    u32 word = candBits[wi];
    while (word) {
      u32 t = word & (0u - word);
      word ^= t;
      u32 bit = wi * 32 + (31 - __builtin_clz(t));
      int i = (int)(bit / (u32)n), k = (int)(bit - (u32)i * n);
      if (prefilter) {
        float lim = ((sphR[i] + sphR[k]) + fmin32(thr[i], thr[k])) + slack;
        float dx = org[k * 3] - org[i * 3], dy = org[k * 3 + 1] - org[i * 3 + 1], dz = org[k * 3 + 2] - org[i * 3 + 2];
        if (dx * dx + dy * dy + dz * dz >= lim * lim) continue;
      }
      const float *bi = bb + i * 6, *bk = bb + k * 6;
      if (!(bi[0] <= bk[3] && bi[3] >= bk[0] && bi[1] <= bk[4] && bi[4] >= bk[1] && bi[2] <= bk[5] && bi[5] >= bk[2])) continue;
      outPairs[cnt * 2] = i;
      outPairs[cnt * 2 + 1] = k;
      cnt++;
    }
  }
  return cnt;
}

// ---------------------------------------------------------------------------
// Joint.Prepare for the default flag set (BulletLimitRowGating, AngularMixedAxes, BulletAngleConvention,
// LinearLeverMode=1, SpringAsMotorRow, no warm start / split / FreezeCrossAxes) — a transcription of
// Joint.prepareFast in constraints.ts, which itself mirrors the C# Prepare. atan2 / asin come from JS
// (imports) so the transcendental results are exactly the ones of the TypeScript path.
// joint (JNT words): qA[4] oA[3] qB[4] oB[3] linLo[3] linHi[3] angLo[3] angHi[3] springLin[3] springAng[3]
//                    beta bodyA(int) bodyB(int)                                                    = 35
// params: invDt gate maxCorr lockedBound angBetaScale errDeadband biasDeadband
// ---------------------------------------------------------------------------
#define JNT 35
__attribute__((import_module("m"), import_name("atan2"))) double js_atan2(double y, double x);
__attribute__((import_module("m"), import_name("asin"))) double js_asin(double x);

static inline float fsqrt(float x) { return __builtin_sqrtf(x); }

static void qMulNormalized(const float *a, const float *b, float *out) {
  float x = a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1];
  float y = a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0];
  float z = a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3];
  float w = a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2];
  float len = fsqrt(x * x + y * y + z * z + w * w);
  if (len > 0.0f) {
    float i = 1.0f / len;
    out[0] = x * i; out[1] = y * i; out[2] = z * i; out[3] = w * i;
  } else {
    out[0] = 0.0f; out[1] = 0.0f; out[2] = 0.0f; out[3] = 1.0f;
  }
}

static void qRotate(const float *q, float ix, float iy, float iz, float *out) {
  float qx = q[0], qy = q[1], qz = q[2], qw = q[3];
  float t = 2.0f / (qx * qx + qy * qy + qz * qz + qw * qw);
  out[0] = (1.0f - t * (qy * qy + qz * qz)) * ix + t * (qx * qy - qw * qz) * iy + t * (qx * qz + qw * qy) * iz;
  out[1] = t * (qx * qy + qw * qz) * ix + (1.0f - t * (qx * qx + qz * qz)) * iy + t * (qy * qz - qw * qx) * iz;
  out[2] = t * (qx * qz - qw * qy) * ix + t * (qy * qz + qw * qx) * iy + (1.0f - t * (qx * qx + qy * qy)) * iz;
}

static void m3FromQuat(const float *q, float *o) {
  float x = q[0], y = q[1], z = q[2], w = q[3];
  float xx = x * x, yy = y * y, zz = z * z, xy = x * y, xz = x * z, yz = y * z, wx = w * x, wy = w * y, wz = w * z;
  o[0] = 1.0f - 2.0f * (yy + zz); o[1] = 2.0f * (xy - wz); o[2] = 2.0f * (xz + wy);
  o[3] = 2.0f * (xy + wz); o[4] = 1.0f - 2.0f * (xx + zz); o[5] = 2.0f * (yz - wx);
  o[6] = 2.0f * (xz - wy); o[7] = 2.0f * (yz + wx); o[8] = 1.0f - 2.0f * (xx + yy);
}

static void m3FromQuatBullet(const float *q, float *o) {
  float d = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
  float sc = 2.0f / d;
  float xs = q[0] * sc, ys = q[1] * sc, zs = q[2] * sc;
  float wx = q[3] * xs, wy = q[3] * ys, wz = q[3] * zs;
  float xx = q[0] * xs, xy = q[0] * ys, xz = q[0] * zs;
  float yy = q[1] * ys, yz = q[1] * zs, zz = q[2] * zs;
  o[0] = 1.0f - (yy + zz); o[1] = xy - wz; o[2] = xz + wy;
  o[3] = xy + wz; o[4] = 1.0f - (xx + zz); o[5] = yz - wx;
  o[6] = xz - wy; o[7] = yz + wx; o[8] = 1.0f - (xx + yy);
}

static inline float m3Row(const float *m, int o, float x, float y, float z) { return m[o] * x + m[o + 1] * y + m[o + 2] * z; }

static void m3Mul(const float *a, const float *b, float *out) {
  for (int i = 0; i < 9; i += 3) {
    float a0 = a[i], a1 = a[i + 1], a2 = a[i + 2];
    out[i] = a0 * b[0] + a1 * b[3] + a2 * b[6];
    out[i + 1] = a0 * b[1] + a1 * b[4] + a2 * b[7];
    out[i + 2] = a0 * b[2] + a1 * b[5] + a2 * b[8];
  }
}

static inline float cofac(float a, float b, float c, float d) { return a * b - c * d; }

static void m3BulletInverse(const float *m, float *out) {
  float m00 = m[0], m01 = m[1], m02 = m[2], m10 = m[3], m11 = m[4], m12 = m[5], m20 = m[6], m21 = m[7], m22 = m[8];
  float cx = cofac(m11, m22, m12, m21), cy = cofac(m12, m20, m10, m22), cz = cofac(m10, m21, m11, m20);
  float det = m00 * cx + m01 * cy + m02 * cz;
  float sc = 1.0f / det;
  out[0] = cx * sc; out[1] = cofac(m02, m21, m01, m22) * sc; out[2] = cofac(m01, m12, m02, m11) * sc;
  out[3] = cy * sc; out[4] = cofac(m00, m22, m02, m20) * sc; out[5] = cofac(m02, m10, m00, m12) * sc;
  out[6] = cz * sc; out[7] = cofac(m01, m20, m00, m21) * sc; out[8] = cofac(m00, m11, m01, m10) * sc;
}

#define HALF_PI_F 1.57079632679489661923f

// ToEulerXYZBullet(m, bulletElem = true)
static void eulerBullet(const float *m, float *out) {
  float e02 = m[6], e12 = m[7], e22 = m[8], e01 = m[3], e00 = m[0], e10 = m[1], e11 = m[4];
  if (e02 < 1.0f) {
    if (e02 > -1.0f) {
      out[0] = (float)js_atan2(-e12, e22);
      out[1] = (float)js_asin(e02);
      out[2] = (float)js_atan2(-e01, e00);
    } else {
      out[0] = -(float)js_atan2(e10, e11);
      out[1] = -HALF_PI_F;
      out[2] = 0.0f;
    }
  } else {
    out[0] = (float)js_atan2(e10, e11);
    out[1] = HALF_PI_F;
    out[2] = 0.0f;
  }
}

static inline void normalize3(float x, float y, float z, float *out) {
  float len = fsqrt(x * x + y * y + z * z);
  if (len > 0.0f) {
    float i = 1.0f / len;
    out[0] = x * i; out[1] = y * i; out[2] = z * i;
  } else {
    out[0] = 0.0f; out[1] = 0.0f; out[2] = 0.0f;
  }
}

static inline int testLimitValue(float v, float lo, float hi) {
  if (lo > hi) return 0;
  if (v < lo) return 2;
  if (v > hi) return 1;
  return 0;
}

static inline float clampToLimit(float v, float lo, float hi) {
  if (lo > hi) return v;
  return fmax32(lo, fmin32(hi, v));
}

static float motorFactor(float pos, float lowLim, float uppLim, float vel, float timeFact) {
  if (lowLim > uppLim) return 1.0f;
  if (lowLim == uppLim) return 0.0f;
  float limFact = 1.0f;
  float deltaMax = timeFact != 0.0f ? vel / timeFact : 0.0f;
  if (deltaMax < 0.0f) {
    if (pos >= lowLim && pos < (lowLim - deltaMax)) limFact = (lowLim - pos) / deltaMax;
    else if (pos < lowLim) limFact = 0.0f;
    else limFact = 1.0f;
  } else if (deltaMax > 0.0f) {
    if (pos <= uppLim && pos > (uppLim - deltaMax)) limFact = (uppLim - pos) / deltaMax;
    else if (pos > uppLim) limFact = 0.0f;
    else limFact = 1.0f;
  } else limFact = 0.0f;
  return limFact;
}

static inline float inertiaDot(const float *s, float x, float y, float z) {
  float ix = s[6] * x + s[7] * y + s[8] * z;
  float iy = s[9] * x + s[10] * y + s[11] * z;
  float iz = s[12] * x + s[13] * y + s[14] * z;
  return x * ix + y * iy + z * iz;
}

typedef struct {
  float *rows;
  int count;
  int bodyA, bodyB;
  float maxCorr;
} RowOut;

static void pushRow(RowOut *ro, float ax, float ay, float az, float rax, float ray, float raz, float rbx, float rby,
                    float rbz, int angular, float lo, float hi, float tv, float em) {
  float *r = ro->rows + ro->count * ROW;
  r[0] = ax; r[1] = ay; r[2] = az; r[3] = rax; r[4] = ray; r[5] = raz; r[6] = rbx; r[7] = rby; r[8] = rbz;
  r[9] = lo; r[10] = hi; r[11] = tv; r[12] = em; r[13] = 0.0f; r[14] = angular ? 1.0f : 0.0f;
  ((int *)r)[15] = ro->bodyA;
  ((int *)r)[16] = ro->bodyB;
  ro->count++;
}

static void linearRow(RowOut *ro, const float *sa, const float *sb, float ax, float ay, float az, float rax, float ray,
                      float raz, float rbx, float rby, float rbz, float tv, float lo, float hi) {
  float cAx = ray * az - raz * ay, cAy = raz * ax - rax * az, cAz = rax * ay - ray * ax;
  float cBx = rby * az - rbz * ay, cBy = rbz * ax - rbx * az, cBz = rbx * ay - rby * ax;
  float k = sa[15] + sb[15] + inertiaDot(sa, cAx, cAy, cAz) + inertiaDot(sb, cBx, cBy, cBz);
  pushRow(ro, ax, ay, az, rax, ray, raz, rbx, rby, rbz, 0, lo, hi, tv, k > 0.0f ? 1.0f / k : 0.0f);
}

static void angularRow(RowOut *ro, const float *sa, const float *sb, float ax, float ay, float az, float tv, float lo, float hi) {
  float k = inertiaDot(sa, ax, ay, az) + inertiaDot(sb, ax, ay, az);
  pushRow(ro, ax, ay, az, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1, lo, hi, tv, k > 0.0f ? 1.0f / k : 0.0f);
}

static inline float clampCorr(float m, float v) { return fmax32(-m, fmin32(m, v)); }

typedef struct {
  float aAx, aAy, aAz, aBx, aBy, aBz;
  float delta;
  int deltaReady;
} Anchors;

static float anchorDelta(Anchors *an) {
  if (!an->deltaReady) {
    float ex = an->aBx - an->aAx, ey = an->aBy - an->aAy, ez = an->aBz - an->aAz;
    an->delta = fsqrt(ex * ex + ey * ey + ez * ez);
    an->deltaReady = 1;
  }
  return an->delta;
}

static void motorRow(RowOut *ro, const float *sa, const float *sb, int angular, float ax, float ay, float az, float cur,
                     float lo, float hi, float k, float invDt, float beta, int iters, float gate, float rAx, float rAy,
                     float rAz, float rBx, float rBy, float rBz, float pAx, float pAy, float pAz, Anchors *an) {
  float fps = invDt;
  if (fps <= 0.0f) return;
  float eq = clampToLimit(0.0f, lo, hi);
  float delta = cur - eq;
  float velFactor = fps * 1.0f / (float)iters;
  float force = delta * k;
  float targetVel = -velFactor * force;
  float maxMotorForce = __builtin_fabsf(force) / fps;
  float motFact = motorFactor(cur, lo, hi, targetVel, fps * beta);
  targetVel = targetVel * motFact;
  if (angular) {
    angularRow(ro, sa, sb, ax, ay, az, targetVel, -maxMotorForce, maxMotorForce);
    return;
  }
  float armAx = pAx, armAy = pAy, armAz = pAz;
  if (gate > 0.0f) {
    float gscale = fsqrt(rAx * rAx + rAy * rAy + rAz * rAz) + fsqrt(rBx * rBx + rBy * rBy + rBz * rBz);
    if (anchorDelta(an) > gate * gscale) { armAx = rAx; armAy = rAy; armAz = rAz; }
  }
  linearRow(ro, sa, sb, ax, ay, az, armAx, armAy, armAz, rBx, rBy, rBz, targetVel, -maxMotorForce, maxMotorForce);
}

// Prepares all joints; writes rows (solver layout) and the row count of each joint. Returns the total.
__attribute__((export_name("prepare"))) int prepare(int nJoints, const float *joints, float *bodies, const float *params,
                                                    int iters, float *rows, int *rowCounts) {
  float invDt = params[0], gate = params[1], maxCorr = params[2], BND = params[3], angBetaScale = params[4];
  float errDeadband = params[5], biasDeadband = params[6];
  RowOut ro;
  ro.rows = rows;
  ro.count = 0;
  ro.maxCorr = maxCorr;
  float qa[4], qb[4], tmp[3], BA[9], BB9[9], BMA[9], BMB[9], T[9], BAb[9], BBb[9], INV[9], eu[3], mix[9];
  for (int j = 0; j < nJoints; j++) {
    const float *J = joints + j * JNT;
    int start = ro.count;
    int ia = ((const int *)J)[33], ib = ((const int *)J)[34];
    float *sa = bodies + ia * BODY, *sb = bodies + ib * BODY;
    ro.bodyA = ia;
    ro.bodyB = ib;
    const float *rotA = sa + 19, *rotB = sb + 19;
    const float *oa = sa + 16, *ob = sb + 16;
    // worldA / worldB
    qMulNormalized(rotA, J + 0, qa);
    qRotate(rotA, J[4], J[5], J[6], tmp);
    Anchors an;
    an.aAx = tmp[0] + oa[0]; an.aAy = tmp[1] + oa[1]; an.aAz = tmp[2] + oa[2];
    qMulNormalized(rotB, J + 7, qb);
    qRotate(rotB, J[11], J[12], J[13], tmp);
    an.aBx = tmp[0] + ob[0]; an.aBy = tmp[1] + ob[1]; an.aBz = tmp[2] + ob[2];
    an.deltaReady = 0;
    float rAx = an.aAx - oa[0], rAy = an.aAy - oa[1], rAz = an.aAz - oa[2];
    float rBx = an.aBx - ob[0], rBy = an.aBy - ob[1], rBz = an.aBz - ob[2];
    m3FromQuat(qa, BA);
    // Bullet chain
    m3FromQuatBullet(rotA, BMA);
    m3FromQuatBullet(rotB, BMB);
    m3FromQuatBullet(J + 0, T);
    m3Mul(BMA, T, BAb);
    m3FromQuatBullet(J + 7, T);
    m3Mul(BMB, T, BBb);
    float oAx = m3Row(BMA, 0, J[4], J[5], J[6]) + oa[0], oAy = m3Row(BMA, 3, J[4], J[5], J[6]) + oa[1], oAz = m3Row(BMA, 6, J[4], J[5], J[6]) + oa[2];
    float oBx = m3Row(BMB, 0, J[11], J[12], J[13]) + ob[0], oBy = m3Row(BMB, 3, J[11], J[12], J[13]) + ob[1], oBz = m3Row(BMB, 6, J[11], J[12], J[13]) + ob[2];
    m3BulletInverse(BAb, INV);
    float dx = oBx - oAx, dy = oBy - oAy, dz = oBz - oAz;
    float ld[3] = { m3Row(INV, 0, dx, dy, dz), m3Row(INV, 3, dx, dy, dz), m3Row(INV, 6, dx, dy, dz) };
    m3Mul(INV, BBb, T);
    eulerBullet(T, eu);

    float beta = J[32];
    float pAx = an.aBx - oa[0], pAy = an.aBy - oa[1], pAz = an.aBz - oa[2];
    float lenRA = -1.0f, lenRB = -1.0f;
    int lenReady = 0;
    for (int i = 0; i < 3; i++) {
      float ax = BA[i], ay = BA[3 + i], az = BA[6 + i];
      float lo = J[14 + i], hi = J[17 + i], cur = ld[i];
      if (lo > hi || (lo == hi && testLimitValue(cur, lo, hi) == 0) || (lo != hi && !(cur < lo) && !(cur > hi))) {
        float k = J[26 + i];
        if (!(k <= 0.0f))
          motorRow(&ro, sa, sb, 0, ax, ay, az, cur, lo, hi, k, invDt, beta, iters, gate, rAx, rAy, rAz, rBx, rBy, rBz, pAx, pAy, pAz, &an);
        continue;
      }
      float err, lower, upper;
      if (lo == hi) { err = lo - cur; lower = -BND; upper = BND; }
      else if (cur < lo) { err = lo - cur; lower = 0.0f; upper = BND; }
      else { err = hi - cur; lower = -BND; upper = 0.0f; }
      if (errDeadband > 0.0f && __builtin_fabsf(err) < errDeadband) continue;
      float armAx = pAx, armAy = pAy, armAz = pAz;
      if (gate > 0.0f) {
        if (!lenReady) {
          lenRA = fsqrt(rAx * rAx + rAy * rAy + rAz * rAz);
          lenRB = fsqrt(rBx * rBx + rBy * rBy + rBz * rBz);
          lenReady = 1;
        }
        float scale = lenRA + lenRB;
        float ratio = scale > 1e-9f ? anchorDelta(&an) / scale : 0.0f;
        if (ratio > gate) { armAx = rAx; armAy = rAy; armAz = rAz; }
      }
      float linTarget = clampCorr(maxCorr, err * beta * invDt);
      if (biasDeadband > 0.0f && __builtin_fabsf(err) < biasDeadband) linTarget = 0.0f;
      linearRow(&ro, sa, sb, ax, ay, az, armAx, armAy, armAz, rBx, rBy, rBz, linTarget, lower, upper);
    }
    // mixed axes
    m3FromQuat(qb, BB9);
    float b0x = BB9[0], b0y = BB9[3], b0z = BB9[6];
    float a2x = BA[2], a2y = BA[5], a2z = BA[8];
    float m1x = a2y * b0z - a2z * b0y, m1y = a2z * b0x - a2x * b0z, m1z = a2x * b0y - a2y * b0x;
    float m0x = m1y * a2z - m1z * a2y, m0y = m1z * a2x - m1x * a2z, m0z = m1x * a2y - m1y * a2x;
    float m2x = b0y * m1z - b0z * m1y, m2y = b0z * m1x - b0x * m1z, m2z = b0x * m1y - b0y * m1x;
    normalize3(m0x, m0y, m0z, mix);
    normalize3(m1x, m1y, m1z, mix + 3);
    normalize3(m2x, m2y, m2z, mix + 6);
    float angBeta = beta * angBetaScale;
    for (int i = 0; i < 3; i++) {
      float ax = -mix[i * 3], ay = -mix[i * 3 + 1], az = -mix[i * 3 + 2];
      float lo = J[20 + i], hi = J[23 + i], cur = eu[i];
      if (lo > hi || (lo == hi && testLimitValue(cur, lo, hi) == 0) || (lo != hi && !(cur < lo) && !(cur > hi))) {
        float k = J[29 + i];
        if (!(k <= 0.0f))
          motorRow(&ro, sa, sb, 1, ax, ay, az, cur, lo, hi, k, invDt, beta, iters, gate, 0, 0, 0, 0, 0, 0, 0, 0, 0, &an);
        continue;
      }
      float err, lower, upper;
      if (lo == hi) { err = lo - cur; lower = -1e18f; upper = 1e18f; }
      else if (cur < lo) { err = lo - cur; lower = 0.0f; upper = 1e18f; }
      else { err = hi - cur; lower = -1e18f; upper = 0.0f; }
      if (errDeadband > 0.0f && __builtin_fabsf(err) < errDeadband) continue;
      angularRow(&ro, sa, sb, ax, ay, az, clampCorr(maxCorr, err * angBeta * invDt), lower, upper);
    }
    rowCounts[j] = ro.count - start;
  }
  return ro.count;
}

extern unsigned char __heap_base;
__attribute__((export_name("heapBase"))) u32 heapBase(void) { return (u32)&__heap_base; }
