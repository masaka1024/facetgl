// ===========================================================================
// MMD physics engine (TypeScript port) – core math
//
// Port of the C# engine's MathTypes.cs + Transform.cs (namespace BulletPhysics,
// https://github.com/masaka1024/mmd2gltf-cs-physics).
//
// ★float32 emulation: the C# engine computes in `float` and some decisions depend on the
//   exact float32 rounding (for example whether a locked axis error is exactly 0, which
//   decides whether a constraint row is created at all). JavaScript numbers are float64,
//   so every arithmetic result is rounded back to float32 with Math.fround (`f`), one
//   operation at a time and in the same order as the C# expression. For + - * / and sqrt
//   this reproduces IEEE float32 exactly (double rounding is innocuous for these).
//   Transcendental functions are evaluated in double and rounded, like `(float)Math.Sin(x)`.
//
// Values are immutable: C# structs are copied by value, so every operation returns a new
// object. Never mutate a Vec3 / Quat / Matrix3x3 / RigidTransform after construction.
// ===========================================================================

export const f = Math.fround;

/** float.MaxValue */
export const FLT_MAX = 3.4028234663852886e38;

export class Vec3 {
  constructor(
    readonly x: number,
    readonly y: number,
    readonly z: number,
  ) {}

  /** From arbitrary numbers (rounds each component to float32). */
  static of(x: number, y: number, z: number) {
    return new Vec3(f(x), f(y), f(z));
  }

  /** new Vec3(v) — all three components the same. */
  static splat(v: number) {
    const s = f(v);
    return new Vec3(s, s, s);
  }

  static readonly Zero = new Vec3(0, 0, 0);
  static readonly One = new Vec3(1, 1, 1);
  static readonly XAxis = new Vec3(1, 0, 0);
  static readonly YAxis = new Vec3(0, 1, 0);
  static readonly ZAxis = new Vec3(0, 0, 1);

  /** this[index] */
  get(index: number): number {
    switch (index) {
      case 0: return this.x;
      case 1: return this.y;
      case 2: return this.z;
      default: throw new RangeError('index');
    }
  }

  /** Copy with this[index] = value (C# `v[i] = value` on a local copy). */
  with(index: number, value: number): Vec3 {
    switch (index) {
      case 0: return new Vec3(value, this.y, this.z);
      case 1: return new Vec3(this.x, value, this.z);
      case 2: return new Vec3(this.x, this.y, value);
      default: throw new RangeError('index');
    }
  }

  get lengthSquared() {
    return f(f(f(this.x * this.x) + f(this.y * this.y)) + f(this.z * this.z));
  }

  get length() {
    return f(Math.sqrt(this.lengthSquared));
  }

  get normalized(): Vec3 {
    const len = this.length;
    return len > 0 ? this.div(len) : Vec3.Zero;
  }

  add(b: Vec3) {
    return new Vec3(f(this.x + b.x), f(this.y + b.y), f(this.z + b.z));
  }

  sub(b: Vec3) {
    return new Vec3(f(this.x - b.x), f(this.y - b.y), f(this.z - b.z));
  }

  neg() {
    return new Vec3(-this.x, -this.y, -this.z);
  }

  /** a * s (also s * a) */
  mul(s: number) {
    return new Vec3(f(this.x * s), f(this.y * s), f(this.z * s));
  }

  /** a / s = a * (1 / s) */
  div(s: number) {
    return this.mul(f(1 / s));
  }

  static cross(a: Vec3, b: Vec3) {
    return new Vec3(
      f(f(a.y * b.z) - f(a.z * b.y)),
      f(f(a.z * b.x) - f(a.x * b.z)),
      f(f(a.x * b.y) - f(a.y * b.x)),
    );
  }

  dot(b: Vec3) {
    return f(f(f(this.x * b.x) + f(this.y * b.y)) + f(this.z * b.z));
  }

  angle(b: Vec3) {
    const d = f(this.dot(b) / f(this.length * b.length));
    return f(Math.acos(clamp(d, -1, 1)));
  }

  /** C# Equals / == (component-wise tolerance 1e-6) */
  equals(o: Vec3) {
    return Math.abs(f(this.x - o.x)) < EPS6 && Math.abs(f(this.y - o.y)) < EPS6 && Math.abs(f(this.z - o.z)) < EPS6;
  }

  toString() {
    return `(${this.x.toFixed(4)}, ${this.y.toFixed(4)}, ${this.z.toFixed(4)})`;
  }
}

const EPS6 = f(1e-6);

/** Math.Clamp for floats (inputs are already float32). */
export function clamp(v: number, lo: number, hi: number) {
  return v < lo ? lo : v > hi ? hi : v;
}

export class Quat {
  constructor(
    readonly x: number,
    readonly y: number,
    readonly z: number,
    readonly w: number,
  ) {}

  static readonly Identity = new Quat(0, 0, 0, 1);
  /** default(Quat) — all zero (w = 0). */
  static readonly Zero = new Quat(0, 0, 0, 0);

  static fromAxisAngle(axis: Vec3, angleRad: number) {
    const half = f(angleRad * 0.5);
    const sin = f(Math.sin(half));
    const n = axis.normalized;
    return new Quat(f(n.x * sin), f(n.y * sin), f(n.z * sin), f(Math.cos(half)));
  }

  /** MMD / PMX Euler (radians) → quaternion, R = Ry * Rx * Rz (YXZ order). */
  static fromEulerYxz(rx: number, ry: number, rz: number) {
    const cx = f(Math.cos(f(rx * 0.5))), sx = f(Math.sin(f(rx * 0.5)));
    const cy = f(Math.cos(f(ry * 0.5))), sy = f(Math.sin(f(ry * 0.5)));
    const cz = f(Math.cos(f(rz * 0.5))), sz = f(Math.sin(f(rz * 0.5)));
    return new Quat(
      f(f(f(cy * sx) * cz) + f(f(sy * cx) * sz)),
      f(f(f(sy * cx) * cz) - f(f(cy * sx) * sz)),
      f(f(f(cy * cx) * sz) - f(f(sy * sx) * cz)),
      f(f(f(cy * cx) * cz) + f(f(sy * sx) * sz)),
    );
  }

  /** ZYX order (NOT the PMX order; use fromEulerYxz for PMX data). */
  static fromEuler(rx: number, ry: number, rz: number) {
    const cx = f(Math.cos(f(rx * 0.5))), sx = f(Math.sin(f(rx * 0.5)));
    const cy = f(Math.cos(f(ry * 0.5))), sy = f(Math.sin(f(ry * 0.5)));
    const cz = f(Math.cos(f(rz * 0.5))), sz = f(Math.sin(f(rz * 0.5)));
    return new Quat(
      f(f(f(sx * cy) * cz) - f(f(cx * sy) * sz)),
      f(f(f(cx * sy) * cz) + f(f(sx * cy) * sz)),
      f(f(f(cx * cy) * sz) - f(f(sx * sy) * cz)),
      f(f(f(cx * cy) * cz) + f(f(sx * sy) * sz)),
    );
  }

  /** a * b (Hamilton product) */
  mul(b: Quat) {
    const a = this;
    return new Quat(
      f(f(f(f(a.w * b.x) + f(a.x * b.w)) + f(a.y * b.z)) - f(a.z * b.y)),
      f(f(f(f(a.w * b.y) - f(a.x * b.z)) + f(a.y * b.w)) + f(a.z * b.x)),
      f(f(f(f(a.w * b.z) + f(a.x * b.y)) - f(a.y * b.x)) + f(a.z * b.w)),
      f(f(f(f(a.w * b.w) - f(a.x * b.x)) - f(a.y * b.y)) - f(a.z * b.z)),
    );
  }

  /** q * v (rotates v; works for non-unit q too, like the C# operator) */
  rotate(v: Vec3) {
    const qx = this.x, qy = this.y, qz = this.z, qw = this.w;
    const ix = v.x, iy = v.y, iz = v.z;
    const t = f(2 / f(f(f(f(qx * qx) + f(qy * qy)) + f(qz * qz)) + f(qw * qw)));

    const ox = f(
      f(
        f(f(1 - f(t * f(f(qy * qy) + f(qz * qz)))) * ix) +
        f(f(t * f(f(qx * qy) - f(qw * qz))) * iy),
      ) + f(f(t * f(f(qx * qz) + f(qw * qy))) * iz),
    );
    const oy = f(
      f(
        f(f(t * f(f(qx * qy) + f(qw * qz))) * ix) +
        f(f(1 - f(t * f(f(qx * qx) + f(qz * qz)))) * iy),
      ) + f(f(t * f(f(qy * qz) - f(qw * qx))) * iz),
    );
    const oz = f(
      f(
        f(f(t * f(f(qx * qz) - f(qw * qy))) * ix) +
        f(f(t * f(f(qy * qz) + f(qw * qx))) * iy),
      ) + f(f(1 - f(t * f(f(qx * qx) + f(qy * qy)))) * iz),
    );
    return new Vec3(ox, oy, oz);
  }

  static fromMatrix(m: Matrix4x4) {
    const trace = f(f(m.m00 + m.m11) + m.m22);
    if (trace > 0) {
      const s = f(f(Math.sqrt(f(trace + 1))) * 2);
      return new Quat(f(f(m.m21 - m.m12) / s), f(f(m.m02 - m.m20) / s), f(f(m.m10 - m.m01) / s), f(0.25 * s));
    }
    if (m.m00 > m.m11 && m.m00 > m.m22) {
      const s = f(f(Math.sqrt(f(f(f(1 + m.m00) - m.m11) - m.m22))) * 2);
      return new Quat(f(0.25 * s), f(f(m.m01 + m.m10) / s), f(f(m.m02 + m.m20) / s), f(f(m.m21 - m.m12) / s));
    }
    if (m.m11 > m.m22) {
      const s = f(f(Math.sqrt(f(f(f(1 + m.m11) - m.m00) - m.m22))) * 2);
      return new Quat(f(f(m.m01 + m.m10) / s), f(0.25 * s), f(f(m.m12 + m.m21) / s), f(f(m.m02 - m.m20) / s));
    }
    const s = f(f(Math.sqrt(f(f(f(1 + m.m22) - m.m00) - m.m11))) * 2);
    return new Quat(f(f(m.m02 + m.m20) / s), f(f(m.m12 + m.m21) / s), f(0.25 * s), f(f(m.m10 - m.m01) / s));
  }

  get normalized(): Quat {
    const len = f(Math.sqrt(f(f(f(f(this.x * this.x) + f(this.y * this.y)) + f(this.z * this.z)) + f(this.w * this.w))));
    return len > 0 ? this.div(len) : Quat.Identity;
  }

  div(s: number) {
    const i = f(1 / s);
    return new Quat(f(this.x * i), f(this.y * i), f(this.z * i), f(this.w * i));
  }

  mulScalar(s: number) {
    return new Quat(f(this.x * s), f(this.y * s), f(this.z * s), f(this.w * s));
  }

  static slerp(a: Quat, b: Quat, t: number) {
    let cos = f(f(f(f(a.x * b.x) + f(a.y * b.y)) + f(a.z * b.z)) + f(a.w * b.w));
    let wrap = false;
    if (cos < 0) {
      cos = -cos;
      wrap = true;
    }
    let s0: number, s1: number;
    if (f(1 - cos) > SLERP_EPS) {
      const omega = f(Math.acos(cos));
      const sin = f(Math.sin(omega));
      s0 = f(f(Math.sin(f(f(1 - t) * omega))) / sin);
      s1 = f(f(Math.sin(f(t * omega))) / sin);
    } else {
      s0 = f(1 - t);
      s1 = t;
    }
    if (wrap) s1 = -s1;
    return new Quat(
      f(f(s0 * a.x) + f(s1 * b.x)),
      f(f(s0 * a.y) + f(s1 * b.y)),
      f(f(s0 * a.z) + f(s1 * b.z)),
      f(f(s0 * a.w) + f(s1 * b.w)),
    );
  }

  conjugated() {
    return new Quat(-this.x, -this.y, -this.z, this.w);
  }

  equals(o: Quat) {
    return (
      Math.abs(f(this.x - o.x)) < EPS6 && Math.abs(f(this.y - o.y)) < EPS6 &&
      Math.abs(f(this.z - o.z)) < EPS6 && Math.abs(f(this.w - o.w)) < EPS6
    );
  }

  toString() {
    return `q(${this.x.toFixed(4)}, ${this.y.toFixed(4)}, ${this.z.toFixed(4)}, ${this.w.toFixed(4)})`;
  }
}

const SLERP_EPS = f(0.001);

/** 4x4 matrix with the C# engine's (transposed) layout. */
export class Matrix4x4 {
  constructor(
    readonly m00: number, readonly m01: number, readonly m02: number, readonly m03: number,
    readonly m10: number, readonly m11: number, readonly m12: number, readonly m13: number,
    readonly m20: number, readonly m21: number, readonly m22: number, readonly m23: number,
    readonly m30: number, readonly m31: number, readonly m32: number, readonly m33: number,
  ) {}

  static readonly Identity = new Matrix4x4(1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1);

  static translation(t: Vec3) {
    return new Matrix4x4(1, 0, 0, t.x, 0, 1, 0, t.y, 0, 0, 1, t.z, 0, 0, 0, 1);
  }

  static rotation(q: Quat) {
    const xx = f(q.x * q.x), yy = f(q.y * q.y), zz = f(q.z * q.z);
    const xy = f(q.x * q.y), xz = f(q.x * q.z), yz = f(q.y * q.z);
    const wx = f(q.w * q.x), wy = f(q.w * q.y), wz = f(q.w * q.z);
    return new Matrix4x4(
      f(1 - f(2 * f(yy + zz))), f(2 * f(xy + wz)), f(2 * f(xz - wy)), 0,
      f(2 * f(xy - wz)), f(1 - f(2 * f(xx + zz))), f(2 * f(yz + wx)), 0,
      f(2 * f(xz + wy)), f(2 * f(yz - wx)), f(1 - f(2 * f(xx + yy))), 0,
      0, 0, 0, 1,
    );
  }

  static scale(s: Vec3) {
    return new Matrix4x4(s.x, 0, 0, 0, 0, s.y, 0, 0, 0, 0, s.z, 0, 0, 0, 0, 1);
  }

  static multiply(a: Matrix4x4, b: Matrix4x4) {
    const d = (a0: number, a1: number, a2: number, a3: number, b0: number, b1: number, b2: number, b3: number) =>
      f(f(f(f(a0 * b0) + f(a1 * b1)) + f(a2 * b2)) + f(a3 * b3));
    return new Matrix4x4(
      d(a.m00, a.m01, a.m02, a.m03, b.m00, b.m10, b.m20, b.m30),
      d(a.m00, a.m01, a.m02, a.m03, b.m01, b.m11, b.m21, b.m31),
      d(a.m00, a.m01, a.m02, a.m03, b.m02, b.m12, b.m22, b.m32),
      d(a.m00, a.m01, a.m02, a.m03, b.m03, b.m13, b.m23, b.m33),
      d(a.m10, a.m11, a.m12, a.m13, b.m00, b.m10, b.m20, b.m30),
      d(a.m10, a.m11, a.m12, a.m13, b.m01, b.m11, b.m21, b.m31),
      d(a.m10, a.m11, a.m12, a.m13, b.m02, b.m12, b.m22, b.m32),
      d(a.m10, a.m11, a.m12, a.m13, b.m03, b.m13, b.m23, b.m33),
      d(a.m20, a.m21, a.m22, a.m23, b.m00, b.m10, b.m20, b.m30),
      d(a.m20, a.m21, a.m22, a.m23, b.m01, b.m11, b.m21, b.m31),
      d(a.m20, a.m21, a.m22, a.m23, b.m02, b.m12, b.m22, b.m32),
      d(a.m20, a.m21, a.m22, a.m23, b.m03, b.m13, b.m23, b.m33),
      d(a.m30, a.m31, a.m32, a.m33, b.m00, b.m10, b.m20, b.m30),
      d(a.m30, a.m31, a.m32, a.m33, b.m01, b.m11, b.m21, b.m31),
      d(a.m30, a.m31, a.m32, a.m33, b.m02, b.m12, b.m22, b.m32),
      d(a.m30, a.m31, a.m32, a.m33, b.m03, b.m13, b.m23, b.m33),
    );
  }

  transformPoint(p: Vec3) {
    const w = f(f(f(f(this.m03 * p.x) + f(this.m13 * p.y)) + f(this.m23 * p.z)) + this.m33);
    return new Vec3(
      f(f(f(f(f(this.m00 * p.x) + f(this.m10 * p.y)) + f(this.m20 * p.z)) + this.m30) / w),
      f(f(f(f(f(this.m01 * p.x) + f(this.m11 * p.y)) + f(this.m21 * p.z)) + this.m31) / w),
      f(f(f(f(f(this.m02 * p.x) + f(this.m12 * p.y)) + f(this.m22 * p.z)) + this.m32) / w),
    );
  }

  transformVector(v: Vec3) {
    return new Vec3(
      f(f(f(this.m00 * v.x) + f(this.m10 * v.y)) + f(this.m20 * v.z)),
      f(f(f(this.m01 * v.x) + f(this.m11 * v.y)) + f(this.m21 * v.z)),
      f(f(f(this.m02 * v.x) + f(this.m12 * v.y)) + f(this.m22 * v.z)),
    );
  }

  /** Copy with the translation column set (C# `m.m03 = ...; m.m13 = ...; m.m23 = ...`). */
  withTranslation(t: Vec3) {
    return new Matrix4x4(
      this.m00, this.m01, this.m02, t.x,
      this.m10, this.m11, this.m12, t.y,
      this.m20, this.m21, this.m22, t.z,
      this.m30, this.m31, this.m32, this.m33,
    );
  }
}

/** Infinite plane n·x + d = 0. */
export class Plane {
  readonly normal: Vec3;
  constructor(normal: Vec3, readonly distance: number) {
    this.normal = normal.normalized;
  }
  signedDistance(p: Vec3) {
    return f(this.normal.dot(p) + this.distance);
  }
}

/** Axis-aligned bounding box. Mutable like the C# struct: use clone() where C# copies by value. */
export class Aabb {
  constructor(
    public min: Vec3,
    public max: Vec3,
  ) {}

  clone() {
    return new Aabb(this.min, this.max);
  }

  contains(p: Vec3) {
    return p.x >= this.min.x && p.x <= this.max.x && p.y >= this.min.y && p.y <= this.max.y && p.z >= this.min.z && p.z <= this.max.z;
  }

  intersects(o: Aabb) {
    return (
      this.min.x <= o.max.x && this.max.x >= o.min.x &&
      this.min.y <= o.max.y && this.max.y >= o.min.y &&
      this.min.z <= o.max.z && this.max.z >= o.min.z
    );
  }

  get center() {
    return this.min.add(this.max).mul(0.5);
  }

  get extents() {
    return this.max.sub(this.min).mul(0.5);
  }

  expandPoint(p: Vec3) {
    this.min = new Vec3(Math.min(this.min.x, p.x), Math.min(this.min.y, p.y), Math.min(this.min.z, p.z));
    this.max = new Vec3(Math.max(this.max.x, p.x), Math.max(this.max.y, p.y), Math.max(this.max.z, p.z));
  }

  expandRadius(radius: number) {
    this.min = new Vec3(f(this.min.x - radius), f(this.min.y - radius), f(this.min.z - radius));
    this.max = new Vec3(f(this.max.x + radius), f(this.max.y + radius), f(this.max.z + radius));
  }
}

/** Rotation + translation (Bullet btTransform). */
export class RigidTransform {
  constructor(
    readonly rotation: Quat,
    readonly origin: Vec3,
  ) {}

  static readonly Identity = new RigidTransform(Quat.Identity, Vec3.Zero);

  transformPoint(p: Vec3) {
    return this.rotation.rotate(p).add(this.origin);
  }

  transformDirection(v: Vec3) {
    return this.rotation.rotate(v);
  }

  inverseTransformPoint(p: Vec3) {
    return this.rotation.conjugated().rotate(p.sub(this.origin));
  }

  inverseTransformDirection(v: Vec3) {
    return this.rotation.conjugated().rotate(v);
  }

  inverse() {
    const invRot = this.rotation.conjugated();
    return new RigidTransform(invRot, invRot.rotate(this.origin.neg()));
  }

  /** a * b (b first, then a) */
  mul(b: RigidTransform) {
    return new RigidTransform(this.rotation.mul(b.rotation).normalized, this.rotation.rotate(b.origin).add(this.origin));
  }

  /** this^-1 * other */
  inverseTimes(other: RigidTransform) {
    const v = other.origin.sub(this.origin);
    const invRot = this.rotation.conjugated();
    return new RigidTransform(invRot.mul(other.rotation), invRot.rotate(v));
  }

  toMatrix() {
    return Matrix4x4.rotation(this.rotation).withTranslation(this.origin);
  }

  /** PMX (position, Euler radians, YXZ order) → transform */
  static fromEuler(pos: Vec3, eulerRad: Vec3) {
    return new RigidTransform(Quat.fromEulerYxz(eulerRad.x, eulerRad.y, eulerRad.z), pos);
  }

  equals(o: RigidTransform) {
    return this.rotation.equals(o.rotation) && this.origin.equals(o.origin);
  }

  toString() {
    return `T[${this.origin}, ${this.rotation}]`;
  }
}

/** 3x3 matrix (row-major), for inertia tensors and bases. */
export class Matrix3x3 {
  constructor(
    readonly row0: Vec3,
    readonly row1: Vec3,
    readonly row2: Vec3,
  ) {}

  static readonly Identity = new Matrix3x3(Vec3.XAxis, Vec3.YAxis, Vec3.ZAxis);
  static readonly Zero = new Matrix3x3(Vec3.Zero, Vec3.Zero, Vec3.Zero);

  static diagonal(d: Vec3) {
    return new Matrix3x3(new Vec3(d.x, 0, 0), new Vec3(0, d.y, 0), new Vec3(0, 0, d.z));
  }

  static fromQuat(q: Quat) {
    const xx = f(q.x * q.x), yy = f(q.y * q.y), zz = f(q.z * q.z);
    const xy = f(q.x * q.y), xz = f(q.x * q.z), yz = f(q.y * q.z);
    const wx = f(q.w * q.x), wy = f(q.w * q.y), wz = f(q.w * q.z);
    return new Matrix3x3(
      new Vec3(f(1 - f(2 * f(yy + zz))), f(2 * f(xy - wz)), f(2 * f(xz + wy))),
      new Vec3(f(2 * f(xy + wz)), f(1 - f(2 * f(xx + zz))), f(2 * f(yz - wx))),
      new Vec3(f(2 * f(xz - wy)), f(2 * f(yz + wx)), f(1 - f(2 * f(xx + yy)))),
    );
  }

  column(i: number) {
    switch (i) {
      case 0: return new Vec3(this.row0.x, this.row1.x, this.row2.x);
      case 1: return new Vec3(this.row0.y, this.row1.y, this.row2.y);
      case 2: return new Vec3(this.row0.z, this.row1.z, this.row2.z);
      default: throw new RangeError('i');
    }
  }

  /** Bullet 2.75 btMatrix3x3::setRotation (different rounding from fromQuat). */
  static fromQuatBullet(q: Quat) {
    const d = f(f(f(f(q.x * q.x) + f(q.y * q.y)) + f(q.z * q.z)) + f(q.w * q.w));
    const sc = f(2 / d);
    const xs = f(q.x * sc), ys = f(q.y * sc), zs = f(q.z * sc);
    const wx = f(q.w * xs), wy = f(q.w * ys), wz = f(q.w * zs);
    const xx = f(q.x * xs), xy = f(q.x * ys), xz = f(q.x * zs);
    const yy = f(q.y * ys), yz = f(q.y * zs), zz = f(q.z * zs);
    return new Matrix3x3(
      new Vec3(f(1 - f(yy + zz)), f(xy - wz), f(xz + wy)),
      new Vec3(f(xy + wz), f(1 - f(xx + zz)), f(yz - wx)),
      new Vec3(f(xz - wy), f(yz + wx), f(1 - f(xx + yy))),
    );
  }

  /** m * v */
  mulVec(v: Vec3) {
    return new Vec3(this.row0.dot(v), this.row1.dot(v), this.row2.dot(v));
  }

  /** a * b */
  mul(b: Matrix3x3) {
    const c0 = b.column(0), c1 = b.column(1), c2 = b.column(2);
    return new Matrix3x3(
      new Vec3(this.row0.dot(c0), this.row0.dot(c1), this.row0.dot(c2)),
      new Vec3(this.row1.dot(c0), this.row1.dot(c1), this.row1.dot(c2)),
      new Vec3(this.row2.dot(c0), this.row2.dot(c1), this.row2.dot(c2)),
    );
  }

  transposed() {
    return new Matrix3x3(this.column(0), this.column(1), this.column(2));
  }

  /** Bullet 2.75 btMatrix3x3::inverse() — cofactors / determinant (not the transpose). */
  bulletInverse() {
    const m00 = this.row0.x, m01 = this.row0.y, m02 = this.row0.z;
    const m10 = this.row1.x, m11 = this.row1.y, m12 = this.row1.z;
    const m20 = this.row2.x, m21 = this.row2.y, m22 = this.row2.z;
    const cofac = (a: number, b: number, c: number, d: number) => f(f(a * b) - f(c * d));
    const co = new Vec3(cofac(m11, m22, m12, m21), cofac(m12, m20, m10, m22), cofac(m10, m21, m11, m20));
    const det = this.row0.dot(co);
    const sc = f(1 / det);
    return new Matrix3x3(
      new Vec3(f(co.x * sc), f(cofac(m02, m21, m01, m22) * sc), f(cofac(m01, m12, m02, m11) * sc)),
      new Vec3(f(co.y * sc), f(cofac(m00, m22, m02, m20) * sc), f(cofac(m02, m10, m00, m12) * sc)),
      new Vec3(f(co.z * sc), f(cofac(m01, m20, m00, m21) * sc), f(cofac(m00, m11, m01, m10) * sc)),
    );
  }

  /** this * diag(scale) * this^T */
  scaled(scale: Vec3) {
    return this.mul(Matrix3x3.diagonal(scale)).mul(this.transposed());
  }
}
