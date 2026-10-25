// ===========================================================================
// MMD physics engine (TypeScript port) – collision shapes
// Port of CollisionShape.cs (PMX shape 0: sphere, 1: box, 2: capsule).
// float32 emulation rules: see math.ts.
// ===========================================================================

import { f, Vec3 } from './math';

export enum ShapeType {
  Sphere = 0,
  Box = 1,
  Capsule = 2,
}

const F_004 = f(0.04);
const F_0999 = f(0.999);
const F_04 = f(0.4);
const F_1EM12 = f(1e-12);

export abstract class CollisionShape {
  abstract readonly type: ShapeType;

  /** Collision margin (Bullet default 0.04). */
  margin = F_004;

  /** Bullet 2.75 CONVEX_DISTANCE_MARGIN. */
  static readonly bulletConvexDistanceMargin = F_004;

  /** Task 38 switch (default false). Read at shape construction time. */
  static bulletShapeMargin = false;

  abstract localSupport(dir: Vec3): Vec3;

  localSupportWithMargin(dir: Vec3): Vec3 {
    const d = dir;
    const len2 = d.lengthSquared;
    if (len2 < F_1EM12) return this.localSupport(Vec3.XAxis);
    return this.localSupport(d).add(d.normalized.mul(this.margin));
  }

  abstract calculateLocalInertia(mass: number): Vec3;

  abstract get boundingRadius(): number;
}

export class SphereShape extends CollisionShape {
  readonly type = ShapeType.Sphere;
  radius: number;

  constructor(radius: number) {
    super();
    this.radius = f(radius);
    this.margin = this.radius;
  }

  localSupport(_dir: Vec3): Vec3 {
    return Vec3.Zero;
  }

  calculateLocalInertia(mass: number): Vec3 {
    const i = f(f(f(F_04 * mass) * this.radius) * this.radius);
    return new Vec3(i, i, i);
  }

  get boundingRadius() {
    return this.radius;
  }
}

export class BoxShape extends CollisionShape {
  readonly type = ShapeType.Box;
  halfExtents: Vec3;

  constructor(halfExtents: Vec3) {
    super();
    this.halfExtents = halfExtents;
    const minHalf = Math.min(Math.min(halfExtents.x, halfExtents.y), halfExtents.z);
    this.margin = CollisionShape.bulletShapeMargin
      ? Math.min(CollisionShape.bulletConvexDistanceMargin, f(minHalf * F_0999))
      : f(minHalf * F_004);
  }

  localSupport(dir: Vec3): Vec3 {
    const h = this.halfExtents.sub(Vec3.splat(this.margin));
    return new Vec3(dir.x >= 0 ? h.x : -h.x, dir.y >= 0 ? h.y : -h.y, dir.z >= 0 ? h.z : -h.z);
  }

  calculateLocalInertia(mass: number): Vec3 {
    const x = f(this.halfExtents.x * 2);
    const y = f(this.halfExtents.y * 2);
    const z = f(this.halfExtents.z * 2);
    const c = f(mass / 12);
    return new Vec3(
      f(c * f(f(y * y) + f(z * z))),
      f(c * f(f(x * x) + f(z * z))),
      f(c * f(f(x * x) + f(y * y))),
    );
  }

  get boundingRadius() {
    return this.halfExtents.length;
  }
}

export class CapsuleShape extends CollisionShape {
  readonly type = ShapeType.Capsule;
  radius: number;
  height: number;

  /** Margin added to the half extents for the inertia (Bullet CONVEX_DISTANCE_MARGIN). */
  static inertiaMargin = F_004;

  constructor(radius: number, height: number) {
    super();
    this.radius = f(radius);
    this.height = f(height);
    this.margin = CollisionShape.bulletShapeMargin
      ? Math.min(CollisionShape.bulletConvexDistanceMargin, this.radius)
      : this.radius;
  }

  get halfHeight() {
    return f(this.height * 0.5);
  }

  localSupport(dir: Vec3): Vec3 {
    const hh = this.halfHeight;
    const seg = dir.y >= 0 ? new Vec3(0, hh, 0) : new Vec3(0, -hh, 0);
    const core = f(this.radius - this.margin);
    if (core <= 0) return seg;
    const len2 = dir.lengthSquared;
    if (len2 < F_1EM12) return seg;
    return seg.add(dir.mul(f(core / f(Math.sqrt(len2)))));
  }

  calculateLocalInertia(mass: number): Vec3 {
    const r = this.radius;
    const m = CapsuleShape.inertiaMargin;
    const hx = f(r + m);
    const hy = f(f(r + this.halfHeight) + m);
    const hz = f(r + m);
    const lx = f(2 * hx);
    const ly = f(2 * hy);
    const lz = f(2 * hz);
    const c = f(mass / 12);
    return new Vec3(
      f(c * f(f(ly * ly) + f(lz * lz))),
      f(c * f(f(lx * lx) + f(lz * lz))),
      f(c * f(f(lx * lx) + f(ly * ly))),
    );
  }

  get boundingRadius() {
    return f(this.halfHeight + this.radius);
  }
}
