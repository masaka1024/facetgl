// ===========================================================================
// MMD physics engine (TypeScript port of the C# BulletPhysics engine) – public API
// ===========================================================================

export * from './math';
export { ShapeType, CollisionShape, SphereShape, BoxShape, CapsuleShape } from './shapes';
export { PhysicsMode, RigidBody } from './rigidBody';
export { ContactPoint, PersistentManifold, GjkEpa } from './collision';
export { Joint, JointType } from './constraints';
export { PhysicsWorld } from './world';
export { readPmx, PmxPhysicsModel, PmxRigidBody, PmxJoint, PmxSoftBody } from './pmx';
export type { PmxSoftBodyAnchor } from './pmx';
export { PmxPhysicsBuilder } from './builder';
export type { BoneLink, BoneWorldFn } from './builder';
export { initSolverKernel, solverKernelOptions } from './wasmSolver';
