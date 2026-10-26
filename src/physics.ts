// MMD physics for PMX models (and glTF files converted from PMX, see gltfPhysics.ts), using the
// TypeScript port of the FacetGL author's own Bullet 2.75-compatible engine (src/physics/engine,
// ported from the C# engine https://github.com/masaka1024/mmd2gltf-cs-physics).
//
// The engine works in MMD's own (left-handed) coordinates and reads the rigid bodies and
// joints straight from the PMX file. This file is the boundary between it and three.js,
// and follows the Unity importer (MmdPhysicsBehaviour.cs) step for step:
//   push : bone world poses → kinematic (bone-following) bodies
//   step : World.stepSimulation(dt)   (fixed 1/60 s steps, 2 substeps, inside the engine)
//   pull : physics bodies → bone poses (mode 2 "physics + bone alignment" uses the aligned pose)
import * as THREE from 'three';
import {
  PhysicsMode,
  PmxPhysicsBuilder,
  initSolverKernel,
  Quat,
  RigidTransform,
  Vec3,
  readPmx,
} from './physics/engine';
import type { LoadedModel } from './formats';
import type { GltfPhysicsSource } from './gltfPhysics';

// Same settings as the Unity importer's defaults (MmdPhysicsBehaviour)
const GRAVITY = 98; // MMD scale (9.8 m/s² × 10)
const SOLVER_ITERATIONS = 10;
const SUB_STEPS = 2;
const FIXED_TIME_STEP = 1 / 60;
const ALIGN_ROT_CLAMP_ALPHA = 0.5;
// Longest time advanced in one call: after a stall, the engine would otherwise run many
// fixed steps at once and make the next frame even slower (Unity: MaxAllowedTimestep)
export const MAX_FRAME_TIME = 0.05;

// --- MMD (left-handed) ⇄ three.js (right-handed) ---
// MMDLoader mirrors the model across the XY plane: z is negated, and a rotation (x, y, z, w)
// becomes (-x, -y, z, w). glTF files from mmd2gltf use the same mirroring, with lengths scaled
// by unitScale (glTF length per MMD length; 1 for a PMX).
function toMmd(position: THREE.Vector3, quaternion: THREE.Quaternion, unitScale: number) {
  const f = Math.fround;
  return new RigidTransform(
    new Quat(f(-quaternion.x), f(-quaternion.y), f(quaternion.z), f(quaternion.w)),
    Vec3.of(position.x / unitScale, position.y / unitScale, -position.z / unitScale),
  );
}

function toThree(t: RigidTransform, position: THREE.Vector3, quaternion: THREE.Quaternion, unitScale: number) {
  position.set(t.origin.x * unitScale, t.origin.y * unitScale, -t.origin.z * unitScale);
  quaternion.set(-t.rotation.x, -t.rotation.y, t.rotation.z, t.rotation.w);
}

const isFiniteTransform = (t: RigidTransform) =>
  Number.isFinite(t.origin.x) && Number.isFinite(t.origin.y) && Number.isFinite(t.origin.z) &&
  Number.isFinite(t.rotation.x) && Number.isFinite(t.rotation.y) && Number.isFinite(t.rotation.z) && Number.isFinite(t.rotation.w);

// The PMX mesh (or a mesh of a glTF with MMD physics) if physics can run for this model,
// otherwise null
export function physicsMesh(loaded: LoadedModel | null): THREE.SkinnedMesh | null {
  if (loaded?.gltfPhysics) return loaded.gltfPhysics.mesh;
  const mesh = loaded?.object as THREE.SkinnedMesh | undefined;
  if (!loaded?.pmxSource || !mesh?.isSkinnedMesh) return null;
  return mesh.geometry.userData.MMD?.rigidBodies?.length > 0 ? mesh : null;
}

export class ModelPhysics {
  private readonly bones: THREE.Bone[];
  private readonly indexOfBone = new Map<THREE.Object3D, number>();
  private readonly boneWorldCache: (RigidTransform | null | undefined)[];
  // Bones whose pose comes from physics, parents before children
  private readonly physicsBones: number[];
  private readonly linkOfBone = new Map<number, { body: { worldTransform: RigidTransform }; offsetInverse: RigidTransform; mode: PhysicsMode }>();
  private readonly position = new THREE.Vector3();
  private readonly quaternion = new THREE.Quaternion();
  private readonly scale = new THREE.Vector3();
  private readonly matrix = new THREE.Matrix4();

  // glTF: local pose of the physics bones before physics took them (see dispose)
  private readonly restLocal: { bone: THREE.Bone; position: THREE.Vector3; quaternion: THREE.Quaternion }[] = [];

  private constructor(
    private readonly mesh: THREE.SkinnedMesh,
    private readonly builder: PmxPhysicsBuilder,
    // PMX: the mesh (its bones are its children); glTF: the model root
    private readonly root: THREE.Object3D,
    bones: THREE.Bone[],
    private readonly unitScale: number,
    private readonly isGltf: boolean,
  ) {
    this.bones = bones;
    this.bones.forEach((bone, i) => this.indexOfBone.set(bone, i));
    this.boneWorldCache = new Array(this.bones.length);

    const world = builder.world;
    world.gravity = new Vec3(0, -GRAVITY, 0);
    world.solverIterations = SOLVER_ITERATIONS;
    world.subSteps = SUB_STEPS;
    world.fixedTimeStep = Math.fround(FIXED_TIME_STEP);
    world.useSplitImpulse = false;
    world.useJointSplitImpulse = false;
    world.enableSleeping = false;
    world.useBroadphasePrefilter = true;

    for (const link of builder.boneLinks) {
      if (link.mode === PhysicsMode.BoneFollow) continue;
      if (link.boneIndex < 0 || link.boneIndex >= this.bones.length) continue;
      this.linkOfBone.set(link.boneIndex, { body: link.body, offsetInverse: link.bodyOffsetFromBone.inverse(), mode: link.mode });
    }
    const depth = (bone: THREE.Object3D) => {
      let d = 0;
      for (let b = bone.parent; b; b = b.parent) d++;
      return d;
    };
    this.physicsBones = [...this.linkOfBone.keys()].sort((a, b) => depth(this.bones[a]) - depth(this.bones[b]) || a - b);
    if (isGltf) {
      for (const index of this.physicsBones) {
        const bone = this.bones[index];
        this.restLocal.push({ bone, position: bone.position.clone(), quaternion: bone.quaternion.clone() });
      }
    }

    this.settle();
  }

  static async create(loaded: LoadedModel) {
    if (loaded.gltfPhysics) return ModelPhysics.createGltf(loaded.gltfPhysics);
    const mesh = physicsMesh(loaded);
    if (!mesh || !loaded.pmxSource) throw new Error('このモデルは物理演算に対応していません。');
    // The float32 WebAssembly kernel (same results, about 3x faster). Browsers refuse to compile
    // it synchronously on the main thread, so it is prepared here; without it the engine uses
    // the TypeScript path.
    const [buffer] = await Promise.all([loaded.pmxSource(), initSolverKernel()]);
    const model = readPmx(buffer);
    if (model.boneNames.length !== mesh.skeleton.bones.length) {
      throw new Error(`ボーン数が一致しません (PMX ${model.boneNames.length} / 表示 ${mesh.skeleton.bones.length})`);
    }
    PmxPhysicsBuilder.maxDynamicMass = 0; // engine default (no mass clamp)
    return new ModelPhysics(mesh, PmxPhysicsBuilder.build(model), mesh, mesh.skeleton.bones, 1, false);
  }

  private static async createGltf(source: GltfPhysicsSource) {
    await initSolverKernel();
    PmxPhysicsBuilder.maxDynamicMass = 0;
    return new ModelPhysics(source.mesh, PmxPhysicsBuilder.build(source.model), source.root, source.bones, source.unitScale, true);
  }

  get bodyCount() {
    return this.builder.world.bodies.length;
  }

  // Advances the simulation by `delta` seconds and moves the physics bones
  update(delta: number) {
    this.readBoneWorlds();
    this.builder.applyKinematicTargets(this.boneWorld);
    this.builder.world.stepSimulation(Math.fround(Math.min(delta, MAX_FRAME_TIME)));
    this.writeBones();
  }

  // Puts every bone back to the rest pose and places the bodies there (MMD's physics reset)
  reset() {
    this.restPose();
    this.settle();
  }

  // Places the bodies around the current (animated) pose without touching it. Used when a
  // motion starts, loops or is seeked: the bones jump, and the bodies would otherwise be
  // yanked across the distance in one step (the Unity importer's teleport reset).
  realign() {
    this.settle();
  }

  // restPose = false keeps the bones as they are (a motion will pose them again)
  dispose(restPose = true) {
    if (restPose) this.restPose();
  }

  // PMX: the whole skeleton (nothing else moves it without a motion). glTF: only the physics
  // bones, so that the body keeps the pose of the file's own animation.
  private restPose() {
    if (!this.isGltf) {
      this.mesh.pose();
      return;
    }
    for (const r of this.restLocal) {
      r.bone.position.copy(r.position);
      r.bone.quaternion.copy(r.quaternion);
    }
  }

  private settle() {
    this.readBoneWorlds();
    this.builder.applyKinematicTargets(this.boneWorld);
    this.builder.resetBodiesToBonePoseFk(this.boneWorld);
    this.writeBones();
  }

  // Bone world poses in MMD coordinates, read lazily from the current matrices
  private readBoneWorlds() {
    this.root.updateMatrixWorld(true);
    this.boneWorldCache.fill(undefined);
  }

  private readonly boneWorld = (index: number): RigidTransform | null => {
    if (index < 0 || index >= this.bones.length) return null;
    let cached = this.boneWorldCache[index];
    if (cached === undefined) {
      this.bones[index].matrixWorld.decompose(this.position, this.quaternion, this.scale);
      cached = toMmd(this.position, this.quaternion, this.unitScale);
      this.boneWorldCache[index] = cached;
    }
    return cached;
  };

  // Physics → bones. mode 1: the body's pose; mode 2: position rebuilt from the parent
  // chain, rotation from physics (the engine's ComputeAlignedBonePoses, as in Unity).
  private writeBones() {
    const aligned = this.builder.hasBoneMergeBodies
      ? this.builder.computeAlignedBonePoses(this.boneWorld, ALIGN_ROT_CLAMP_ALPHA, false)
      : null;

    // New world matrices of the physics bones (three.js coordinates), parents first
    const newWorld = new Map<number, THREE.Matrix4>();
    const worldOf = (bone: THREE.Object3D): THREE.Matrix4 => {
      const index = this.indexOfBone.get(bone) ?? -1;
      const own = newWorld.get(index);
      if (own) return own;
      // A non-physics bone below a physics bone moves with it
      const parent = bone.parent;
      if (parent && parent !== this.root && this.hasPhysicsAncestor(parent)) {
        return new THREE.Matrix4().multiplyMatrices(worldOf(parent), bone.matrix);
      }
      return bone.matrixWorld;
    };

    for (const index of this.physicsBones) {
      const link = this.linkOfBone.get(index)!;
      const useAligned = aligned !== null && link.mode === PhysicsMode.DynamicBoneMerge && aligned[index] != null;
      const world = useAligned ? aligned[index]! : link.body.worldTransform.mul(link.offsetInverse);
      if (!isFiniteTransform(world)) continue; // a diverged body must not break the skeleton

      toThree(world, this.position, this.quaternion, this.unitScale);
      const m = new THREE.Matrix4().compose(this.position, this.quaternion, this.scale.set(1, 1, 1));
      newWorld.set(index, m);

      // local = parentWorld⁻¹ · world
      const bone = this.bones[index];
      const parent = bone.parent;
      if (parent) this.matrix.copy(worldOf(parent)).invert().multiply(m);
      else this.matrix.copy(m);
      this.matrix.decompose(bone.position, bone.quaternion, this.scale);
    }
  }

  private hasPhysicsAncestor(object: THREE.Object3D) {
    for (let o: THREE.Object3D | null = object; o && o !== this.root; o = o.parent) {
      const index = this.indexOfBone.get(o);
      if (index !== undefined && this.linkOfBone.has(index)) return true;
    }
    return false;
  }
}
