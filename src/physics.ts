// MMD physics (rigid bodies and joints of PMX / PMD models) using three.js's MMDPhysics
// and ammo.js (Bullet compiled to WebAssembly).
//
// ammo.js is about 1 MB, so it is loaded only when physics is first turned on
// (public/libs/ammo/, the same way as the Draco and Basis decoders).
import * as THREE from 'three';
import { MMDPhysics } from 'three/addons/animation/MMDPhysics.js';

type AmmoModule = any;

declare global {
  interface Window {
    Ammo?: AmmoModule;
  }
}

let ammoReady: Promise<AmmoModule> | null = null;
// One Bullet world for the whole session: MMDPhysics has no dispose(), and the world's
// own parts (dispatcher, broadphase, solver) cannot be freed afterwards. Each model only
// adds and removes its bodies and joints.
let sharedWorld: AmmoModule = null;

function loadAmmo(): Promise<AmmoModule> {
  ammoReady ??= new Promise<AmmoModule>((resolve, reject) => {
    const script = document.createElement('script');
    // ammo.wasm.js finds ammo.wasm.wasm next to itself
    script.src = `${import.meta.env.BASE_URL}libs/ammo/ammo.wasm.js`;
    script.onerror = () => reject(new Error('物理演算ライブラリ (ammo.js) を読み込めませんでした。'));
    script.onload = () => {
      const factory = (window as any).Ammo;
      if (typeof factory !== 'function') {
        reject(new Error('物理演算ライブラリ (ammo.js) の初期化に失敗しました。'));
        return;
      }
      // The module is itself "then-able", so it is not passed to resolve() directly
      // (that would make the promise wait on it forever)
      factory().then((lib: AmmoModule) => {
        window.Ammo = lib; // MMDPhysics uses the global "Ammo"
        resolve(undefined);
      });
    };
    document.head.append(script);
  }).catch((error) => {
    ammoReady = null; // allow a retry
    throw error;
  });
  return ammoReady;
}

function getWorld() {
  if (!sharedWorld) {
    const Ammo = window.Ammo;
    const config = new Ammo.btDefaultCollisionConfiguration();
    const dispatcher = new Ammo.btCollisionDispatcher(config);
    const broadphase = new Ammo.btDbvtBroadphase();
    const solver = new Ammo.btSequentialImpulseConstraintSolver();
    sharedWorld = new Ammo.btDiscreteDynamicsWorld(dispatcher, broadphase, solver, config);
  }
  return sharedWorld;
}

// The PMX / PMD mesh if the model has rigid bodies, otherwise null
export function physicsMesh(object: THREE.Object3D | null): THREE.SkinnedMesh | null {
  const mesh = object as THREE.SkinnedMesh | null;
  if (!mesh?.isSkinnedMesh) return null;
  const mmd = mesh.geometry.userData.MMD;
  return mmd?.rigidBodies?.length > 0 ? mesh : null;
}

// Gravity in MMD units (1 unit = 8 cm): 9.8 m/s² × 10, as MMD itself uses
const GRAVITY = new THREE.Vector3(0, -9.8 * 10, 0);
const WARMUP_STEPS = 60;

export class ModelPhysics {
  private readonly physics: MMDPhysics;

  private constructor(private readonly mesh: THREE.SkinnedMesh) {
    const mmd = mesh.geometry.userData.MMD;
    const world = getWorld();
    world.setGravity(new window.Ammo.btVector3(GRAVITY.x, GRAVITY.y, GRAVITY.z));
    // "world" is accepted by MMDPhysics but missing from its type definition
    this.physics = new MMDPhysics(mesh, mmd.rigidBodies, mmd.constraints, { world } as any);
    this.settle();
  }

  static async create(mesh: THREE.SkinnedMesh) {
    await loadAmmo();
    return new ModelPhysics(mesh);
  }

  get bodyCount() {
    return this.physics.bodies.length;
  }

  // Advances the simulation by `delta` seconds and moves the physics bones
  update(delta: number) {
    this.mesh.updateMatrixWorld(true);
    this.physics.update(delta);
  }

  // Puts every bone back to the rest pose and lets the hair etc. settle there
  reset() {
    this.mesh.pose();
    this.settle();
  }

  private settle() {
    this.mesh.updateMatrixWorld(true);
    this.physics.reset();
    this.physics.warmup(WARMUP_STEPS);
  }

  // Removes this model's bodies and joints from the shared world and frees them,
  // then returns the bones to the rest pose
  dispose() {
    const Ammo = window.Ammo;
    const world = getWorld();
    for (const constraint of this.physics.constraints as any[]) {
      world.removeConstraint(constraint.constraint);
      Ammo.destroy(constraint.constraint);
    }
    for (const body of this.physics.bodies as any[]) {
      const rigidBody = body.body;
      world.removeRigidBody(rigidBody);
      Ammo.destroy(rigidBody.getMotionState());
      Ammo.destroy(rigidBody.getCollisionShape());
      Ammo.destroy(rigidBody);
    }
    this.mesh.pose();
  }
}
