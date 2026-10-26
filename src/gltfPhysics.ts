// MMD physics for glTF files converted from PMX by mmd2gltf / mmd2gltf-cs.
//
// Those files keep the PMX rigid bodies and joints in extras.mmd as raw PMX values (MMD's
// left-handed space, unscaled; glTF lengths are MMD lengths × unitScale), and the PMX bones in
// extras.mmd.bones in PMX order. That is all the physics engine reads from a PMX file except
// the bone positions, which come from the rest pose of the glTF skeleton.
//
// Baked physics: a converter can bake the physics into the animation (the hair and skirt
// bones then carry their own keyframes). A baked bone keeps its animation: its bodies follow
// the bone instead of being simulated. A file whose animation only moves the body still gets
// physics for the hair and skirt.
import * as THREE from 'three';
import type { GLTF } from 'three/addons/loaders/GLTFLoader.js';
import { PmxJoint, PmxPhysicsModel, PmxRigidBody, Vec3 } from './physics/engine';

export interface GltfPhysicsSource {
  /** Root of the loaded model; the bones are below it (not below the meshes as in a PMX). */
  root: THREE.Object3D;
  /** A mesh skinned to the bones (used to put the skeleton back to its rest pose). */
  mesh: THREE.SkinnedMesh;
  /** The bones in PMX order (= the bone indices used by the rigid bodies). */
  bones: THREE.Bone[];
  /** glTF length per MMD length. */
  unitScale: number;
  model: PmxPhysicsModel;
  /** Simulated bodies whose bone is animated, and therefore follow the animation instead. */
  bakedBodies: number;
}

export interface GltfPhysicsInfo {
  /** null: no physics to run (no rigid bodies, unsupported data, or everything baked). */
  source: GltfPhysicsSource | null;
  /** The file has simulated bodies, but the animation already moves all their bones. */
  baked: boolean;
}

const NONE: GltfPhysicsInfo = { source: null, baked: false };

// extras.mmd is plain JSON written by the converter
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Json = any;

const vec = (v: Json): Vec3 =>
  Array.isArray(v) && v.length >= 3 ? Vec3.of(Number(v[0]) || 0, Number(v[1]) || 0, Number(v[2]) || 0) : Vec3.Zero;
const num = (v: Json, fallback = 0) => (typeof v === 'number' && Number.isFinite(v) ? v : fallback);
const index = (v: Json) => (Number.isInteger(v) ? (v as number) : -1);

const isSimulated = (mode: number) => mode === 1 || mode === 2;

/** Call right after loading, before any animation has moved the bones (rest pose). */
export function readGltfPhysics(gltf: GLTF): GltfPhysicsInfo {
  const json = gltf.parser.json;
  const mmd = json.extras?.mmd;
  const rigidBodies: Json[] = Array.isArray(mmd?.rigidBodies) ? mmd.rigidBodies : [];
  const joints: Json[] = Array.isArray(mmd?.joints) ? mmd.joints : [];
  const mmdBones: Json[] = Array.isArray(mmd?.bones) ? mmd.bones : [];
  const unitScale = num(mmd?.unitScale);
  if (rigidBodies.length === 0 || mmdBones.length === 0 || !(unitScale > 0)) return NONE;
  if (!rigidBodies.some((rb) => isSimulated(num(rb.mode)))) return NONE;

  // glTF node index → three.js object
  const objectOfNode = new Map<number, THREE.Object3D>();
  gltf.scene.traverse((object) => {
    const node = gltf.parser.associations.get(object)?.nodes;
    if (node !== undefined && !objectOfNode.has(node)) objectOfNode.set(node, object);
  });

  // PMX bone → glTF node, by name. The skin's joints come first: other nodes (the mesh, the
  // skeleton root) can share a name with a bone.
  const nodeOfName = new Map<string, number>();
  for (const skin of json.skins ?? []) {
    for (const node of skin.joints ?? []) {
      const name = json.nodes?.[node]?.name;
      if (typeof name === 'string' && !nodeOfName.has(name)) nodeOfName.set(name, node);
    }
  }
  (json.nodes ?? []).forEach((node: Json, i: number) => {
    if (typeof node?.name === 'string' && !nodeOfName.has(node.name)) nodeOfName.set(node.name, i);
  });

  const boneNodes: number[] = [];
  const bones: THREE.Bone[] = [];
  for (const b of mmdBones) {
    const node = nodeOfName.get(b?.name);
    const object = node !== undefined ? objectOfNode.get(node) : undefined;
    if (node === undefined || !(object as THREE.Bone | undefined)?.isBone) return NONE;
    boneNodes.push(node);
    bones.push(object as THREE.Bone);
  }

  let mesh: THREE.SkinnedMesh | null = null;
  gltf.scene.traverse((object) => {
    const m = object as THREE.SkinnedMesh;
    if (!mesh && m.isSkinnedMesh && m.skeleton.bones.includes(bones[0])) mesh = m;
  });
  if (!mesh) return NONE;

  // Bones moved by any animation of the file: their physics is baked into it
  const animatedNodes = new Set<number>();
  for (const animation of json.animations ?? []) {
    for (const channel of animation.channels ?? []) {
      const path = channel.target?.path;
      if (path === 'rotation' || path === 'translation') animatedNodes.add(channel.target.node);
    }
  }

  const model = new PmxPhysicsModel();
  model.modelName = typeof mmd.name === 'string' ? mmd.name : '';

  // Rest pose of the bones in MMD coordinates, relative to the model root
  gltf.scene.updateMatrixWorld(true);
  const rootInverse = gltf.scene.matrixWorld.clone().invert();
  const position = new THREE.Vector3();
  mmdBones.forEach((b, i) => {
    position.setFromMatrixPosition(bones[i].matrixWorld).applyMatrix4(rootInverse).divideScalar(unitScale);
    model.boneNames.push(String(b.name));
    model.bonePositions.push(Vec3.of(position.x, position.y, -position.z));
    model.boneParents.push(index(b.parent));
    model.boneDeformLayers.push(num(b.deformLayer));
  });

  let simulated = 0;
  let bakedBodies = 0;
  for (const rb of rigidBodies) {
    const body = new PmxRigidBody();
    body.name = String(rb.name ?? '');
    body.nameEn = String(rb.name_en ?? '');
    body.boneIndex = index(rb.bone);
    body.group = num(rb.group);
    body.nonCollisionGroup = num(rb.no_collision_mask);
    body.shapeType = num(rb.shape);
    body.size = vec(rb.size);
    body.position = vec(rb.pos);
    body.rotation = vec(rb.rot);
    body.mass = num(rb.mass);
    body.linearDamping = num(rb.linear_damping);
    body.angularDamping = num(rb.angular_damping);
    body.restitution = num(rb.restitution);
    body.friction = num(rb.friction);
    body.physicsMode = num(rb.mode);
    if (isSimulated(body.physicsMode)) {
      const bone = body.boneIndex >= 0 && body.boneIndex < boneNodes.length ? boneNodes[body.boneIndex] : -1;
      if (animatedNodes.has(bone)) {
        body.physicsMode = 0; // baked: follow the animated bone
        bakedBodies++;
      } else {
        simulated++;
      }
    }
    model.rigidBodies.push(body);
  }

  for (const j of joints) {
    const joint = new PmxJoint();
    joint.name = String(j.name ?? '');
    joint.nameEn = String(j.name_en ?? '');
    joint.jointType = num(j.type);
    joint.rigidBodyAIndex = index(j.rigid_a);
    joint.rigidBodyBIndex = index(j.rigid_b);
    joint.position = vec(j.pos);
    joint.rotation = vec(j.rot);
    joint.linearLowerLimit = vec(j.pos_min);
    joint.linearUpperLimit = vec(j.pos_max);
    joint.angularLowerLimit = vec(j.rot_min);
    joint.angularUpperLimit = vec(j.rot_max);
    joint.springLinear = vec(j.spring_pos);
    joint.springAngular = vec(j.spring_rot);
    model.joints.push(joint);
  }

  if (simulated === 0) return { source: null, baked: true };
  return { source: { root: gltf.scene, mesh, bones, unitScale, model, bakedBodies }, baked: false };
}
