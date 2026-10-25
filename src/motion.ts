// VMD motion playback for MMD models (PMX / PMD), using three.js's MMDAnimationHelper for the
// MMD-specific parts (bone order by deform layer, IK, "grant" / 付与親). The helper's own physics
// is not used: FacetGL's physics (physics.ts) runs after the helper has posed the bones.
import * as THREE from 'three';
import { MMDLoader } from 'three/addons/loaders/MMDLoader.js';
import { MMDAnimationHelper } from 'three/addons/animation/MMDAnimationHelper.js';
import { assetUrl, readFile, resolveUrl } from './platform';

export const MOTION_EXTENSIONS = ['vmd'];

export function isMotionFile(path: string) {
  return MOTION_EXTENSIONS.includes(path.split('.').pop()?.toLowerCase() ?? '');
}

// Meshes a VMD can drive: what MMDLoader made from a PMX / PMD
export function motionTarget(object: THREE.Object3D | null): THREE.SkinnedMesh | null {
  const mesh = object as THREE.SkinnedMesh | null;
  return mesh?.isSkinnedMesh && mesh.geometry.userData.MMD ? mesh : null;
}

interface HelperObjects {
  mixer: THREE.AnimationMixer;
}

interface IkLink {
  index: number;
  enabled?: boolean;
}
interface IkParam {
  target: number;
  links: IkLink[];
}

// --- IK on/off keys (VMD "表示・IK" section) ---
// three.js's VMD parser stops after the camera section and never reads these, so a motion that
// turns the leg IK off and drives the legs with FK keys (common for "full key" motions) had its
// legs pulled to the IK bones. Read them here.

const VMD_FPS = 30;

interface IkKey {
  frame: number;
  enabled: boolean;
}

function readIkKeys(buffer: ArrayBuffer): Map<string, IkKey[]> {
  const keys = new Map<string, IkKey[]>();
  const dv = new DataView(buffer);
  const sjis = new TextDecoder('shift_jis');
  const name = (offset: number, length: number) => {
    const bytes = new Uint8Array(buffer, offset, length);
    const end = bytes.indexOf(0);
    return sjis.decode(end >= 0 ? bytes.subarray(0, end) : bytes);
  };
  try {
    let o = 50; // magic (30) + model name (20)
    const skip = (recordSize: number) => {
      const n = dv.getUint32(o, true);
      o += 4 + n * recordSize;
    };
    skip(111); // bone keys
    skip(23); // morph keys
    if (o >= buffer.byteLength) return keys;
    skip(61); // camera
    if (o >= buffer.byteLength) return keys;
    skip(28); // light
    if (o >= buffer.byteLength) return keys;
    skip(9); // self shadow
    if (o >= buffer.byteLength) return keys;
    const count = dv.getUint32(o, true);
    o += 4;
    for (let i = 0; i < count; i++) {
      const frame = dv.getUint32(o, true);
      const ikCount = dv.getUint32(o + 5, true); // after frame (4) and show flag (1)
      o += 9;
      for (let j = 0; j < ikCount; j++) {
        const ikName = name(o, 20);
        const enabled = dv.getUint8(o + 20) !== 0;
        o += 21;
        let list = keys.get(ikName);
        if (!list) keys.set(ikName, (list = []));
        list.push({ frame, enabled });
      }
    }
  } catch {
    // Truncated or older file: use what was read
  }
  for (const list of keys.values()) list.sort((a, b) => a.frame - b.frame);
  return keys;
}

// The last key at or before the frame (IK is on before the first key, as in MMD)
function ikEnabledAt(keys: IkKey[], frame: number) {
  let enabled = true;
  for (const key of keys) {
    if (key.frame > frame) break;
    enabled = key.enabled;
  }
  return enabled;
}

export class MmdMotion {
  readonly mixer: THREE.AnimationMixer;
  readonly action: THREE.AnimationAction;

  private physicsEnabled = false;
  // IK solved by MMDAnimationHelper, with the VMD's on/off keys (null = no keys: always on)
  private readonly iks: { ik: IkParam; keys: IkKey[] | null }[];
  private readonly rigidBodyType: number[];

  private constructor(
    private readonly helper: MMDAnimationHelper,
    private readonly mesh: THREE.SkinnedMesh,
    readonly clip: THREE.AnimationClip,
    ikKeys: Map<string, IkKey[]>,
  ) {
    this.mixer = (helper.objects.get(mesh) as unknown as HelperObjects).mixer;
    this.action = this.mixer.existingAction(clip)!;
    const mmd = mesh.geometry.userData.MMD;
    this.rigidBodyType = (mmd.bones as { rigidBodyType: number }[]).map((b) => b.rigidBodyType);
    this.iks = (mmd.iks as IkParam[]).map((ik) => ({ ik, keys: ikKeys.get(mesh.skeleton.bones[ik.target]?.name) ?? null }));
  }

  /** Number of IK chains the motion switches on or off */
  get ikKeyCount() {
    return this.iks.filter((i) => i.keys).length;
  }

  static async load(mesh: THREE.SkinnedMesh, path: string): Promise<MmdMotion> {
    const manager = new THREE.LoadingManager();
    manager.setURLModifier(resolveUrl); // web version: virtual file → blob URL
    const [clip, buffer] = await Promise.all([
      new Promise<THREE.AnimationClip>((resolve, reject) => {
        new MMDLoader(manager).loadAnimation(assetUrl(path), mesh, (animation) => resolve(animation as THREE.AnimationClip), undefined, reject);
      }),
      readFile(path),
    ]);
    clip.name = path.split(/[\\/]/).pop() ?? clip.name;
    if (clip.tracks.length === 0) throw new Error('このモデルで動かせるボーンやモーフがモーションに含まれていません。');

    const helper = new MMDAnimationHelper({ afterglow: 0, resetPhysicsOnLoop: false });
    helper.add(mesh, { animation: clip, physics: false });
    const motion = new MmdMotion(helper, mesh, clip, readIkKeys(buffer));
    motion.update(0);
    return motion;
  }

  get duration() {
    return this.clip.duration;
  }

  // Advances the motion and poses the bones (IK and grant included)
  update(delta: number) {
    // The helper advances the mixer and then solves IK in the same call, so the IK switches are
    // set for the time the mixer is about to reach
    const duration = this.clip.duration;
    let time = this.action.time + delta * this.mixer.timeScale * this.action.timeScale;
    if (duration > 0) time = ((time % duration) + duration) % duration;
    this.applyIkSwitches(time);
    this.helper.update(delta);
  }

  // While physics drives the hair / skirt bones, IK must not move them (MMDAnimationHelper does
  // the same when its own physics is on: IK links on bones with a physics rigid body are skipped)
  setPhysicsEnabled(enabled: boolean) {
    this.physicsEnabled = enabled;
    this.applyIkSwitches(this.action.time);
  }

  // CCDIKSolver stops a chain at the first link with enabled === false
  private applyIkSwitches(time: number) {
    const frame = time * VMD_FPS;
    for (const { ik, keys } of this.iks) {
      const on = keys ? ikEnabledAt(keys, frame) : true;
      for (const link of ik.links) {
        link.enabled = on && !(this.physicsEnabled && this.rigidBodyType[link.index] > 0);
      }
    }
  }

  dispose() {
    for (const { ik } of this.iks) for (const link of ik.links) link.enabled = true;
    this.mixer.stopAllAction();
    this.mixer.uncacheRoot(this.mesh);
    this.helper.remove(this.mesh);
    this.mesh.pose();
  }
}
