// VMD motion playback for MMD models (PMX / PMD), using three.js's MMDAnimationHelper for the
// MMD-specific parts (bone order by deform layer, IK, "grant" / 付与親). The helper's own physics
// is not used: FacetGL's physics (physics.ts) runs after the helper has posed the bones.
import * as THREE from 'three';
import { MMDLoader } from 'three/addons/loaders/MMDLoader.js';
import { MMDAnimationHelper } from 'three/addons/animation/MMDAnimationHelper.js';
import { assetUrl, resolveUrl } from './platform';

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

export class MmdMotion {
  readonly mixer: THREE.AnimationMixer;
  readonly action: THREE.AnimationAction;

  private constructor(
    private readonly helper: MMDAnimationHelper,
    private readonly mesh: THREE.SkinnedMesh,
    readonly clip: THREE.AnimationClip,
  ) {
    this.mixer = (helper.objects.get(mesh) as unknown as HelperObjects).mixer;
    this.action = this.mixer.existingAction(clip)!;
  }

  static async load(mesh: THREE.SkinnedMesh, path: string): Promise<MmdMotion> {
    const manager = new THREE.LoadingManager();
    manager.setURLModifier(resolveUrl); // web version: virtual file → blob URL
    const clip = await new Promise<THREE.AnimationClip>((resolve, reject) => {
      new MMDLoader(manager).loadAnimation(assetUrl(path), mesh, (animation) => resolve(animation as THREE.AnimationClip), undefined, reject);
    });
    clip.name = path.split(/[\\/]/).pop() ?? clip.name;
    if (clip.tracks.length === 0) throw new Error('このモデルで動かせるボーンやモーフがモーションに含まれていません。');

    const helper = new MMDAnimationHelper({ afterglow: 0, resetPhysicsOnLoop: false });
    helper.add(mesh, { animation: clip, physics: false });
    const motion = new MmdMotion(helper, mesh, clip);
    motion.update(0);
    return motion;
  }

  get duration() {
    return this.clip.duration;
  }

  // Advances the motion and poses the bones (IK and grant included)
  update(delta: number) {
    this.helper.update(delta);
  }

  // While physics drives the hair / skirt bones, IK must not move them (MMDAnimationHelper does
  // the same when its own physics is on: IK links on bones with a physics rigid body are skipped)
  setPhysicsEnabled(enabled: boolean) {
    (this.helper as unknown as { _optimizeIK(mesh: THREE.SkinnedMesh, physicsEnabled: boolean): void })._optimizeIK(this.mesh, enabled);
  }

  dispose() {
    this.mixer.stopAllAction();
    this.mixer.uncacheRoot(this.mesh);
    this.helper.remove(this.mesh);
    this.mesh.pose();
  }
}
