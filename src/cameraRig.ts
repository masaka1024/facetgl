import * as THREE from 'three';
import type { OrbitControls } from 'three/addons/controls/OrbitControls.js';

export type Projection = 'perspective' | 'orthographic';

// A perspective and an orthographic camera that share one view. Switching projection or
// changing the field of view keeps the model the same size on screen (measured at the
// orbit target), so only the amount of perspective changes.
export class CameraRig {
  readonly perspective: THREE.PerspectiveCamera;
  readonly orthographic: THREE.OrthographicCamera;
  private projection: Projection = 'perspective';

  constructor(fov: number, aspect: number, private readonly controls: () => OrbitControls) {
    this.perspective = new THREE.PerspectiveCamera(fov, aspect, 0.1, 1000);
    this.perspective.position.set(0, 2, 5);
    this.orthographic = new THREE.OrthographicCamera(-aspect, aspect, 1, -1, 0.1, 1000);
    this.orthographic.position.copy(this.perspective.position);
  }

  get active(): THREE.PerspectiveCamera | THREE.OrthographicCamera {
    return this.projection === 'perspective' ? this.perspective : this.orthographic;
  }

  get fov() {
    return this.perspective.fov;
  }

  setAspect(aspect: number) {
    this.perspective.aspect = aspect;
    this.perspective.updateProjectionMatrix();
    this.setOrthoHalfHeight(this.orthographic.top);
  }

  // Frame a model: the bounding sphere fits the view from slightly above and to the side
  fit(center: THREE.Vector3, radius: number, maxDim: number) {
    const vFov = THREE.MathUtils.degToRad(this.perspective.fov);
    const hFov = 2 * Math.atan(Math.tan(vFov / 2) * this.perspective.aspect);
    const distance = (radius / Math.sin(Math.min(vFov, hFov) / 2)) * 1.1; // Add some padding
    const position = center.clone().add(new THREE.Vector3(0.5, 0.4, 1).normalize().multiplyScalar(distance));

    for (const camera of [this.perspective, this.orthographic]) {
      camera.position.copy(position);
      // Scale clipping planes to the model so large/small models are not clipped
      camera.near = maxDim / 1000;
      camera.far = maxDim * 100;
    }
    this.perspective.updateProjectionMatrix();
    this.orthographic.zoom = 1;
    this.setOrthoHalfHeight(this.visibleHalfHeight(distance));

    const controls = this.controls();
    controls.target.copy(center);
    controls.maxDistance = maxDim * 50;
    controls.update();
  }

  setProjection(projection: Projection) {
    if (projection === this.projection) return;
    const controls = this.controls();
    const from = this.active;
    const target = controls.target;
    const direction = from.position.clone().sub(target).normalize();

    if (projection === 'orthographic') {
      // Same visible height at the target as the perspective view
      const distance = from.position.distanceTo(target);
      this.orthographic.position.copy(from.position);
      this.orthographic.zoom = 1;
      this.setOrthoHalfHeight(this.visibleHalfHeight(distance));
    } else {
      // Move the perspective camera to the distance where the target height matches
      const halfHeight = this.orthographic.top / this.orthographic.zoom;
      const distance = halfHeight / Math.tan(THREE.MathUtils.degToRad(this.perspective.fov) / 2);
      this.perspective.position.copy(target).add(direction.multiplyScalar(distance));
    }

    this.active.quaternion.copy(from.quaternion);
    this.projection = projection;
    controls.object = this.active;
    controls.update();
  }

  // Changing the field of view dollies the camera so the model keeps its size
  setFov(fov: number) {
    const camera = this.perspective;
    if (this.projection === 'perspective') {
      const target = this.controls().target;
      const distance = camera.position.distanceTo(target);
      const ratio = Math.tan(THREE.MathUtils.degToRad(camera.fov) / 2) / Math.tan(THREE.MathUtils.degToRad(fov) / 2);
      camera.position.sub(target).multiplyScalar(ratio).add(target);
      this.controls().maxDistance = Math.max(this.controls().maxDistance, distance * ratio * 2);
    }
    camera.fov = fov;
    camera.updateProjectionMatrix();
    this.controls().update();
  }

  private visibleHalfHeight(distance: number) {
    return distance * Math.tan(THREE.MathUtils.degToRad(this.perspective.fov) / 2);
  }

  private setOrthoHalfHeight(halfHeight: number) {
    const aspect = this.perspective.aspect;
    const camera = this.orthographic;
    camera.top = halfHeight;
    camera.bottom = -halfHeight;
    camera.left = -halfHeight * aspect;
    camera.right = halfHeight * aspect;
    camera.updateProjectionMatrix();
  }
}
