import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { KTX2Loader } from 'three/addons/loaders/KTX2Loader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { OutlineEffect } from 'three/addons/effects/OutlineEffect.js';
import { updateRenderStats } from './info';
import { FORMATS, MODEL_EXTENSIONS, loadModelFile, type LoadedModel } from './formats';
import {
  isMac, isTauri, onModelDrop, onOpenFile, pickModelFile, pickModelFolder, setWindowTitle, showError, startupFile,
} from './platform';
import { CameraRig } from './cameraRig';
import { loadViewSettings, setupViewSettingsPanel, type ViewSettings } from './viewSettings';

// --- Variables ---
let scene: THREE.Scene;
let cameraRig: CameraRig;
let viewSettings: ViewSettings;
let renderer: THREE.WebGLRenderer;
let controls: OrbitControls;
let dirLight: THREE.DirectionalLight;
let ambientLight: THREE.AmbientLight;
let roomEnvironment: THREE.Texture;
let outlineEffect: OutlineEffect;
let useOutline = false;
const lightDirection = new THREE.Vector3(1, 2, 1.5).normalize();
let dracoLoader: DRACOLoader;
let ktx2Loader: KTX2Loader;
let currentModel: THREE.Object3D | null = null;
let currentLoaded: LoadedModel | null = null;
let loading = false;

// Animation state
const clock = new THREE.Clock();
let mixer: THREE.AnimationMixer | null = null;
let clips: THREE.AnimationClip[] = [];
let currentAction: THREE.AnimationAction | null = null;
let playing = false;
let seeking = false;
let statsElapsed = 0;
let shownFailedCount = 0;
let mmdExtrasEnabled = false;

// --- Initialization ---
function init() {
  // 1. Scene
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x1a1a1a);

  // 2. Camera (perspective / orthographic, see 表示設定)
  viewSettings = loadViewSettings();
  cameraRig = new CameraRig(viewSettings.fov, window.innerWidth / window.innerHeight, () => controls);

  // 3. Renderer
  renderer = new THREE.WebGLRenderer({ antialias: true });
  renderer.setSize(window.innerWidth, window.innerHeight);
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  renderer.shadowMap.enabled = true;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1;
  document.body.appendChild(renderer.domElement);

  // 4. Environment map (PBR metals look dark without one)
  const pmremGenerator = new THREE.PMREMGenerator(renderer);
  roomEnvironment = pmremGenerator.fromScene(new RoomEnvironment(renderer), 0.04).texture;
  scene.environment = roomEnvironment;
  pmremGenerator.dispose();

  // Edge outlines for MMD models (uses each material's PMX edge settings)
  outlineEffect = new OutlineEffect(renderer);

  // 5. Controls
  controls = new OrbitControls(cameraRig.active, renderer.domElement);
  controls.enableDamping = true;
  cameraRig.setProjection(viewSettings.projection);

  // 6. Lights
  ambientLight = new THREE.AmbientLight(0xffffff, 0.3);
  scene.add(ambientLight);

  dirLight = new THREE.DirectionalLight(0xffffff, 1.5);
  dirLight.position.set(5, 10, 7.5);
  dirLight.castShadow = true;
  dirLight.shadow.mapSize.set(2048, 2048);
  scene.add(dirLight);
  scene.add(dirLight.target);

  // 7. Compressed glTF support (decoders are served from public/libs; BASE_URL keeps the
  //    path relative so the same build works in the app and under GitHub Pages' sub-path)
  dracoLoader = new DRACOLoader().setDecoderPath(`${import.meta.env.BASE_URL}libs/draco/`);
  ktx2Loader = new KTX2Loader().setTranscoderPath(`${import.meta.env.BASE_URL}libs/basis/`).detectSupport(renderer);

  // 8. Event Listeners
  window.addEventListener('resize', onWindowResize);

  document.body.classList.toggle('web', !isTauri);
  const openButton = document.getElementById('open-button')!;
  if (isMac) openButton.title = '⌘O';
  openButton.addEventListener('click', openFileDialog);
  document.getElementById('open-folder-button')!.addEventListener('click', async () => {
    openModel(await pickModelFolder(MODEL_EXTENSIONS));
  });
  window.addEventListener('keydown', (e) => {
    if ((isMac ? e.metaKey : e.ctrlKey) && e.key.toLowerCase() === 'o') {
      e.preventDefault();
      openFileDialog();
    } else if (e.target instanceof HTMLElement && e.target.matches('input, select')) {
      return;
    } else if (e.key === ' ') {
      e.preventDefault();
      togglePlay();
    } else if (e.key.toLowerCase() === 'i') {
      toggleInfoPanel();
    } else if (e.key.toLowerCase() === 'p') {
      viewPanel.toggleProjection();
    }
  });

  const viewPanel = setupViewSettingsPanel(viewSettings, (settings) => {
    viewSettings = settings;
    cameraRig.setProjection(settings.projection);
    cameraRig.setFov(settings.fov);
    applyEdgeScale();
  });

  document.getElementById('info-button')!.addEventListener('click', toggleInfoPanel);
  document.getElementById('mmd-button')!.addEventListener('click', toggleMmdExtras);
  setupAnimationControls();

  // Drag and drop (files, or a whole folder on the web)
  onModelDrop(MODEL_EXTENSIONS, {
    enter: () => document.body.classList.add('dragover'),
    leave: () => document.body.classList.remove('dragover'),
    drop: openModel,
  });

  // File passed on launch (file association / "Open with"), and files the OS sends
  // to the running app later (macOS). The listener is set up first so nothing is missed.
  onOpenFile(loadModel)
    .then(() => startupFile())
    .then((path) => {
      if (path) loadModel(path);
    });
}

async function openFileDialog() {
  const filters = [{ name: '対応しているすべての形式', extensions: MODEL_EXTENSIONS }, ...FORMATS];
  openModel(await pickModelFile(filters, MODEL_EXTENSIONS));
}

function openModel(path: string | null) {
  if (path) loadModel(path);
  else showError(`対応している形式のファイルが見つかりませんでした。\n(${MODEL_EXTENSIONS.join(', ')})`);
}

async function loadModel(path: string) {
  if (loading) return;
  loading = true;

  const fileName = path.split(/[\\/]/).pop() ?? path;
  const info = document.getElementById('info')!;
  const previousInfo = info.textContent;
  info.textContent = `読み込み中: ${fileName}`;

  try {
    const loaded = await loadModelFile(path, { dracoLoader, ktx2Loader });

    // Clear previous model
    clearCurrentModel();
    currentLoaded = loaded;
    currentModel = loaded.object;
    scene.add(currentModel);

    // Setup shadows for all meshes in the model
    currentModel.traverse((child) => {
      const mesh = child as THREE.Mesh;
      if (mesh.isMesh) {
        mesh.geometry = compactGeometry(mesh.geometry);
        // Some OBJ files have no normals and would render black
        if (!mesh.geometry.getAttribute('normal')) mesh.geometry.computeVertexNormals();
        mesh.castShadow = true;
        mesh.receiveShadow = true;
        // Animated skinned meshes can move outside their rest-pose bounds
        if ((mesh as THREE.SkinnedMesh).isSkinnedMesh) mesh.frustumCulled = false;
      }
    });

    applyRenderStyle(loaded.style);
    applyEdgeScale();
    mmdExtrasEnabled = !!loaded.mmdExtras;
    document.body.classList.toggle('has-mmd-extras', !!loaded.mmdExtras);
    updateMmdButton();
    fitCameraToModel(currentModel);
    setupAnimations(currentModel, loaded.animations);
    loaded.renderInfo(document.getElementById('info-panel')!);

    document.body.classList.add('loaded');
    info.textContent = fileName;
    shownFailedCount = 0;
    setWindowTitle(`${fileName} - FacetGL`);
    console.log('Model loaded:', path);
  } catch (error) {
    console.error('Error loading model:', error);
    info.textContent = previousInfo;
    showError(`モデルの読み込みに失敗しました。\n${fileName}\n\n${error}`);
  } finally {
    loading = false;
  }
}

// Primitives exported from one mesh (e.g. MMD conversions) often share a single huge
// vertex buffer. three.js builds a morph target texture per geometry, so each one would
// carry every vertex of the model. Keep only the vertices each geometry actually uses.
function compactGeometry(geometry: THREE.BufferGeometry) {
  const index = geometry.index;
  const position = geometry.getAttribute('position');
  if (!index || !position) return geometry;

  const remap = new Int32Array(position.count).fill(-1);
  const used: number[] = [];
  for (let i = 0; i < index.count; i++) {
    const v = index.getX(i);
    if (remap[v] === -1) {
      remap[v] = used.length;
      used.push(v);
    }
  }
  if (used.length > position.count * 0.9) return geometry;

  const compacted = new THREE.BufferGeometry();
  compacted.name = geometry.name;
  compacted.userData = geometry.userData;

  const newIndex = used.length > 65535 ? new Uint32Array(index.count) : new Uint16Array(index.count);
  for (let i = 0; i < index.count; i++) newIndex[i] = remap[index.getX(i)];
  compacted.setIndex(new THREE.BufferAttribute(newIndex, 1));

  for (const [name, attribute] of Object.entries(geometry.attributes)) {
    compacted.setAttribute(name, compactAttribute(attribute, used));
  }
  for (const [name, targets] of Object.entries(geometry.morphAttributes)) {
    compacted.morphAttributes[name] = targets.map((target) => compactAttribute(target, used));
  }
  compacted.morphTargetsRelative = geometry.morphTargetsRelative;
  for (const group of geometry.groups) {
    compacted.addGroup(group.start, group.count, group.materialIndex);
  }

  geometry.dispose();
  return compacted;
}

function compactAttribute(attribute: THREE.BufferAttribute | THREE.InterleavedBufferAttribute, used: number[]) {
  const itemSize = attribute.itemSize;
  // Copy raw values so normalized integer attributes keep their encoding
  const source = attribute instanceof THREE.InterleavedBufferAttribute ? attribute.data.array : attribute.array;
  const stride = attribute instanceof THREE.InterleavedBufferAttribute ? attribute.data.stride : itemSize;
  const offset = attribute instanceof THREE.InterleavedBufferAttribute ? attribute.offset : 0;

  const ArrayType = source.constructor as new (length: number) => THREE.TypedArray;
  const array = new ArrayType(used.length * itemSize);
  for (let i = 0; i < used.length; i++) {
    const base = used[i] * stride + offset;
    for (let c = 0; c < itemSize; c++) array[i * itemSize + c] = source[base + c];
  }

  const compacted = new THREE.BufferAttribute(array, itemSize, attribute.normalized);
  compacted.name = attribute.name;
  return compacted;
}

// --- Animation ---

function setupAnimations(model: THREE.Object3D, animations: THREE.AnimationClip[]) {
  mixer = new THREE.AnimationMixer(model);
  clips = animations;
  currentAction = null;

  const select = document.getElementById('anim-clip') as HTMLSelectElement;
  select.replaceChildren(...clips.map((clip, i) => new Option(clip.name || `アニメーション ${i}`, String(i))));
  document.body.classList.toggle('has-animation', clips.length > 0);

  if (clips.length > 0) selectClip(0, true);
}

function selectClip(index: number, play: boolean) {
  if (!mixer) return;
  currentAction?.stop();
  currentAction = mixer.clipAction(clips[index]);
  currentAction.play();
  const seek = document.getElementById('anim-seek') as HTMLInputElement;
  seek.max = String(clips[index].duration);
  setPlaying(play);
  mixer.update(0);
}

function setPlaying(value: boolean) {
  playing = value && currentAction !== null;
  document.getElementById('anim-play')!.textContent = playing ? '❚❚' : '▶';
}

function togglePlay() {
  if (!currentAction) return;
  setPlaying(!playing);
}

function setAnimationTime(time: number) {
  if (!mixer || !currentAction) return;
  currentAction.time = time;
  mixer.update(0);
}

function setupAnimationControls() {
  const select = document.getElementById('anim-clip') as HTMLSelectElement;
  const seek = document.getElementById('anim-seek') as HTMLInputElement;
  const speed = document.getElementById('anim-speed') as HTMLSelectElement;

  document.getElementById('anim-play')!.addEventListener('click', togglePlay);
  select.addEventListener('change', () => selectClip(Number(select.value), playing));
  seek.addEventListener('pointerdown', () => { seeking = true; });
  seek.addEventListener('pointerup', () => { seeking = false; });
  seek.addEventListener('input', () => setAnimationTime(Number(seek.value)));
  speed.addEventListener('change', () => {
    if (mixer) mixer.timeScale = Number(speed.value);
  });
}

function updateAnimationUI() {
  if (!currentAction) return;
  const duration = currentAction.getClip().duration;
  const time = currentAction.time % duration;
  if (!seeking) (document.getElementById('anim-seek') as HTMLInputElement).value = String(time);
  document.getElementById('anim-time')!.textContent = `${time.toFixed(2)} / ${duration.toFixed(2)}`;
}

// --- Info panel ---

function toggleInfoPanel() {
  const show = document.body.classList.toggle('show-info');
  // Rebuild on open: textures of some formats finish loading after the model appears
  if (show && currentLoaded) currentLoaded.renderInfo(document.getElementById('info-panel')!);
}

// --- extras.mmd (glTF converted from PMX by mmd2gltf) ---

function toggleMmdExtras() {
  if (!currentLoaded?.mmdExtras) return;
  mmdExtrasEnabled = !mmdExtrasEnabled;
  currentLoaded.mmdExtras.setEnabled(mmdExtrasEnabled);
  applyRenderStyle(mmdExtrasEnabled ? 'mmd' : undefined);
  // The swapped-in materials may not have seen the current edge multiplier yet
  applyEdgeScale();
  if (currentModel) fitLightToModel(currentModel);
  updateMmdButton();
}

function updateMmdButton() {
  const button = document.getElementById('mmd-button')!;
  button.classList.toggle('active', mmdExtrasEnabled);
  button.textContent = mmdExtrasEnabled ? 'MMD表示: ON' : 'MMD表示: OFF';
}

function clearCurrentModel() {
  if (!currentModel) return;

  currentLoaded?.mmdExtras?.dispose();

  mixer?.stopAllAction();
  mixer?.uncacheRoot(currentModel);
  mixer = null;
  currentAction = null;
  setPlaying(false);

  scene.remove(currentModel);
  currentModel.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    mesh.geometry.dispose();
    const materials = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
    for (const material of materials) {
      for (const value of Object.values(material)) {
        if (value instanceof THREE.Texture) value.dispose();
      }
      material.dispose();
    }
  });
  currentLoaded?.blobUrls.forEach((u) => URL.revokeObjectURL(u));
  currentLoaded = null;
  currentModel = null;
}

// MMD toon materials are tuned for three.js's MMD example: no tone mapping, a strong
// gray ambient plus a white key light from the front-left, and edge outlines.
function applyRenderStyle(style: 'mmd' | undefined) {
  const mmd = style === 'mmd';
  renderer.toneMapping = mmd ? THREE.NoToneMapping : THREE.ACESFilmicToneMapping;
  scene.environment = mmd ? null : roomEnvironment;
  ambientLight.color.set(mmd ? 0xaaaaaa : 0xffffff);
  ambientLight.intensity = mmd ? 3 : 0.3;
  dirLight.intensity = mmd ? 3 : 1.5;
  lightDirection.set(mmd ? -1 : 1, mmd ? 1 : 2, mmd ? 1 : 1.5).normalize();
  useOutline = mmd;
}

// MMD-compatible renderers draw the edge "edge size" screen pixels wide, regardless of zoom
// or window size. MMDLoader instead stores edgeSize / 300 as a fraction of the screen
// height, which made outlines far heavier than MMD. OutlineEffect's thickness is in
// clip space (NDC), where 1 pixel = 2 / drawing-buffer height.
function applyEdgeScale() {
  const pixel = 2 / renderer.getDrawingBufferSize(new THREE.Vector2()).y;
  currentModel?.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      const outline = material.userData.outlineParameters;
      if (!outline) continue;
      outline.edgeSize ??= outline.thickness * 300; // undo MMDLoader's / 300
      outline.thickness = outline.edgeSize * viewSettings.edgeScale * pixel;
    }
  });
}

function fitCameraToModel(model: THREE.Object3D) {
  // Auto-center and auto-zoom camera
  const box = new THREE.Box3().setFromObject(model);
  if (box.isEmpty()) return;

  const size = box.getSize(new THREE.Vector3());
  cameraRig.fit(
    box.getCenter(new THREE.Vector3()),
    box.getBoundingSphere(new THREE.Sphere()).radius || 1,
    Math.max(size.x, size.y, size.z) || 1,
  );
  fitLightToModel(model);
}

// Place the key light along the current style's direction and fit its shadow camera
function fitLightToModel(model: THREE.Object3D) {
  const box = new THREE.Box3().setFromObject(model);
  if (box.isEmpty()) return;
  const center = box.getCenter(new THREE.Vector3());
  const radius = box.getBoundingSphere(new THREE.Sphere()).radius || 1;

  dirLight.position.copy(center).add(lightDirection.clone().multiplyScalar(radius * 2));
  dirLight.target.position.copy(center);
  const shadowCam = dirLight.shadow.camera;
  shadowCam.left = -radius;
  shadowCam.right = radius;
  shadowCam.top = radius;
  shadowCam.bottom = -radius;
  shadowCam.near = radius * 0.01;
  shadowCam.far = radius * 4;
  shadowCam.updateProjectionMatrix();
  dirLight.shadow.bias = -0.0005;
}

function onWindowResize() {
  cameraRig.setAspect(window.innerWidth / window.innerHeight);
  renderer.setSize(window.innerWidth, window.innerHeight);
  // Outlines are specified in pixels, so they depend on the canvas height
  applyEdgeScale();
}

function animate() {
  requestAnimationFrame(animate);
  const delta = clock.getDelta();

  if (mixer && playing && !seeking) mixer.update(delta);
  // VRM spring bones / expressions
  currentLoaded?.update?.(delta);
  updateAnimationUI();

  controls.update();
  const camera = cameraRig.active;
  if (useOutline && viewSettings.edgeScale > 0) outlineEffect.render(scene, camera);
  else renderer.render(scene, camera);

  // Refresh the live render stats a few times per second
  statsElapsed += delta;
  if (statsElapsed > 0.5) {
    statsElapsed = 0;
    if (document.body.classList.contains('show-info')) updateRenderStats(renderer);
    updateFailedNotice();
  }
}

// Textures load in the background, so failures can appear after the model is shown
function updateFailedNotice() {
  const failed = currentLoaded?.failedFiles ?? [];
  if (failed.length === shownFailedCount) return;
  shownFailedCount = failed.length;

  const info = document.getElementById('info')!;
  info.querySelector('.warn')?.remove();
  if (failed.length > 0) {
    const warn = document.createElement('span');
    warn.className = 'warn';
    warn.textContent = `  ⚠ 読み込めないファイル ${failed.length} 件 (詳細は「情報」)`;
    info.append(warn);
  }
  if (document.body.classList.contains('show-info')) currentLoaded?.renderInfo(document.getElementById('info-panel')!);
}

// Start the app
init();
animate();
