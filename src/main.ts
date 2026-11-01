import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import { KTX2Loader } from 'three/addons/loaders/KTX2Loader.js';
import { RoomEnvironment } from 'three/addons/environments/RoomEnvironment.js';
import { OutlineEffect } from 'three/addons/effects/OutlineEffect.js';
import { updateRenderStats } from './info';
import { FORMATS, MODEL_EXTENSIONS, loadModelFile, type LoadedModel } from './formats';
import {
  isMac, isTauri, onModelDrop, onOpenFile, pickModelFile, pickModelFolder, pickMotionFile, setWindowTitle, showError, startupFile,
  type PickedFiles,
} from './platform';
import { CameraRig } from './cameraRig';
import { loadViewSettings, setupViewSettingsPanel, type ViewSettings } from './viewSettings';
import { MAX_FRAME_TIME, ModelPhysics, physicsMesh } from './physics';
import { MOTION_EXTENSIONS, MmdMotion, isMotionFile, motionTarget } from './motion';
import { applyStaticTexts, onLanguageChange, t } from './i18n';

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
let viewPanel: ReturnType<typeof setupViewSettingsPanel> | null = null;

// Physics state (PMX, and glTF from mmd2gltf with extras.mmd rigid bodies)
let physics: ModelPhysics | null = null;
let physicsStarting = false;
let physicsTime = 0; // milliseconds spent in physics since the last stats refresh
let physicsFrames = 0;
let physicsTimeText = ''; // " (4.0 ms)" on the physics button

// VMD motion of the current MMD model (motion.ts)
let motion: MmdMotion | null = null;
let motionLoading = false;
let lastMotionTime = 0;
let lastClipTime = 0; // the same for the file's own animation (glTF etc.)
// Alt (option) + drag moves the model to shake it; it springs back when released
const modelDrag = {
  active: false,
  pointerId: -1,
  plane: new THREE.Plane(),
  start: new THREE.Vector3(),
  startOffset: new THREE.Vector3(),
  offset: new THREE.Vector3(),
};

// --- Initialization ---
function init() {
  // Texts in index.html in the current language (Japanese or English, see i18n.ts)
  applyStaticTexts();

  // Which build this is (version, branch, commit), so that test builds can be told apart
  setWindowTitle(`FacetGL ${__BUILD_LABEL__}`);
  document.getElementById('info')!.textContent = `FacetGL ${__BUILD_LABEL__}`;

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
  document.getElementById('motion-button')!.addEventListener('click', async () => {
    const picked = await pickMotionFile({ name: t('dialog.motion'), extensions: MOTION_EXTENSIONS }, MODEL_EXTENSIONS);
    if (picked) openModel(picked);
  });
  document.getElementById('open-folder-button')!.addEventListener('click', async () => {
    const picked = await pickModelFolder(MODEL_EXTENSIONS);
    if (picked) openModel(picked);
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
      viewPanel?.toggleProjection();
    } else if (e.key.toLowerCase() === 'f') {
      togglePhysics();
    } else if (e.key.toLowerCase() === 'r') {
      resetPhysics();
    }
  });

  viewPanel = setupViewSettingsPanel(viewSettings, (settings) => {
    viewSettings = settings;
    cameraRig.setProjection(settings.projection);
    cameraRig.setFov(settings.fov);
    applyEdgeScale();
  });

  document.getElementById('info-button')!.addEventListener('click', toggleInfoPanel);
  document.getElementById('mmd-button')!.addEventListener('click', toggleMmdExtras);
  document.getElementById('physics-button')!.addEventListener('click', togglePhysics);
  document.getElementById('physics-reset')!.addEventListener('click', resetPhysics);
  updatePhysicsHint();
  onLanguageChange(refreshTexts);
  setupModelDrag();
  setupAnimationControls();

  // Drag and drop (files, or a whole folder on the web)
  onModelDrop(MODEL_EXTENSIONS, {
    enter: () => document.body.classList.add('dragover'),
    leave: () => document.body.classList.remove('dragover'),
    drop: openModel,
  });

  // File passed on launch (file association / "Open with"), and files the OS sends
  // to the running app later (macOS). The listener is set up first so nothing is missed.
  const openPath = (path: string) => openModel(isMotionFile(path) ? { model: null, motion: path } : { model: path, motion: null });
  onOpenFile(openPath)
    .then(() => startupFile())
    .then((path) => {
      if (path) openPath(path);
    });
}

async function openFileDialog() {
  const filters = [
    { name: t('dialog.allFormats'), extensions: [...MODEL_EXTENSIONS, ...MOTION_EXTENSIONS] },
    ...FORMATS,
    { name: t('dialog.motion'), extensions: MOTION_EXTENSIONS },
  ];
  const picked = await pickModelFile(filters, MODEL_EXTENSIONS);
  if (picked) openModel(picked);
}

// A model, a motion for the open MMD model, or both (a folder with a PMX and a VMD)
async function openModel(picked: PickedFiles | null) {
  if (!picked) return;
  if (picked.model) {
    await loadModel(picked.model);
    if (picked.motion && motionTarget(currentModel)) await loadMotion(picked.motion);
  } else if (picked.motion) {
    if (motionTarget(currentModel)) await loadMotion(picked.motion);
    else showError(t('error.motionNeedsModel'));
  } else {
    showError(t('error.noSupportedFile', { extensions: [...MODEL_EXTENSIONS, ...MOTION_EXTENSIONS].join(', ') }));
  }
}

async function loadMotion(path: string) {
  const mesh = motionTarget(currentModel);
  if (!mesh || motionLoading) return;
  motionLoading = true;
  const info = document.getElementById('info')!;
  const previousInfo = info.textContent;
  const fileName = path.split(/[\\/]/).pop() ?? path;
  info.textContent = t('status.loadingMotion', { name: fileName });
  try {
    const loaded = await MmdMotion.load(mesh, path);
    if (currentModel !== mesh) {
      loaded.dispose(); // another model was opened meanwhile
      return;
    }
    motion?.dispose();
    motion = loaded;
    motion.setPhysicsEnabled(physics !== null);
    useMotionInAnimationBar(motion);
    physics?.realign(); // the bones jumped to the motion's first frame
    info.textContent = previousInfo;
  } catch (error) {
    console.error('Error loading motion:', error);
    info.textContent = previousInfo;
    showError(t('error.motionLoad', { name: fileName, error: String(error) }));
  } finally {
    motionLoading = false;
  }
}

async function loadModel(path: string) {
  if (loading) return;
  loading = true;

  const fileName = path.split(/[\\/]/).pop() ?? path;
  const info = document.getElementById('info')!;
  const previousInfo = info.textContent;
  info.textContent = t('status.loading', { name: fileName });

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
    document.body.classList.toggle('has-physics', !!physicsMesh(currentLoaded));
    document.body.classList.toggle('can-play-motion', !!motionTarget(currentModel));
    document.body.classList.toggle('physics-baked', !!currentLoaded?.physicsBaked);
    updatePhysicsButton();
    fitCameraToModel(currentModel);
    setupAnimations(currentModel, loaded.animations);
    loaded.renderInfo(document.getElementById('info-panel')!);

    document.body.classList.add('loaded');
    info.textContent = fileName;
    shownFailedCount = 0;
    setWindowTitle(`${fileName} - FacetGL ${__BUILD_LABEL__}`);
    console.log('Model loaded:', path);
  } catch (error) {
    console.error('Error loading model:', error);
    info.textContent = previousInfo;
    showError(t('error.modelLoad', { name: fileName, error: String(error) }));
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
  select.replaceChildren(...clips.map((clip, i) => new Option(clip.name || t('anim.unnamed', { index: i }), String(i))));
  document.body.classList.toggle('has-animation', clips.length > 0);

  if (clips.length > 0) selectClip(0, true);
}

// A VMD motion takes over the animation bar (one clip, played through MMDAnimationHelper)
function useMotionInAnimationBar(m: MmdMotion) {
  mixer = m.mixer;
  clips = [m.clip];
  currentAction = m.action;
  mixer.timeScale = Number((document.getElementById('anim-speed') as HTMLSelectElement).value);
  const select = document.getElementById('anim-clip') as HTMLSelectElement;
  select.replaceChildren(new Option(m.clip.name, '0'));
  (document.getElementById('anim-seek') as HTMLInputElement).max = String(m.duration);
  document.body.classList.add('has-animation');
  lastMotionTime = m.action.time;
  setPlaying(true);
}

function selectClip(index: number, play: boolean) {
  if (!mixer || motion) return;
  currentAction?.stop();
  currentAction = mixer.clipAction(clips[index]);
  currentAction.play();
  const seek = document.getElementById('anim-seek') as HTMLInputElement;
  seek.max = String(clips[index].duration);
  setPlaying(play);
  mixer.update(0);
  lastClipTime = currentAction.time;
  physics?.realign(); // the pose jumped to the new clip
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
  if (motion) {
    motion.update(0);
    lastMotionTime = time;
    physics?.realign(); // the pose jumped: place the bodies around it
  } else {
    mixer.update(0);
    lastClipTime = time;
    physics?.realign();
  }
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
  button.textContent = mmdExtrasEnabled ? t('mmd.on') : t('mmd.off');
}

// --- Physics (PMX rigid bodies, see physics.ts) ---

async function togglePhysics() {
  if (physics) {
    stopPhysics();
    updatePhysicsButton();
    return;
  }
  const loaded = currentLoaded;
  if (!physicsMesh(loaded) || physicsStarting) return;
  physicsStarting = true;
  updatePhysicsButton();
  try {
    const created = await ModelPhysics.create(loaded!);
    if (currentLoaded === loaded) {
      physics = created;
      motion?.setPhysicsEnabled(true);
    }
    else created.dispose(); // another model was opened while the PMX was being read
  } catch (error) {
    console.error('Error starting physics:', error);
    showError(t('error.physicsStart', { error: String(error) }));
  } finally {
    physicsStarting = false;
    updatePhysicsButton();
  }
}

function stopPhysics() {
  if (!physics) return;
  physics.dispose(!motion); // with a motion, the motion poses the bones again below
  physics = null;
  endModelDrag();
  modelDrag.offset.set(0, 0, 0);
  currentModel?.position.set(0, 0, 0);
  if (motion) {
    motion.setPhysicsEnabled(false);
    motion.update(0); // hair and skirt bones back to the motion's own pose
  }
}

function resetPhysics() {
  if (!physics || !currentModel) return;
  endModelDrag();
  modelDrag.offset.set(0, 0, 0);
  currentModel.position.set(0, 0, 0);
  if (motion) {
    // Keep the motion's pose and start the physics over around it
    motion.update(0);
    physics.realign();
  } else if (mixer && currentAction) {
    // glTF animation: the same, keeping the clip's pose
    mixer.update(0);
    physics.realign();
  } else {
    physics.reset();
  }
}

// Plays the motion, then (in animate) physics runs on the posed bones
function updateMotion(delta: number) {
  if (!motion) return;
  if (playing && !seeking) motion.update(delta);
  // Looped back to the start: the pose jumps, so the bodies are placed again (MMD resets
  // physics at the loop too)
  const time = motion.action.time;
  if (time < lastMotionTime - 1e-6 && !seeking) physics?.realign();
  lastMotionTime = time;
}

// Plays the file's own animation (glTF etc.); physics runs on the posed bones in animate
function updateClip(delta: number) {
  if (!mixer || !currentAction) return;
  if (playing && !seeking) mixer.update(delta);
  const time = currentAction.time;
  if (time < lastClipTime - 1e-6 && !seeking) physics?.realign(); // looped back to the start
  lastClipTime = time;
}

function updatePhysicsButton() {
  const button = document.getElementById('physics-button') as HTMLButtonElement;
  const on = physics !== null;
  button.classList.toggle('active', on);
  document.body.classList.toggle('physics-on', on);
  if (currentLoaded?.physicsBaked) {
    // The animation already contains the physics: it is played as it is
    button.disabled = true;
    button.textContent = t('physics.baked');
    button.title = t('physics.bakedTitle');
    return;
  }
  button.disabled = false;
  button.title = t('physics.title');
  if (!on) physicsTimeText = '';
  button.textContent = physicsStarting ? t('physics.starting') : on ? t('physics.on', { time: physicsTimeText }) : t('physics.off');
}

function updatePhysicsHint() {
  document.getElementById('physics-hint')!.textContent = t('physics.hint', { key: isMac ? 'option' : 'Alt' });
}

// The language was switched in 表示設定: redraw the texts set from code
function refreshTexts() {
  updateMmdButton();
  updatePhysicsButton();
  updatePhysicsHint();
  viewPanel?.refresh();
  if (!motion) {
    const select = document.getElementById('anim-clip') as HTMLSelectElement;
    clips.forEach((clip, i) => {
      if (!clip.name && select.options[i]) select.options[i].text = t('anim.unnamed', { index: i });
    });
  }
  shownFailedCount = -1; // rewrite the "files could not be loaded" notice
  updateFailedNotice();
  if (currentLoaded) currentLoaded.renderInfo(document.getElementById('info-panel')!);
}

function setupModelDrag() {
  const canvas = renderer.domElement;
  const raycaster = new THREE.Raycaster();
  const hit = new THREE.Vector3();

  const pointOnPlane = (e: PointerEvent) => {
    const rect = canvas.getBoundingClientRect();
    const ndc = new THREE.Vector2(
      ((e.clientX - rect.left) / rect.width) * 2 - 1,
      -((e.clientY - rect.top) / rect.height) * 2 + 1,
    );
    raycaster.setFromCamera(ndc, cameraRig.active);
    return raycaster.ray.intersectPlane(modelDrag.plane, hit);
  };

  // Registered in the capture phase so that it runs before OrbitControls' own handler
  canvas.addEventListener(
    'pointerdown',
    (e) => {
      if (!physics || !currentModel || !e.altKey || e.button !== 0) return;
      // Drag on a plane facing the camera through the orbit center
      const normal = cameraRig.active.getWorldDirection(new THREE.Vector3());
      modelDrag.plane.setFromNormalAndCoplanarPoint(normal, controls.target);
      if (!pointOnPlane(e)) return;
      modelDrag.active = true;
      modelDrag.pointerId = e.pointerId;
      modelDrag.start.copy(hit);
      modelDrag.startOffset.copy(modelDrag.offset);
      controls.enabled = false; // OrbitControls ignores this press
      canvas.setPointerCapture(e.pointerId);
      document.body.classList.add('dragging-model');
      e.preventDefault();
    },
    { capture: true },
  );
  canvas.addEventListener('pointermove', (e) => {
    if (!modelDrag.active || e.pointerId !== modelDrag.pointerId) return;
    if (pointOnPlane(e)) modelDrag.offset.copy(modelDrag.startOffset).add(hit).sub(modelDrag.start);
  });
  canvas.addEventListener('pointerup', endModelDrag);
  canvas.addEventListener('pointercancel', endModelDrag);
}

function endModelDrag() {
  if (!modelDrag.active) return;
  modelDrag.active = false;
  controls.enabled = true;
  document.body.classList.remove('dragging-model');
}

function updatePhysics(delta: number) {
  if (!physics || !currentModel) return;
  // While dragged the model follows the pointer; afterwards it springs back to where it was
  if (!modelDrag.active) modelDrag.offset.multiplyScalar(Math.exp(-delta * 8));
  currentModel.position.copy(modelDrag.offset);

  const start = performance.now();
  physics.update(Math.min(delta, MAX_FRAME_TIME)); // after a pause (hidden tab etc.) do not jump ahead
  physicsTime += performance.now() - start;
  physicsFrames++;
}

function clearCurrentModel() {
  if (!currentModel) return;

  stopPhysics();
  motion?.dispose();
  motion = null;
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

  // With physics on, the motion advances exactly as far as the physics does. On a slow frame the
  // physics catches up at most MAX_FRAME_TIME; if the motion went further, the bones would jump
  // and kick the hair (the cause of the hair blow-ups found with the Unity importer). The motion
  // plays a little slower on such a machine instead.
  if (motion) updateMotion(physics ? Math.min(delta, MAX_FRAME_TIME) : delta);
  else updateClip(physics ? Math.min(delta, MAX_FRAME_TIME) : delta);
  // VRM spring bones / expressions
  currentLoaded?.update?.(delta);
  updatePhysics(delta);
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
    // Average physics time per frame, shown on the button (how heavy the model is)
    if (physics && physicsFrames > 0) {
      physicsTimeText = ` (${(physicsTime / physicsFrames).toFixed(1)} ms)`;
      updatePhysicsButton();
      physicsTime = 0;
      physicsFrames = 0;
    }
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
    warn.textContent = t('status.failedFiles', { count: failed.length });
    info.append(warn);
  }
  if (document.body.classList.contains('show-info')) currentLoaded?.renderInfo(document.getElementById('info-panel')!);
}

// Start the app
init();
animate();
