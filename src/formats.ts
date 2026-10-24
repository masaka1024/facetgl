import * as THREE from 'three';
import { GLTFLoader } from 'three/addons/loaders/GLTFLoader.js';
import type { DRACOLoader } from 'three/addons/loaders/DRACOLoader.js';
import type { KTX2Loader } from 'three/addons/loaders/KTX2Loader.js';
import { MeshoptDecoder } from 'three/addons/libs/meshopt_decoder.module.js';
import { MMDLoader } from 'three/addons/loaders/MMDLoader.js';
import { FBXLoader } from 'three/addons/loaders/FBXLoader.js';
import { OBJLoader } from 'three/addons/loaders/OBJLoader.js';
import { MTLLoader } from 'three/addons/loaders/MTLLoader.js';
import { STLLoader } from 'three/addons/loaders/STLLoader.js';
import { TGALoader } from 'three/addons/loaders/TGALoader.js';
import { VRMLoaderPlugin, VRMUtils, type VRM } from '@pixiv/three-vrm';
import { assetUrl, fileSize, listFiles, readFile, resolveUrl, textureSearchRoots } from './platform';
import { renderInfoPanel, renderSceneInfoPanel, type ModelSource } from './info';
import { applyMmdShading, buildMmdExtrasMaterials, type MmdExtrasMaterials } from './mmdShading';

export const FORMATS = [
  { name: 'glTF / GLB', extensions: ['glb', 'gltf'] },
  { name: 'VRM', extensions: ['vrm'] },
  { name: 'MMD (PMX / PMD)', extensions: ['pmx', 'pmd'] },
  { name: 'FBX', extensions: ['fbx'] },
  { name: 'OBJ', extensions: ['obj'] },
  { name: 'STL', extensions: ['stl'] },
];

export const MODEL_EXTENSIONS = FORMATS.flatMap((f) => f.extensions);

export interface LoaderContext {
  dracoLoader: DRACOLoader;
  ktx2Loader: KTX2Loader;
}

export interface LoadedModel {
  object: THREE.Object3D;
  animations: THREE.AnimationClip[];
  blobUrls: string[];
  // Textures etc. that failed to load; filled in as loading finishes in the background
  failedFiles: string[];
  renderInfo: (container: HTMLElement) => void;
  update?: (delta: number) => void;
  // 'mmd' switches to MMD-style lighting and edge outlines
  style?: 'mmd';
  // glTF converted from PMX with extras.mmd (mmd2gltf): MMD materials that can be toggled
  mmdExtras?: MmdExtrasMaterials;
  // PMX: the raw file, read again when physics is turned on (the physics engine parses the
  // rigid bodies and joints itself, in MMD's own coordinates)
  pmxSource?: () => Promise<ArrayBuffer>;
}

function trackFailures(manager: THREE.LoadingManager, failedFiles: string[], nameOf: (url: string) => string) {
  manager.onError = (url) => {
    const name = nameOf(url);
    if (!failedFiles.includes(name)) failedFiles.push(name);
  };
}

function urlFileName(url: string) {
  const last = url.split(/[\\/]/).pop() ?? url;
  try {
    return decodeURIComponent(last);
  } catch {
    return last;
  }
}

export function fileExtension(path: string) {
  return path.split('.').pop()?.toLowerCase() ?? '';
}

export function loadModelFile(path: string, ctx: LoaderContext): Promise<LoadedModel> {
  switch (fileExtension(path)) {
    case 'glb':
    case 'gltf':
    case 'vrm':
      return loadGltf(path, ctx);
    case 'pmx':
    case 'pmd':
      return loadMmd(path);
    case 'fbx':
      return loadFbx(path);
    case 'obj':
      return loadObj(path);
    case 'stl':
      return loadStl(path);
    default:
      return Promise.reject(new Error('対応していない形式です。'));
  }
}

// --- File access (desktop or web, see platform.ts) ---

function splitPath(path: string) {
  const fileName = path.split(/[\\/]/).pop() ?? path;
  return { fileName, dir: path.slice(0, path.length - fileName.length - 1) };
}

// Uses the separator of the folder path (Windows paths on desktop, "/" on the web)
function joinPath(dir: string, relative: string) {
  const separator = dir.includes('\\') ? '\\' : '/';
  return dir + separator + relative.replace(/[\\/]/g, separator);
}

function assetDirUrl(path: string) {
  return assetUrl(splitPath(path).dir) + '/';
}

// --- Texture lookup for formats that reference textures by file name ---

interface FolderIndex {
  known: Map<string, string>;       // every file found: URL key (see urlKey) -> its real URL
  byName: Map<string, string>;      // file name (lower case) -> URL of the nearest such file
}

// Compares file URLs however a model spells them: decoded, "/" separators, "." / ".."
// resolved, Unicode-normalized, case-insensitive
function urlKey(url: string) {
  let path = url;
  try {
    path = decodeURIComponent(new URL(url).pathname);
  } catch {
    // Keep the raw string if it is not a valid URL or contains a stray "%"
  }
  const out: string[] = [];
  for (const part of path.replace(/\\/g, '/').split('/')) {
    if (part === '' || part === '.') continue;
    if (part === '..') out.pop();
    else out.push(part);
  }
  // macOS may give file names in decomposed form ("か" + "゛"), while models store "が"
  return out.join('/').normalize('NFC').toLowerCase();
}

// Indexes the model's folder and up to 3 parent folders (each 3 levels deep).
// Files nearer the model come first: its own folder, shallow before deep, then the parents.
async function indexFolder(path: string): Promise<FolderIndex> {
  const roots = await textureSearchRoots(splitPath(path).dir);
  const lists = await Promise.all(roots.map((root) => listFiles(root)));
  const known = new Map<string, string>();
  const byName = new Map<string, string>();
  roots.forEach((root, i) => {
    for (const rel of [...lists[i]].sort((a, b) => a.split('/').length - b.split('/').length)) {
      const url = assetUrl(joinPath(root, rel));
      const key = urlKey(url);
      if (known.has(key)) continue; // already found from a nearer folder
      known.set(key, url);
      const name = rel.split('/').pop()!.normalize('NFC').toLowerCase();
      if (!byName.has(name)) byName.set(name, url);
    }
  });
  return { known, byName };
}

// Sends every texture to the URL of the real file:
// - Paths written with "\\" (fine on Windows, but part of the file name on macOS), "..", or
//   different upper/lower case are mapped to the file they mean.
// - Textures that were moved (e.g. into "tex/" or a parent folder) or that point at the
//   author's absolute paths go to the nearest file with the same name.
function textureManager(index: FolderIndex, failedFiles: string[]) {
  const assetBase = assetUrl('');
  const manager = new THREE.LoadingManager();
  trackFailures(manager, failedFiles, urlFileName);
  const redirect = (url: string) => {
    if (!url.startsWith(assetBase)) return url;
    const key = urlKey(url);
    return index.known.get(key) ?? index.byName.get(key.split('/').pop()!) ?? url;
  };
  manager.setURLModifier((url) => resolveUrl(redirect(url)));
  manager.addHandler(/\.tga$/i, new TGALoader(manager));
  return manager;
}

// --- glTF / GLB / VRM ---

// Extracts the glTF JSON (and the BIN chunk of a .glb) from the loaded file
function parseGltfContainer(buffer: ArrayBuffer): { json: any; glbBin: Uint8Array | null } {
  const view = new DataView(buffer);
  const isGlb = buffer.byteLength >= 20 && view.getUint32(0, true) === 0x46546c67; // 'glTF'
  if (isGlb) {
    const jsonLength = view.getUint32(12, true);
    const json = JSON.parse(new TextDecoder().decode(new Uint8Array(buffer, 20, jsonLength)));
    const binHeader = 20 + jsonLength;
    const glbBin = binHeader + 8 <= buffer.byteLength
      ? new Uint8Array(buffer, binHeader + 8, view.getUint32(binHeader, true))
      : null;
    return { json, glbBin };
  }
  return { json: JSON.parse(new TextDecoder().decode(buffer)), glbBin: null };
}

// Reads every external .bin / texture next to the model
async function loadExternalResources(json: any, dir: string) {
  const uris = new Set<string>();
  for (const item of [...(json.buffers ?? []), ...(json.images ?? [])]) {
    if (typeof item.uri === 'string' && !item.uri.startsWith('data:')) {
      uris.add(item.uri);
    }
  }

  const blobUrls = new Map<string, string>();
  const bytes = new Map<string, Uint8Array>();
  await Promise.all(Array.from(uris, async (uri) => {
    const data = await readFile(joinPath(dir, decodeURIComponent(uri)));
    blobUrls.set(uri, URL.createObjectURL(new Blob([data])));
    bytes.set(uri, new Uint8Array(data));
  }));
  return { blobUrls, bytes };
}

async function loadGltf(path: string, ctx: LoaderContext): Promise<LoadedModel> {
  const { fileName, dir } = splitPath(path);
  const buffer = await readFile(path);
  const { json, glbBin } = parseGltfContainer(buffer);
  const external = await loadExternalResources(json, dir);
  const blobUrls = external.blobUrls;

  try {
    const failedFiles: string[] = [];
    const uriOfBlob = new Map([...blobUrls].map(([uri, blob]) => [blob, uri]));
    const manager = new THREE.LoadingManager();
    manager.setURLModifier((url) => blobUrls.get(url) ?? url);
    trackFailures(manager, failedFiles, (url) => uriOfBlob.get(url) ?? urlFileName(url));

    const loader = new GLTFLoader(manager)
      .setDRACOLoader(ctx.dracoLoader)
      .setKTX2Loader(ctx.ktx2Loader)
      .setMeshoptDecoder(MeshoptDecoder);
    // Only acts on files that contain VRM extensions
    loader.register((parser) => new VRMLoaderPlugin(parser));

    const gltf = await loader.parseAsync(buffer, '');
    const vrm = gltf.userData.vrm as VRM | undefined;
    // VRM 0.x models face -Z; turn them toward the camera like VRM 1.0
    if (vrm) VRMUtils.rotateVRM0(vrm);

    // Files from mmd2gltf keep the PMX materials in extras.mmd; show them the MMD way
    const mmdExtras = await buildMmdExtrasMaterials(gltf);
    mmdExtras?.setEnabled(true);

    const source: ModelSource = {
      fileName,
      json,
      glbBin,
      external: external.bytes,
      totalBytes: buffer.byteLength + [...external.bytes.values()].reduce((sum, b) => sum + b.length, 0),
    };

    return {
      object: gltf.scene,
      animations: gltf.animations,
      blobUrls: [...blobUrls.values()],
      failedFiles,
      renderInfo: (container) => renderInfoPanel(container, source, gltf, vrm?.meta, failedFiles),
      update: vrm ? (delta) => vrm.update(delta) : undefined,
      style: mmdExtras ? 'mmd' : undefined,
      mmdExtras: mmdExtras ?? undefined,
    };
  } catch (error) {
    blobUrls.forEach((u) => URL.revokeObjectURL(u));
    throw error;
  }
}

// --- Other formats (loaded through the asset protocol so relative textures resolve) ---

async function sceneModel(
  path: string,
  format: string,
  object: THREE.Object3D,
  animations: THREE.AnimationClip[],
  failedFiles: string[],
): Promise<LoadedModel> {
  const totalBytes = await fileSize(path);
  const { fileName } = splitPath(path);
  return {
    object,
    animations,
    blobUrls: [],
    failedFiles,
    renderInfo: (container) => renderSceneInfoPanel(container, { fileName, format, totalBytes, object, animations, failedFiles }),
  };
}

async function loadMmd(path: string): Promise<LoadedModel> {
  const failedFiles: string[] = [];
  const manager = textureManager(await indexFolder(path), failedFiles);
  const mesh = await new MMDLoader(manager).loadAsync(assetUrl(path));
  applyMmdShading(mesh);
  const isPmx = fileExtension(path) === 'pmx';
  const model = await sceneModel(path, isPmx ? 'PMX (MMD)' : 'PMD (MMD)', mesh, [], failedFiles);
  return { ...model, style: 'mmd', pmxSource: isPmx ? () => readFile(path) : undefined };
}

async function loadFbx(path: string) {
  const failedFiles: string[] = [];
  const index = await indexFolder(path);
  const buffer = prepareFbx(await readFile(path), index);
  const group = new FBXLoader(textureManager(index, failedFiles)).parse(buffer, assetDirUrl(path));
  fixSelfIlluminatedMaterials(group);
  return sceneModel(path, 'FBX', group, group.animations, failedFiles);
}

// 3ds Max exports often set self-illumination to 100% (EmissiveColor 1,1,1) for a flat,
// unlit look. three.js would then render the mesh pure white, so emit the texture instead.
function fixSelfIlluminatedMaterials(root: THREE.Object3D) {
  root.traverse((child) => {
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      const m = material as THREE.MeshPhongMaterial;
      if (!m.map || !m.emissive || m.emissiveMap) continue;
      if (m.emissive.r + m.emissive.g + m.emissive.b < 2.7) continue;
      m.emissiveMap = m.map;
      m.color.setRGB(0, 0, 0);
      m.needsUpdate = true;
    }
  });
}

const FBX_BINARY_MAGIC = 'Kaydara FBX Binary';
const BROWSER_IMAGE_EXTENSIONS = ['png', 'tga', 'jpg', 'jpeg', 'bmp'];

function prepareFbx(buffer: ArrayBuffer, index: FolderIndex): ArrayBuffer {
  const bytes = new Uint8Array(buffer);
  replacePsdReferences(bytes, index);

  const isBinary = new TextDecoder().decode(bytes.subarray(0, FBX_BINARY_MAGIC.length)) === FBX_BINARY_MAGIC;
  if (isBinary) return buffer;
  return new TextEncoder().encode(reindentAsciiFbx(new TextDecoder().decode(bytes))).buffer;
}

// Browsers cannot decode .psd. Point those references at a same-named image that exists
// in the folder. The extension keeps its length, so binary FBX string sizes stay valid.
function replacePsdReferences(bytes: Uint8Array, index: FolderIndex) {
  const isDelimiter = (b: number) => b < 0x20 || b === 0x22 || b === 0x2f || b === 0x5c; // control, " / \
  for (let i = 0; i + 4 <= bytes.length; i++) {
    if (bytes[i] !== 0x2e) continue; // '.'
    const ext = String.fromCharCode(bytes[i + 1], bytes[i + 2], bytes[i + 3]).toLowerCase();
    if (ext !== 'psd' || (i + 4 < bytes.length && !isDelimiter(bytes[i + 4]))) continue;

    let start = i;
    while (start > 0 && !isDelimiter(bytes[start - 1])) start--;
    const stem = new TextDecoder().decode(bytes.subarray(start, i)).toLowerCase();
    const replacement = BROWSER_IMAGE_EXTENSIONS.find((e) => e.length === 3 && index.byName.has(`${stem}.${e}`));
    if (replacement) {
      for (let k = 0; k < 3; k++) bytes[i + 1 + k] = replacement.charCodeAt(k);
    }
  }
}

// three.js's ASCII FBX parser relies on tab indentation. Some exporters indent with spaces
// or put array data at column 0, so rebuild the indentation from the brace nesting.
function reindentAsciiFbx(text: string) {
  let depth = 0;
  return text.split(/\r?\n/).map((line) => {
    const t = line.trim();
    if (t === '' || t.startsWith(';')) return t;
    if (t.startsWith('}')) {
      depth = Math.max(0, depth - 1);
      return '\t'.repeat(depth) + t;
    }
    if (/^\w+:/.test(t)) {
      const indented = '\t'.repeat(depth) + t;
      if (t.endsWith('{')) depth++;
      return indented;
    }
    return t; // continuation line of a long array
  }).join('\n');
}

async function loadObj(path: string) {
  const text = new TextDecoder().decode(await readFile(path));
  const objLoader = new OBJLoader();
  const failedFiles: string[] = [];

  // Materials and textures come from the .mtl file referenced by "mtllib"
  const mtlName = /^mtllib\s+(.+?)\s*$/m.exec(text)?.[1];
  if (mtlName) {
    try {
      const dirUrl = assetDirUrl(path);
      const manager = textureManager(await indexFolder(path), failedFiles);
      const mtlLoader = new MTLLoader(manager).setPath(dirUrl).setResourcePath(dirUrl);
      const materials = await mtlLoader.loadAsync(mtlName.split(/[\\/]/).map(encodeURIComponent).join('/'));
      materials.preload();
      objLoader.setMaterials(materials);
    } catch (error) {
      console.warn('MTL could not be loaded:', mtlName, error);
      if (!failedFiles.includes(mtlName)) failedFiles.push(mtlName);
    }
  }

  return sceneModel(path, 'OBJ', objLoader.parse(text), [], failedFiles);
}

async function loadStl(path: string) {
  const geometry = new STLLoader().parse(await readFile(path)) as THREE.BufferGeometry & { hasColors?: boolean };
  const material = new THREE.MeshStandardMaterial({
    color: geometry.hasColors ? 0xffffff : 0xb8c0cc,
    vertexColors: !!geometry.hasColors,
    metalness: 0.1,
    roughness: 0.6,
  });
  const mesh = new THREE.Mesh(geometry, material);
  mesh.name = splitPath(path).fileName;
  // STL files from CAD / 3D printing are usually Z-up
  mesh.rotation.x = -Math.PI / 2;

  const group = new THREE.Group();
  group.add(mesh);
  return sceneModel(path, 'STL', group, [], []);
}
