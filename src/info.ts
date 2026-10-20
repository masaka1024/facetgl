import * as THREE from 'three';
import type { GLTF } from 'three/addons/loaders/GLTFLoader.js';
import type { VRMMeta } from '@pixiv/three-vrm';

// --- Summary of the glTF file contents (from the raw JSON + binary data) ---

export interface ModelSource {
  fileName: string;
  json: any;
  glbBin: Uint8Array | null;           // BIN chunk of a .glb
  external: Map<string, Uint8Array>;   // uri -> bytes of external files
  totalBytes: number;
}

const COMPONENT_BYTES: Record<number, number> = { 5120: 1, 5121: 1, 5122: 2, 5123: 2, 5125: 4, 5126: 4 };
const TYPE_SIZE: Record<string, number> = { SCALAR: 1, VEC2: 2, VEC3: 3, VEC4: 4, MAT2: 4, MAT3: 9, MAT4: 16 };

function formatBytes(n: number) {
  if (n >= 1024 * 1024) return (n / 1024 / 1024).toFixed(2) + ' MB';
  if (n >= 1024) return (n / 1024).toFixed(1) + ' KB';
  return n + ' B';
}

function formatCount(n: number) {
  return Math.round(n).toLocaleString();
}

function bufferBytes(src: ModelSource, bufferIndex: number): Uint8Array | null {
  const buffer = src.json.buffers?.[bufferIndex];
  if (!buffer) return null;
  if (buffer.uri === undefined) return src.glbBin;
  return src.external.get(buffer.uri) ?? null;
}

function bufferViewBytes(src: ModelSource, viewIndex: number): Uint8Array | null {
  const view = src.json.bufferViews?.[viewIndex];
  const bytes = view && bufferBytes(src, view.buffer);
  if (!bytes) return null;
  const start = view.byteOffset ?? 0;
  return bytes.subarray(start, start + view.byteLength);
}

function imageSize(b: Uint8Array): [number, number] | null {
  const dv = new DataView(b.buffer, b.byteOffset, b.byteLength);
  if (b.length > 24 && b[0] === 0x89 && b[1] === 0x50) return [dv.getUint32(16), dv.getUint32(20)];
  if (b.length > 4 && b[0] === 0xff && b[1] === 0xd8) {
    let i = 2;
    while (i + 9 < b.length) {
      const marker = b[i + 1];
      if (marker >= 0xc0 && marker <= 0xcf && ![0xc4, 0xc8, 0xcc].includes(marker)) {
        return [dv.getUint16(i + 7), dv.getUint16(i + 5)];
      }
      i += 2 + dv.getUint16(i + 2);
    }
  }
  return null;
}

function accessorBytes(json: any, index: number) {
  const a = json.accessors[index];
  return a.count * (COMPONENT_BYTES[a.componentType] ?? 4) * (TYPE_SIZE[a.type] ?? 1);
}

// --- DOM helpers ---

function el<K extends keyof HTMLElementTagNameMap>(tag: K, text?: string, className?: string) {
  const e = document.createElement(tag);
  if (text !== undefined) e.textContent = text;
  if (className) e.className = className;
  return e;
}

function table(rows: [string, string][]) {
  const t = el('table');
  for (const [k, v] of rows) {
    const tr = el('tr');
    tr.append(el('th', k), el('td', v));
    t.append(tr);
  }
  return t;
}

function section(title: string, body: HTMLElement, open = false) {
  const d = el('details');
  d.open = open;
  d.append(el('summary', title), body);
  return d;
}

function list(header: string[], rows: string[][]) {
  const t = el('table', undefined, 'list');
  const head = el('tr');
  for (const h of header) head.append(el('th', h));
  t.append(head);
  for (const r of rows) {
    const tr = el('tr');
    for (const c of r) tr.append(el('td', c));
    t.append(tr);
  }
  return t;
}

// --- Panel ---

function failedSection(failedFiles: string[]) {
  const box = el('div', undefined, 'warning');
  box.append(el('div', `読み込めなかったファイル (${failedFiles.length})`, 'warning-title'));
  for (const name of failedFiles) box.append(el('div', name));
  return box;
}

export function renderInfoPanel(container: HTMLElement, src: ModelSource, gltf: GLTF, vrmMeta: VRMMeta | undefined, failedFiles: string[]) {
  const json = src.json;
  container.replaceChildren();
  container.append(el('h2', src.fileName));
  if (failedFiles.length > 0) container.append(failedSection(failedFiles));

  // Geometry statistics: primitives may share one vertex buffer, so count unique accessors
  const primitives = (json.meshes ?? []).flatMap((m: any) => m.primitives);
  const positionAccessors = new Set<number>();
  let triangles = 0;
  for (const p of primitives) {
    positionAccessors.add(p.attributes.POSITION);
    const count = p.indices !== undefined ? json.accessors[p.indices].count : json.accessors[p.attributes.POSITION].count;
    if ((p.mode ?? 4) === 4) triangles += count / 3;
  }
  const vertices = [...positionAccessors].reduce((sum, i) => sum + json.accessors[i].count, 0);

  const morphAccessors = new Set<number>(
    primitives.flatMap((p: any) => (p.targets ?? []).flatMap((t: any) => Object.values(t) as number[]))
  );
  const morphBytes = [...morphAccessors].reduce((sum, i) => sum + accessorBytes(json, i), 0);
  const morphNames = morphTargetNames(json, gltf);

  const joints = (json.skins ?? []).reduce((sum: number, s: any) => sum + s.joints.length, 0);

  container.append(section('ファイル', table([
    ['サイズ', formatBytes(src.totalBytes)],
    ['形式', src.glbBin ? 'GLB (バイナリ)' : 'glTF (JSON)'],
    ['glTF バージョン', json.asset?.version ?? '-'],
    ['作成ツール', json.asset?.generator ?? '-'],
    ['著作権', json.asset?.copyright ?? '-'],
    ['使用拡張', (json.extensionsUsed ?? []).join(', ') || 'なし'],
    ['必須拡張', (json.extensionsRequired ?? []).join(', ') || 'なし'],
  ]), true));

  if (vrmMeta) container.append(section('VRM 情報 (利用条件)', vrmMetaTable(vrmMeta), true));
  if (json.extras?.mmd) container.append(section('MMD 情報 (extras.mmd)', mmdExtrasInfo(json), true));

  container.append(section('概要', table([
    ['ノード', formatCount(json.nodes?.length ?? 0)],
    ['メッシュ', formatCount(json.meshes?.length ?? 0)],
    ['プリミティブ (描画単位)', formatCount(primitives.length)],
    ['頂点', formatCount(vertices)],
    ['三角形', formatCount(triangles)],
    ['マテリアル', formatCount(json.materials?.length ?? 0)],
    ['テクスチャ / 画像', `${json.textures?.length ?? 0} / ${json.images?.length ?? 0}`],
    ['スキン / ボーン', `${json.skins?.length ?? 0} / ${joints}`],
    ['モーフ', `${morphNames.length} (${formatBytes(morphBytes)})`],
    ['アニメーション', formatCount(json.animations?.length ?? 0)],
  ]), true));

  const renderStats = el('div');
  renderStats.id = 'render-stats';
  container.append(section('描画状況 (リアルタイム)', renderStats, true));

  container.append(section(`マテリアル (${json.materials?.length ?? 0})`, list(
    ['#', '名前', '種類', '透過', '両面'],
    (json.materials ?? []).map((m: any, i: number) => [
      String(i),
      m.name ?? '',
      m.extensions?.KHR_materials_unlit ? 'Unlit' : 'PBR',
      m.alphaMode ?? 'OPAQUE',
      m.doubleSided ? '○' : '',
    ])
  )));

  let imageTotal = 0;
  const imageRows = (json.images ?? []).map((img: any, i: number) => {
    let bytes: Uint8Array | null = null;
    if (img.bufferView !== undefined) bytes = bufferViewBytes(src, img.bufferView);
    else if (img.uri && !img.uri.startsWith('data:')) bytes = src.external.get(img.uri) ?? null;
    const size = bytes && imageSize(bytes);
    imageTotal += bytes?.length ?? 0;
    return [
      String(i),
      img.name ?? img.uri ?? '',
      size ? `${size[0]}×${size[1]}` : '-',
      bytes ? formatBytes(bytes.length) : '-',
    ];
  });
  container.append(section(`画像 (${imageRows.length}, ${formatBytes(imageTotal)})`, list(
    ['#', '名前', '解像度', 'サイズ'], imageRows
  )));

  container.append(section(`モーフ (${morphNames.length})`, list(
    ['#', '名前'], morphNames.map((n, i) => [String(i), n])
  )));

  container.append(section(`アニメーション (${gltf.animations.length})`, list(
    ['#', '名前', '長さ', 'トラック'],
    gltf.animations.map((clip, i) => [String(i), clip.name, clip.duration.toFixed(2) + ' 秒', String(clip.tracks.length)])
  )));

  container.append(section(`ノード階層 (${json.nodes?.length ?? 0})`, nodeTree(json)));
}

function morphTargetNames(json: any, gltf: GLTF): string[] {
  for (const mesh of json.meshes ?? []) {
    if (Array.isArray(mesh.extras?.targetNames)) return mesh.extras.targetNames;
  }
  let names: string[] = [];
  gltf.scene.traverse((child) => {
    const dict = (child as THREE.Mesh).morphTargetDictionary;
    if (names.length === 0 && dict) names = Object.keys(dict);
  });
  return names;
}

function nodeTree(json: any) {
  const root = el('div', undefined, 'tree');
  const sceneIndex = json.scene ?? 0;
  const rootNodes: number[] = json.scenes?.[sceneIndex]?.nodes ?? [];

  const build = (index: number, depth: number): HTMLElement => {
    const node = json.nodes[index];
    const tags = [
      node.mesh !== undefined ? 'mesh' : '',
      node.skin !== undefined ? 'skin' : '',
      node.camera !== undefined ? 'camera' : '',
    ].filter(Boolean).join(' ');
    const label = `${node.name ?? `node ${index}`}${tags ? `  [${tags}]` : ''}`;
    const children: number[] = node.children ?? [];
    if (children.length === 0) return el('div', label, 'leaf');

    const d = el('details');
    d.open = depth < 2;
    d.append(el('summary', `${label}  (${children.length})`));
    for (const c of children) d.append(build(c, depth + 1));
    return d;
  };

  for (const i of rootNodes) root.append(build(i, 0));
  return root;
}

// glTF converted from PMX by mmd2gltf / mmd2gltf-cs keeps the MMD data in extras.mmd
function mmdExtrasInfo(json: any) {
  const mmd = json.extras.mmd;
  const count = (v: unknown) => (Array.isArray(v) ? formatCount(v.length) : '-');
  const materials = json.materials ?? [];
  const mmdMaterials = materials.filter((m: any) => m.extras?.mmd).length;
  const mmdBones = (json.nodes ?? []).filter((n: any) => n.extras?.mmd).length;

  const box = el('div');
  box.append(table([
    ['表示', 'MMD 材質 (トゥーン・スフィア・エッジ) で表示可能。上部の「MMD表示」で切替'],
    ['変換ツール', json.asset?.generator ?? '-'],
    ['モデル名', mmd.name ?? '-'],
    ['英語名', mmd.nameEn || '-'],
    ['元形式', `${String(mmd.format ?? '-').toUpperCase()} ${mmd.version ?? ''}`.trim()],
    ['unitScale', String(mmd.unitScale ?? '-')],
    ['MMD 材質', `${mmdMaterials} / ${materials.length}`],
    ['MMD ボーン', formatCount(mmdBones)],
    ['モーフ', count(mmd.morphs)],
    ['表示枠', count(mmd.displayFrames)],
    ['剛体 / ジョイント', `${count(mmd.rigidBodies)} / ${count(mmd.joints)}`],
    ['physicsGltf', mmd.physicsGltf ? 'あり' : 'なし'],
  ]));
  if (mmd.comment) box.append(el('div', mmd.comment.replace(/\r\n/g, '\n'), 'comment'));
  return box;
}

// VRM 0.x and 1.0 use different meta fields, so list every simple value as-is
function vrmMetaTable(meta: VRMMeta) {
  const rows: [string, string][] = [['VRM バージョン', meta.metaVersion === '1' ? '1.0' : '0.x']];
  for (const [key, value] of Object.entries(meta)) {
    if (key === 'metaVersion') continue;
    if (typeof value === 'string' || typeof value === 'number' || typeof value === 'boolean') {
      rows.push([key, String(value)]);
    } else if (Array.isArray(value) && value.every((v) => typeof v === 'string')) {
      rows.push([key, value.join(', ')]);
    }
  }
  return table(rows);
}

// --- Panel for non-glTF formats (built from the loaded three.js scene) ---

export interface SceneSource {
  fileName: string;
  format: string;
  totalBytes: number;
  object: THREE.Object3D;
  animations: THREE.AnimationClip[];
  failedFiles: string[];
}

export function renderSceneInfoPanel(container: HTMLElement, src: SceneSource) {
  container.replaceChildren();
  container.append(el('h2', src.fileName));
  if (src.failedFiles.length > 0) container.append(failedSection(src.failedFiles));

  const meshes: THREE.Mesh[] = [];
  const geometries = new Set<THREE.BufferGeometry>();
  const materials = new Set<THREE.Material>();
  const textures = new Set<THREE.Texture>();
  const bones = new Set<THREE.Bone>();
  let nodes = 0;
  let morphNames: string[] = [];

  src.object.traverse((child) => {
    nodes++;
    const mesh = child as THREE.Mesh;
    if (!mesh.isMesh) return;
    meshes.push(mesh);
    geometries.add(mesh.geometry);
    for (const material of Array.isArray(mesh.material) ? mesh.material : [mesh.material]) {
      materials.add(material);
      for (const value of Object.values(material)) {
        if (value instanceof THREE.Texture) textures.add(value);
      }
    }
    (mesh as THREE.SkinnedMesh).skeleton?.bones.forEach((b) => bones.add(b));
    const dict = mesh.morphTargetDictionary;
    if (dict && Object.keys(dict).length > morphNames.length) morphNames = Object.keys(dict);
  });

  let vertices = 0;
  let triangles = 0;
  for (const g of geometries) {
    const count = g.getAttribute('position')?.count ?? 0;
    vertices += count;
    triangles += (g.index ? g.index.count : count) / 3;
  }

  container.append(section('ファイル', table([
    ['サイズ', formatBytes(src.totalBytes)],
    ['形式', src.format],
  ]), true));

  container.append(section('概要', table([
    ['ノード', formatCount(nodes)],
    ['メッシュ', formatCount(meshes.length)],
    ['頂点', formatCount(vertices)],
    ['三角形', formatCount(triangles)],
    ['マテリアル', formatCount(materials.size)],
    ['テクスチャ', formatCount(textures.size)],
    ['ボーン', formatCount(bones.size)],
    ['モーフ', formatCount(morphNames.length)],
    ['アニメーション', formatCount(src.animations.length)],
  ]), true));

  const renderStats = el('div');
  renderStats.id = 'render-stats';
  container.append(section('描画状況 (リアルタイム)', renderStats, true));

  container.append(section(`マテリアル (${materials.size})`, list(
    ['#', '名前', '種類', '透過', '両面'],
    [...materials].map((m, i) => [
      String(i),
      m.name,
      m.type.replace(/Material$/, ''),
      m.transparent ? '○' : '',
      m.side === THREE.DoubleSide ? '○' : '',
    ])
  )));

  container.append(section(`テクスチャ (${textures.size})`, list(
    ['#', '名前', '解像度'],
    [...textures].map((t, i) => {
      const image = t.image as { width?: number; height?: number; src?: string } | undefined;
      const name = t.name || (image?.src ? decodeURIComponent(image.src.split('/').pop() ?? '') : '');
      return [String(i), name, image?.width ? `${image.width}×${image.height}` : '読み込み中'];
    })
  )));

  container.append(section(`モーフ (${morphNames.length})`, list(
    ['#', '名前'], morphNames.map((n, i) => [String(i), n])
  )));

  container.append(section(`アニメーション (${src.animations.length})`, list(
    ['#', '名前', '長さ', 'トラック'],
    src.animations.map((clip, i) => [String(i), clip.name, clip.duration.toFixed(2) + ' 秒', String(clip.tracks.length)])
  )));

  container.append(section(`ノード階層 (${nodes})`, objectTree(src.object)));
}

function objectTree(object: THREE.Object3D) {
  const root = el('div', undefined, 'tree');

  const build = (obj: THREE.Object3D, depth: number): HTMLElement => {
    const tag = (obj as THREE.Bone).isBone ? 'bone' : (obj as THREE.Mesh).isMesh ? 'mesh' : '';
    const label = `${obj.name || obj.type}${tag ? `  [${tag}]` : ''}`;
    if (obj.children.length === 0) return el('div', label, 'leaf');

    const d = el('details');
    d.open = depth < 2;
    d.append(el('summary', `${label}  (${obj.children.length})`));
    for (const c of obj.children) d.append(build(c, depth + 1));
    return d;
  };

  root.append(build(object, 0));
  return root;
}

export function updateRenderStats(renderer: THREE.WebGLRenderer) {
  const target = document.getElementById('render-stats');
  if (!target || !target.isConnected) return;
  const info = renderer.info;
  target.replaceChildren(table([
    ['ドローコール', formatCount(info.render.calls)],
    ['描画三角形', formatCount(info.render.triangles)],
    ['GPU ジオメトリ', formatCount(info.memory.geometries)],
    ['GPU テクスチャ', formatCount(info.memory.textures)],
  ]));
}
