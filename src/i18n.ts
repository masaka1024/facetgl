// UI language (Japanese / English).
//
// The language follows the OS / browser language (Japanese for "ja", English otherwise) unless
// the user picks one in 表示設定 / View. Texts in index.html carry data-i18n (text) and
// data-i18n-title (tooltip) keys; code calls t(). Parameters are written as {name}.

const ja = {
  // Drop zone and links
  'drop.app': '3D モデルのファイル (またはフォルダ) をドロップ、または「開く」から選択してください',
  'drop.appNote': 'MMD モデルを開いたあとにモーション (VMD) をドロップすると再生できます',
  'drop.web': '3D モデルのファイル (またはフォルダ) をドロップしてください',
  'drop.webNote':
    'PMX / FBX / OBJ や外部テクスチャを使う .gltf はフォルダごとドロップ。MMD モデルを開いたあとにモーション (VMD) をドロップすると再生できます。ファイルはブラウザ内だけで読み込まれ、送信されません',
  'link.project': 'FacetGL on GitHub (デスクトップ版)',

  // Toolbar
  'toolbar.open': '開く',
  'toolbar.openFolder': 'フォルダを開く',
  'toolbar.openFolderTitle': 'PMX などテクスチャが別ファイルのモデル向け',
  'toolbar.info': '情報',
  'toolbar.view': '表示設定',
  'mmd.on': 'MMD表示: ON',
  'mmd.off': 'MMD表示: OFF',
  'mmd.title': 'extras.mmd（PMX の材質情報）を検出しました。クリックで MMD 表示と glTF 標準表示を切り替えます',
  'physics.off': '物理: OFF',
  'physics.on': '物理: ON{time}',
  'physics.starting': '物理: 準備中…',
  'physics.baked': '物理: ベイク済み',
  'physics.title': 'F: 髪やスカートの剛体を物理演算で動かします (試験機能)',
  'physics.bakedTitle': 'このファイルのアニメーションには物理演算の結果が焼き込まれているため、そちらを優先して再生しています',
  'physics.reset': '物理リセット',
  'physics.resetTitle': 'R: 姿勢を初期状態に戻して物理をやり直します',
  'physics.hint': '{key} + ドラッグでモデルを動かすと、髪やスカートが揺れます',

  // 表示設定 panel
  'view.edge': '輪郭線の太さ',
  'view.edgeNote': 'MMD 表示のとき。エッジサイズ 1.0 あたりの線の幅 (MMD 相当は 1.0 px)',
  'view.edgeOff': 'なし',
  'view.projection': '投影',
  'view.projectionKey': 'P キーで切替',
  'view.perspective': '透視',
  'view.orthographic': '平行',
  'view.fov': '視野角',
  'view.fovNote': '透視のとき。MMD の初期値は 30°',
  'view.language': '言語 (Language)',
  'view.languageAuto': '自動',
  'view.reset': '初期値に戻す',

  // Animation bar
  'anim.speed': '再生速度',
  'anim.unnamed': 'アニメーション {index}',

  // Status line and messages
  'status.loading': '読み込み中: {name}',
  'status.loadingMotion': 'モーション読み込み中: {name}',
  'status.failedFiles': '  ⚠ 読み込めないファイル {count} 件 (詳細は「情報」)',
  'dialog.allFormats': '対応しているすべての形式',
  'dialog.motion': 'MMD モーション (VMD)',
  'error.motionNeedsModel': 'モーション (VMD) は、先に PMX / PMD モデルを開いてから読み込んでください。',
  'error.noSupportedFile': '対応している形式のファイルが見つかりませんでした。\n({extensions})',
  'error.motionLoad': 'モーションの読み込みに失敗しました。\n{name}\n\n{error}',
  'error.modelLoad': 'モデルの読み込みに失敗しました。\n{name}\n\n{error}',
  'error.physicsStart': '物理演算を開始できませんでした。\n\n{error}',
  'error.unsupportedFormat': '対応していない形式です。',
  'error.fileNotFound': '{path}: ファイルが見つかりません (フォルダごと読み込むと解決することがあります)',
  'error.noPhysics': 'このモデルは物理演算に対応していません。',
  'error.boneCount': 'ボーン数が一致しません (PMX {pmx} / 表示 {shown})',
  'error.motionNoTargets': 'このモデルで動かせるボーンやモーフがモーションに含まれていません。',

  // Info panel
  'info.failedFiles': '読み込めなかったファイル ({count})',
  'info.file': 'ファイル',
  'info.size': 'サイズ',
  'info.format': '形式',
  'info.formatGlb': 'GLB (バイナリ)',
  'info.formatGltf': 'glTF (JSON)',
  'info.gltfVersion': 'glTF バージョン',
  'info.generator': '作成ツール',
  'info.copyright': '著作権',
  'info.extensionsUsed': '使用拡張',
  'info.extensionsRequired': '必須拡張',
  'info.none': 'なし',
  'info.yes': 'あり',
  'info.vrm': 'VRM 情報 (利用条件)',
  'info.vrmVersion': 'VRM バージョン',
  'info.mmd': 'MMD 情報 (extras.mmd)',
  'info.summary': '概要',
  'info.nodes': 'ノード',
  'info.meshes': 'メッシュ',
  'info.primitives': 'プリミティブ (描画単位)',
  'info.vertices': '頂点',
  'info.triangles': '三角形',
  'info.materials': 'マテリアル',
  'info.texturesImages': 'テクスチャ / 画像',
  'info.textures': 'テクスチャ',
  'info.skinsBones': 'スキン / ボーン',
  'info.bones': 'ボーン',
  'info.morphs': 'モーフ',
  'info.animations': 'アニメーション',
  'info.renderStats': '描画状況 (リアルタイム)',
  'info.materialsCount': 'マテリアル ({count})',
  'info.texturesCount': 'テクスチャ ({count})',
  'info.imagesCount': '画像 ({count}, {size})',
  'info.morphsCount': 'モーフ ({count})',
  'info.animationsCount': 'アニメーション ({count})',
  'info.nodeTree': 'ノード階層 ({count})',
  'info.name': '名前',
  'info.type': '種類',
  'info.alpha': '透過',
  'info.doubleSided': '両面',
  'info.resolution': '解像度',
  'info.duration': '長さ',
  'info.tracks': 'トラック',
  'info.seconds': '{value} 秒',
  'info.textureLoading': '読み込み中',
  'info.mmdDisplay': '表示',
  'info.mmdDisplayValue': 'MMD 材質 (トゥーン・スフィア・エッジ) で表示可能。上部の「MMD表示」で切替',
  'info.converter': '変換ツール',
  'info.modelName': 'モデル名',
  'info.englishName': '英語名',
  'info.originalFormat': '元形式',
  'info.mmdMaterials': 'MMD 材質',
  'info.mmdBones': 'MMD ボーン',
  'info.displayFrames': '表示枠',
  'info.rigidBodiesJoints': '剛体 / ジョイント',
  'info.drawCalls': 'ドローコール',
  'info.drawnTriangles': '描画三角形',
  'info.gpuGeometries': 'GPU ジオメトリ',
  'info.gpuTextures': 'GPU テクスチャ',
};

export type TextKey = keyof typeof ja;

const en: Record<TextKey, string> = {
  'drop.app': 'Drop a 3D model file (or folder) here, or choose one with “Open”',
  'drop.appNote': 'After opening an MMD model, drop a motion (VMD) to play it',
  'drop.web': 'Drop a 3D model file (or folder) here',
  'drop.webNote':
    'For PMX / FBX / OBJ, and .gltf files with external textures, drop the whole folder. After opening an MMD model, drop a motion (VMD) to play it. Files are loaded only inside your browser and are never sent anywhere',
  'link.project': 'FacetGL on GitHub (desktop version)',

  'toolbar.open': 'Open',
  'toolbar.openFolder': 'Open folder',
  'toolbar.openFolderTitle': 'For models whose textures are separate files, such as PMX',
  'toolbar.info': 'Info',
  'toolbar.view': 'View',
  'mmd.on': 'MMD view: ON',
  'mmd.off': 'MMD view: OFF',
  'mmd.title': 'extras.mmd (PMX material data) was found. Click to switch between MMD rendering and standard glTF rendering',
  'physics.off': 'Physics: OFF',
  'physics.on': 'Physics: ON{time}',
  'physics.starting': 'Physics: starting…',
  'physics.baked': 'Physics: Baked',
  'physics.title': 'F: Move the rigid bodies of hair and skirts with physics (experimental)',
  'physics.bakedTitle': 'The animation in this file already contains baked physics results, so it is played as it is',
  'physics.reset': 'Reset physics',
  'physics.resetTitle': 'R: Return to the initial pose and start the physics over',
  'physics.hint': '{key} + drag to move the model and make hair and skirts sway',

  'view.edge': 'Outline width',
  'view.edgeNote': 'In MMD view. Line width per 1.0 of edge size (1.0 px matches MMD)',
  'view.edgeOff': 'Off',
  'view.projection': 'Projection',
  'view.projectionKey': 'Toggle with P',
  'view.perspective': 'Perspective',
  'view.orthographic': 'Orthographic',
  'view.fov': 'Field of view',
  'view.fovNote': 'In perspective. MMD’s default is 30°',
  'view.language': 'Language (言語)',
  'view.languageAuto': 'Auto',
  'view.reset': 'Reset to defaults',

  'anim.speed': 'Playback speed',
  'anim.unnamed': 'Animation {index}',

  'status.loading': 'Loading: {name}',
  'status.loadingMotion': 'Loading motion: {name}',
  'status.failedFiles': '  ⚠ {count} file(s) could not be loaded (see “Info”)',
  'dialog.allFormats': 'All supported formats',
  'dialog.motion': 'MMD motion (VMD)',
  'error.motionNeedsModel': 'To load a motion (VMD), open a PMX / PMD model first.',
  'error.noSupportedFile': 'No file in a supported format was found.\n({extensions})',
  'error.motionLoad': 'Failed to load the motion.\n{name}\n\n{error}',
  'error.modelLoad': 'Failed to load the model.\n{name}\n\n{error}',
  'error.physicsStart': 'Could not start physics.\n\n{error}',
  'error.unsupportedFormat': 'This format is not supported.',
  'error.fileNotFound': '{path}: file not found (opening the whole folder may solve this)',
  'error.noPhysics': 'This model does not support physics.',
  'error.boneCount': 'The bone counts do not match (PMX {pmx} / shown {shown})',
  'error.motionNoTargets': 'The motion contains no bones or morphs that this model can move.',

  'info.failedFiles': 'Files that could not be loaded ({count})',
  'info.file': 'File',
  'info.size': 'Size',
  'info.format': 'Format',
  'info.formatGlb': 'GLB (binary)',
  'info.formatGltf': 'glTF (JSON)',
  'info.gltfVersion': 'glTF version',
  'info.generator': 'Generator',
  'info.copyright': 'Copyright',
  'info.extensionsUsed': 'Extensions used',
  'info.extensionsRequired': 'Extensions required',
  'info.none': 'None',
  'info.yes': 'Yes',
  'info.vrm': 'VRM info (license terms)',
  'info.vrmVersion': 'VRM version',
  'info.mmd': 'MMD info (extras.mmd)',
  'info.summary': 'Summary',
  'info.nodes': 'Nodes',
  'info.meshes': 'Meshes',
  'info.primitives': 'Primitives (draw units)',
  'info.vertices': 'Vertices',
  'info.triangles': 'Triangles',
  'info.materials': 'Materials',
  'info.texturesImages': 'Textures / images',
  'info.textures': 'Textures',
  'info.skinsBones': 'Skins / bones',
  'info.bones': 'Bones',
  'info.morphs': 'Morphs',
  'info.animations': 'Animations',
  'info.renderStats': 'Rendering (live)',
  'info.materialsCount': 'Materials ({count})',
  'info.texturesCount': 'Textures ({count})',
  'info.imagesCount': 'Images ({count}, {size})',
  'info.morphsCount': 'Morphs ({count})',
  'info.animationsCount': 'Animations ({count})',
  'info.nodeTree': 'Node hierarchy ({count})',
  'info.name': 'Name',
  'info.type': 'Type',
  'info.alpha': 'Alpha',
  'info.doubleSided': 'Double-sided',
  'info.resolution': 'Resolution',
  'info.duration': 'Length',
  'info.tracks': 'Tracks',
  'info.seconds': '{value} s',
  'info.textureLoading': 'Loading',
  'info.mmdDisplay': 'Rendering',
  'info.mmdDisplayValue': 'Can be shown with MMD materials (toon, sphere, edge). Switch with “MMD view” at the top',
  'info.converter': 'Converter',
  'info.modelName': 'Model name',
  'info.englishName': 'English name',
  'info.originalFormat': 'Original format',
  'info.mmdMaterials': 'MMD materials',
  'info.mmdBones': 'MMD bones',
  'info.displayFrames': 'Display frames',
  'info.rigidBodiesJoints': 'Rigid bodies / joints',
  'info.drawCalls': 'Draw calls',
  'info.drawnTriangles': 'Triangles drawn',
  'info.gpuGeometries': 'GPU geometries',
  'info.gpuTextures': 'GPU textures',
};

export type Language = 'ja' | 'en';
export type LanguageSetting = 'auto' | Language;

const STORAGE_KEY = 'facetgl.language';
const listeners: (() => void)[] = [];
let setting: LanguageSetting = loadSetting();
let language: Language = resolve(setting);

function loadSetting(): LanguageSetting {
  try {
    const saved = localStorage.getItem(STORAGE_KEY);
    return saved === 'ja' || saved === 'en' ? saved : 'auto';
  } catch {
    return 'auto'; // storage can be unavailable
  }
}

function systemLanguage(): Language {
  const langs = navigator.languages?.length ? navigator.languages : [navigator.language];
  return (langs[0] ?? '').toLowerCase().startsWith('ja') ? 'ja' : 'en';
}

function resolve(s: LanguageSetting): Language {
  return s === 'auto' ? systemLanguage() : s;
}

export function currentLanguage() {
  return language;
}

export function languageSetting() {
  return setting;
}

export function setLanguageSetting(value: LanguageSetting) {
  setting = value;
  try {
    if (value === 'auto') localStorage.removeItem(STORAGE_KEY);
    else localStorage.setItem(STORAGE_KEY, value);
  } catch {
    // Not being able to remember the choice is not an error worth showing
  }
  const next = resolve(value);
  if (next === language) return;
  language = next;
  applyStaticTexts();
  for (const listener of listeners) listener();
}

// Called after the language changes (to redraw texts set from code)
export function onLanguageChange(listener: () => void) {
  listeners.push(listener);
}

export function t(key: TextKey, params: Record<string, string | number> = {}) {
  const text = (language === 'en' ? en : ja)[key];
  return text.replace(/\{(\w+)\}/g, (match, name) => (name in params ? String(params[name]) : match));
}

// Texts written in index.html
export function applyStaticTexts(root: ParentNode = document) {
  document.documentElement.lang = language;
  for (const e of root.querySelectorAll<HTMLElement>('[data-i18n]')) e.textContent = t(e.dataset.i18n as TextKey);
  for (const e of root.querySelectorAll<HTMLElement>('[data-i18n-title]')) e.title = t(e.dataset.i18nTitle as TextKey);
}
