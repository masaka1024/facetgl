// Everything that differs between the desktop app (Tauri) and the web version
// (GitHub Pages). The same build runs in both; the branch is taken at runtime.
//
// Desktop: models are real file paths, read through Rust commands and the asset protocol.
// Web: the files the user dropped or picked (a whole folder for PMX) form a small virtual
//      file system addressed by paths like "web/Model/model.pmx". Loaders fetch them through
//      virtual URLs that are mapped to blob URLs, so relative texture paths still resolve.
import { convertFileSrc, invoke } from '@tauri-apps/api/core';
import { listen } from '@tauri-apps/api/event';
import { homeDir } from '@tauri-apps/api/path';
import { getCurrentWebview } from '@tauri-apps/api/webview';
import { getCurrentWindow } from '@tauri-apps/api/window';
import { open, message } from '@tauri-apps/plugin-dialog';

export const isTauri = typeof window !== 'undefined' && '__TAURI_INTERNALS__' in window;

// --- Web virtual file system ---

const VFS_ROOT = 'web';
const VFS_URL = 'https://facetgl.vfs/';

interface WebEntry { path: string; file: File }
const webEntries = new Map<string, WebEntry>(); // key: lower-case normalized path
const webBlobUrls = new Map<string, string>();

// Resolves "." / ".." and backslashes so lookups match however a model spells a path
function normalizePath(path: string) {
  const out: string[] = [];
  for (const part of path.replace(/\\/g, '/').split('/')) {
    if (part === '' || part === '.') continue;
    if (part === '..') out.pop();
    else out.push(part);
  }
  return out.join('/');
}

// replace = false adds the files to the ones already loaded (a motion dropped onto an open
// model must not take the model's own files away)
function setWebFiles(files: { path: string; file: File }[], replace = true) {
  if (replace) {
    webBlobUrls.forEach((u) => URL.revokeObjectURL(u));
    webBlobUrls.clear();
    webEntries.clear();
  }
  for (const { path, file } of files) {
    const full = normalizePath(`${VFS_ROOT}/${path}`);
    webEntries.set(full.toLowerCase(), { path: full, file });
  }
  return [...webEntries.values()].map((e) => e.path);
}

// Adds files under their own sub-folder so that they never clash with the model's files
let webMergeCount = 0;
function addWebFiles(files: { path: string; file: File }[]) {
  const folder = `added${++webMergeCount}`;
  setWebFiles(files.map(({ path, file }) => ({ path: `${folder}/${path}`, file })), false);
  return files.map(({ path }) => normalizePath(`${VFS_ROOT}/${folder}/${path}`));
}

const isMotion = (p: string) => /\.vmd$/i.test(p);

/** A dropped / picked set of files: the model to open (if any) and a motion (if any). */
export interface PickedFiles {
  model: string | null;
  motion: string | null;
}

function pickFrom(paths: string[], extensions: string[]): PickedFiles {
  return { model: chooseModel(paths, extensions), motion: paths.find(isMotion) ?? null };
}

// Web: files that contain a model replace the virtual file system; files without one
// (a motion for the open model) are added to it
function webPick(files: { path: string; file: File }[], extensions: string[]): PickedFiles {
  const hasModel = files.some(({ path }) => extensions.includes(path.split('.').pop()?.toLowerCase() ?? ''));
  return pickFrom(hasModel ? setWebFiles(files) : addWebFiles(files), extensions);
}

function webFile(path: string) {
  const entry = webEntries.get(normalizePath(path).toLowerCase());
  if (!entry) throw new Error(`${path}: ファイルが見つかりません (フォルダごと読み込むと解決することがあります)`);
  return entry.file;
}

// --- File access (used by the loaders) ---

export function readFile(path: string): Promise<ArrayBuffer> {
  if (isTauri) return invoke<ArrayBuffer>('read_file', { path });
  return Promise.resolve().then(() => webFile(path).arrayBuffer());
}

export function fileSize(path: string): Promise<number> {
  if (isTauri) return invoke<number>('file_size', { path });
  return Promise.resolve().then(() => webFile(path).size);
}

// Files under a folder, as paths relative to it with "/" separators
export function listFiles(dir: string): Promise<string[]> {
  if (isTauri) return invoke<string[]>('list_files', { dir });
  const prefix = normalizePath(dir).toLowerCase() + '/';
  return Promise.resolve(
    [...webEntries.entries()].filter(([key]) => key.startsWith(prefix)).map(([, e]) => e.path.slice(prefix.length)),
  );
}

// How many folders deep a path is below the top of its drive:
// "C:\\a\\b" -> 2, "\\\\server\\share\\a" -> 1 (a network share counts like a drive),
// "/Volumes/NAS/a" -> 1 (a macOS external disk or mounted share), "/a/b" -> 2
export function depthBelowDrive(path: string) {
  const segments = path.split(/[\\/]/).filter(Boolean);
  if (/^(\\\\|\/\/)/.test(path)) return segments.length - 2; // UNC: \\server\share
  if (/^\/Volumes\//.test(path)) return segments.length - 2; // macOS: /Volumes/<disk>

  return segments.length - (/^[A-Za-z]:$/.test(segments[0] ?? '') ? 1 : 0);
}

// Folders to search for a model's textures: the model's own folder, then up to `levels`
// parent folders (nearest first). Never climbs to the home folder or the top of a drive
// (including the top of a network share), so that the whole disk, a whole NAS share and
// macOS-protected folders like Documents are not scanned.
export async function textureSearchRoots(modelDir: string, levels = 3) {
  const trim = (p: string) => p.replace(/[\\/]+$/, '');
  const same = (a: string, b: string) => trim(a).replace(/\\/g, '/').toLowerCase() === trim(b).replace(/\\/g, '/').toLowerCase();
  let home = '';
  if (isTauri) {
    try {
      home = await homeDir();
    } catch {
      // Unknown home folder: the top-of-drive check below still applies
    }
  }
  const roots = [modelDir];
  let dir = trim(modelDir);
  for (let i = 0; i < levels; i++) {
    const cut = Math.max(dir.lastIndexOf('/'), dir.lastIndexOf('\\'));
    if (cut <= 0) break;
    const parent = dir.slice(0, cut);
    // Stop below folders that hold whole disks or all users: "/Users", "/Volumes", "C:\\Users",
    // "\\\\server\\share\\folder", and on the web the "web" root of the dropped files
    if (depthBelowDrive(parent) <= 1 || (home && same(parent, home))) break;
    roots.push(parent);
    dir = parent;
  }
  return roots;
}

// URL for a file. Each path segment is encoded separately so that loaders can resolve
// textures relative to the model ("tex/body.png" -> same folder).
export function assetUrl(path: string) {
  const encoded = path.replace(/\\/g, '/').split('/').map(encodeURIComponent).join('/');
  return (isTauri ? convertFileSrc('') : VFS_URL) + encoded;
}

// Last step of every loader URL: on the web, virtual URLs become blob URLs
export function resolveUrl(url: string) {
  if (isTauri || !url.startsWith(VFS_URL)) return url;
  let path = url.slice(VFS_URL.length);
  try {
    path = decodeURIComponent(path);
  } catch {
    // Keep the raw string if it contains a stray "%"
  }
  const key = normalizePath(path).toLowerCase();
  const entry = webEntries.get(key);
  if (!entry) return url; // fails to load and is reported as a missing file
  if (!webBlobUrls.has(key)) webBlobUrls.set(key, URL.createObjectURL(entry.file));
  return webBlobUrls.get(key)!;
}

// --- Window / dialogs ---

export function setWindowTitle(title: string) {
  if (isTauri) getCurrentWindow().setTitle(title);
  else document.title = title;
}

export function showError(text: string) {
  if (isTauri) message(text, { title: 'FacetGL', kind: 'error' });
  else window.alert(text);
}

// Picks the model to open among several files: supported, shallowest, then by format order
function chooseModel(paths: string[], extensions: string[]) {
  const rank = (p: string) => extensions.indexOf(p.split('.').pop()?.toLowerCase() ?? '');
  const candidates = paths.filter((p) => rank(p) >= 0);
  const depth = (p: string) => p.split(/[\\/]/).length;
  candidates.sort((a, b) => depth(a) - depth(b) || rank(a) - rank(b));
  return candidates[0] ?? null;
}

function pickWebFiles(extensions: string[], directory: boolean): Promise<PickedFiles | null> {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.multiple = true;
    if (directory) input.webkitdirectory = true;
    else input.accept = extensions.map((e) => '.' + e).join(',') + ',.vmd,image/*,.bin,.mtl,.tga,.bmp,.sph,.spa';
    input.addEventListener('change', () => {
      const files = [...(input.files ?? [])];
      if (files.length === 0) return resolve(null);
      // webkitRelativePath keeps the folder structure when a folder was picked
      resolve(webPick(files.map((file) => ({ path: file.webkitRelativePath || file.name, file })), extensions));
    });
    input.click();
  });
}

// null = cancelled
export async function pickModelFile(filters: { name: string; extensions: string[] }[], extensions: string[]): Promise<PickedFiles | null> {
  if (isTauri) {
    const path = await open({ multiple: false, directory: false, filters });
    return typeof path === 'string' ? pickFrom([path], extensions) : null;
  }
  return pickWebFiles(extensions, false);
}

export async function pickModelFolder(extensions: string[]): Promise<PickedFiles | null> {
  if (isTauri) {
    const dir = await open({ multiple: false, directory: true });
    return typeof dir === 'string' ? pickFrom(await expandFolders([dir]), extensions) : null;
  }
  return pickWebFiles(extensions, true);
}

// Desktop: replaces each folder in the list with the files inside it (3 levels deep).
// A plain file lists as empty and is kept as it is.
async function expandFolders(paths: string[]) {
  const out: string[] = [];
  for (const path of paths) {
    const inside = await listFiles(path);
    if (inside.length === 0) out.push(path);
    else out.push(...inside.map((rel) => `${path.replace(/[\\/]+$/, '')}/${rel}`));
  }
  return out;
}

export const isMac = typeof navigator !== 'undefined' && /Mac/.test(navigator.platform || navigator.userAgent);

export async function startupFile() {
  return isTauri ? invoke<string | null>('startup_file') : null;
}

// Files the OS asks the running app to open (macOS: Finder double-click, "Open With", Dock).
// Register this before calling startupFile() so that no file is missed in between.
export async function onOpenFile(handler: (path: string) => void) {
  if (!isTauri) return;
  await listen<string>('open-file', (event) => handler(event.payload));
}

// --- Drag and drop ---

interface DropHandlers {
  enter: () => void;
  leave: () => void;
  drop: (picked: PickedFiles) => void;
}

export function onModelDrop(extensions: string[], handlers: DropHandlers) {
  if (isTauri) {
    // Tauri delivers real file paths
    getCurrentWebview().onDragDropEvent((event) => {
      const payload = event.payload;
      if (payload.type === 'enter' || payload.type === 'over') handlers.enter();
      else if (payload.type === 'leave') handlers.leave();
      else if (payload.type === 'drop') {
        handlers.leave();
        // A dropped folder (e.g. a PMX with its textures) is searched for the model
        expandFolders(payload.paths).then((paths) => handlers.drop(pickFrom(paths, extensions)));
      }
    });
    return;
  }

  let depth = 0;
  document.body.addEventListener('dragenter', (e) => {
    e.preventDefault();
    if (depth++ === 0) handlers.enter();
  });
  document.body.addEventListener('dragover', (e) => e.preventDefault());
  document.body.addEventListener('dragleave', () => {
    if (--depth <= 0) {
      depth = 0;
      handlers.leave();
    }
  });
  document.body.addEventListener('drop', async (e) => {
    e.preventDefault();
    depth = 0;
    handlers.leave();
    const files = await collectDroppedFiles(e.dataTransfer);
    handlers.drop(webPick(files, extensions));
  });
}

// Reads dropped files and folders (recursively) with their relative paths
async function collectDroppedFiles(transfer: DataTransfer | null) {
  const out: { path: string; file: File }[] = [];
  if (!transfer) return out;

  const entries = [...transfer.items].map((item) => item.webkitGetAsEntry?.()).filter((e): e is FileSystemEntry => !!e);
  if (entries.length === 0) {
    for (const file of transfer.files) out.push({ path: file.name, file });
    return out;
  }

  const walk = async (entry: FileSystemEntry, prefix: string): Promise<void> => {
    if (entry.isFile) {
      const file = await new Promise<File>((res, rej) => (entry as FileSystemFileEntry).file(res, rej));
      out.push({ path: prefix + entry.name, file });
    } else if (entry.isDirectory) {
      const reader = (entry as FileSystemDirectoryEntry).createReader();
      // readEntries returns results in batches until it returns an empty list
      for (;;) {
        const batch = await new Promise<FileSystemEntry[]>((res, rej) => reader.readEntries(res, rej));
        if (batch.length === 0) break;
        for (const child of batch) await walk(child, `${prefix}${entry.name}/`);
      }
    }
  };
  for (const entry of entries) await walk(entry, '');
  return out;
}
