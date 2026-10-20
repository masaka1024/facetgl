// Everything that differs between the desktop app (Tauri) and the web version
// (GitHub Pages). The same build runs in both; the branch is taken at runtime.
//
// Desktop: models are real file paths, read through Rust commands and the asset protocol.
// Web: the files the user dropped or picked (a whole folder for PMX) form a small virtual
//      file system addressed by paths like "web/Model/model.pmx". Loaders fetch them through
//      virtual URLs that are mapped to blob URLs, so relative texture paths still resolve.
import { convertFileSrc, invoke } from '@tauri-apps/api/core';
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

function setWebFiles(files: { path: string; file: File }[]) {
  webBlobUrls.forEach((u) => URL.revokeObjectURL(u));
  webBlobUrls.clear();
  webEntries.clear();
  for (const { path, file } of files) {
    const full = normalizePath(`${VFS_ROOT}/${path}`);
    webEntries.set(full.toLowerCase(), { path: full, file });
  }
  return [...webEntries.values()].map((e) => e.path);
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

function pickWebFiles(extensions: string[], directory: boolean): Promise<string | null> {
  return new Promise((resolve) => {
    const input = document.createElement('input');
    input.type = 'file';
    input.multiple = true;
    if (directory) input.webkitdirectory = true;
    else input.accept = extensions.map((e) => '.' + e).join(',') + ',image/*,.bin,.mtl,.tga,.bmp,.sph,.spa';
    input.addEventListener('change', () => {
      const files = [...(input.files ?? [])];
      if (files.length === 0) return resolve(null);
      // webkitRelativePath keeps the folder structure when a folder was picked
      const paths = setWebFiles(files.map((file) => ({ path: file.webkitRelativePath || file.name, file })));
      resolve(chooseModel(paths, extensions));
    });
    input.click();
  });
}

export async function pickModelFile(filters: { name: string; extensions: string[] }[], extensions: string[]) {
  if (isTauri) {
    const path = await open({ multiple: false, directory: false, filters });
    return typeof path === 'string' ? path : null;
  }
  return pickWebFiles(extensions, false);
}

export function pickModelFolder(extensions: string[]) {
  return pickWebFiles(extensions, true);
}

export async function startupFile() {
  return isTauri ? invoke<string | null>('startup_file') : null;
}

// --- Drag and drop ---

interface DropHandlers {
  enter: () => void;
  leave: () => void;
  drop: (modelPath: string | null) => void;
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
        handlers.drop(chooseModel(payload.paths, extensions));
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
    handlers.drop(chooseModel(setWebFiles(files), extensions));
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
