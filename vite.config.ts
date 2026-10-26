import { execSync } from 'node:child_process';
import { readFileSync } from 'node:fs';
import { defineConfig } from 'vite';

// Build identity shown in the window title and the toolbar, so that a test build can be told
// apart from a release: "v0.4.1 (7561f4f)", plus the branch for builds not made from main
// ("v0.4.1 glb-physics (7561f4f)"). GitHub Actions provides the commit and branch in its
// environment; local builds ask git.
function git(args: string) {
  try {
    return execSync(`git ${args}`, { stdio: ['ignore', 'pipe', 'ignore'] }).toString().trim();
  } catch {
    return '';
  }
}

const version = JSON.parse(readFileSync(new URL('./package.json', import.meta.url), 'utf8')).version as string;
const commit = (process.env.GITHUB_SHA ?? git('rev-parse HEAD')).slice(0, 7);
const branch =
  process.env.GITHUB_REF_TYPE === 'tag' ? '' : (process.env.GITHUB_REF_NAME ?? git('rev-parse --abbrev-ref HEAD'));
const buildLabel = [`v${version}`, branch && branch !== 'main' && branch !== 'HEAD' ? branch : '', commit ? `(${commit})` : '']
  .filter(Boolean)
  .join(' ');

export default defineConfig({
  // Relative asset paths: the same build runs inside Tauri and under GitHub Pages'
  // /facetgl/ sub-path
  base: './',
  define: {
    __BUILD_LABEL__: JSON.stringify(buildLabel),
  },
  // Tauri expects a fixed port and shows its own window
  clearScreen: false,
  server: {
    port: 3000,
    strictPort: true
  }
});
