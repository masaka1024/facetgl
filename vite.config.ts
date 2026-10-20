import { defineConfig } from 'vite';

export default defineConfig({
  // Relative asset paths: the same build runs inside Tauri and under GitHub Pages'
  // /facetgl/ sub-path
  base: './',
  // Tauri expects a fixed port and shows its own window
  clearScreen: false,
  server: {
    port: 3000,
    strictPort: true
  }
});
