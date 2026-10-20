import { defineConfig } from 'vite';

export default defineConfig({
  // Tauri expects a fixed port and shows its own window
  clearScreen: false,
  server: {
    port: 3000,
    strictPort: true
  }
});
