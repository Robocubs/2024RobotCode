import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    outDir: process.env.DASHBOARD_OUT_DIR ?? 'dist',
    chunkSizeWarningLimit: 3000,
    rollupOptions: {
      output: {
        manualChunks: {
          fwc: ['@frc-web-components/fwc'],
        },
      },
    },
  },
});
