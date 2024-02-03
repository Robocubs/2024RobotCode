import { defineConfig } from 'vite';

export default defineConfig({
  base: process.env.DASHBOARD_BASE ?? '/',
  build: {
    outDir: process.env.DASHBOARD_OUT_DIR ?? 'dist',
    emptyOutDir: true,
    chunkSizeWarningLimit: 1000,
    rollupOptions: {
      output: {
        manualChunks: {
          'fwc-components': ['@frc-web-components/fwc/components'],
          'fwc-source-providers': ['@frc-web-components/fwc/source-providers'],
          'fwc': ['@frc-web-components/fwc'],
        },
      },
    },
  },
});
