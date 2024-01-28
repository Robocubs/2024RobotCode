import { defineConfig } from 'vite';

export default defineConfig({
  build: {
    outDir: process.env.DASHBOARD_OUT_DIR ?? 'dist',
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
