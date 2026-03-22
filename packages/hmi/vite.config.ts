import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

export default defineConfig({
  plugins: [react()],
  assetsInclude: ['**/*.glb'],
  resolve: {
    alias: {
      '@convex-api': path.resolve(__dirname, '../app/convex/_generated/api'),
      '@convex-types': path.resolve(__dirname, '../app/convex/_generated/dataModel'),
    },
  },
  server: {
    port: 3000,
    host: true
  }
})
