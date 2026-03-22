import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

// packages/hmi/node_modules/convex — the only place convex is installed on Vercel
const convexRoot = path.resolve(__dirname, 'node_modules/convex')

export default defineConfig({
  plugins: [
    react(),
    // Redirect all "convex" and "convex/*" imports to HMI's own node_modules.
    // This is needed because ../app/convex/_generated/api imports convex/server
    // but convex is only installed under packages/hmi, not packages/app.
    {
      name: 'resolve-convex-from-hmi',
      resolveId(source) {
        if (source === 'convex' || source.startsWith('convex/')) {
          return path.join(convexRoot, source.slice('convex'.length))
        }
      },
    },
  ],
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
