import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import path from 'path'

// Anchor for resolving convex/* as if imported from inside packages/hmi/node_modules
const hmiNodeModules = path.resolve(__dirname, 'node_modules')

export default defineConfig({
  plugins: [
    react(),
    // Redirect all "convex" and "convex/*" imports to HMI's own node_modules.
    // Uses a fake importer inside packages/hmi so that Node module resolution
    // finds the correct file via convex's package.json exports map.
    {
      name: 'resolve-convex-from-hmi',
      async resolveId(source, _importer, options) {
        if (source === 'convex' || source.startsWith('convex/')) {
          const fakeImporter = path.join(hmiNodeModules, '.fake-importer.js')
          return this.resolve(source, fakeImporter, { ...options, skipSelf: true })
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
