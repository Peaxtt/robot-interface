import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'
import { nodePolyfills } from 'vite-plugin-node-polyfills'

// https://vitejs.dev/config/
export default defineConfig({
  plugins: [
    react(),
    nodePolyfills({
      // สั่งให้ Plugin เติมเต็มส่วนที่ขาดหายไปของ ROSLib (แก้จอขาว)
      protocolImports: true,
    }),
  ],
})