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
  // ✅ เพิ่มส่วนนี้เข้าไปครับ
  server: {
    host: true, // หรือ '0.0.0.0' เพื่อเปิดให้เข้าถึงจาก IP ภายนอกได้
    port: 5174, // กำหนดพอร์ตให้แน่นอน (ถ้าไม่ซีเรียสไม่ต้องใส่ก็ได้ แต่ใส่ไว้ชัวร์กว่า)
  },
})