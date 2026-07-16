import { defineConfig } from 'vite'
import react from '@vitejs/plugin-react'

// Dev server port mirrors the launcher's --port default; override with --port.
export default defineConfig({
  plugins: [react()],
  server: { port: 5173, host: true, proxy: { '/api': 'http://localhost:8321' } },
})
