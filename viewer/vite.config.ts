import { defineConfig } from "vite";
import react from "@vitejs/plugin-react";

export default defineConfig({
  plugins: [react()],
  // Relative asset paths so the build can be served from any root
  // (VizServer serves it from robox3d/viz/static on the WebSocket port).
  base: "./",
  server: { port: 5173 },
});
