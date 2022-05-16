import { create } from "zustand";
import type { PoseFrame, SceneMsg } from "./types";
import { decodePoseFrame } from "./types";

// Pose buffer lives outside React (read every frame from useFrame).
export const poseBuffer: { prev: PoseFrame | null; latest: PoseFrame | null } = {
  prev: null,
  latest: null,
};

// When the page is served by the sim/replay server itself (VizServer bundles the
// built viewer on the same port), connect back to the same host:port. The Vite
// dev server (port 5173) falls back to the default VizServer port.
const defaultWsUrl =
  location.protocol.startsWith("http") && location.port !== "5173"
    ? `ws://${location.host}`
    : `ws://${location.hostname || "127.0.0.1"}:8765`;

export type ConnState = "disconnected" | "connecting" | "connected";

interface ViewerStore {
  url: string;
  setUrl: (url: string) => void;
  conn: ConnState;
  scene: SceneMsg | null;
  simTime: number;
  showCollision: boolean;
  toggleCollision: () => void;
  connect: () => void;
  disconnect: () => void;
  sendCommand: (cmd: object) => void;
}

let ws: WebSocket | null = null;
let reconnectTimer: ReturnType<typeof setTimeout> | null = null;
let wantConnected = false;

export const useViewer = create<ViewerStore>((set, get) => {
  const open = () => {
    const { url } = get();
    set({ conn: "connecting" });
    ws = new WebSocket(url);
    ws.binaryType = "arraybuffer";

    ws.onopen = () => set({ conn: "connected" });

    ws.onmessage = (ev) => {
      if (typeof ev.data === "string") {
        const msg = JSON.parse(ev.data);
        if (msg.type === "scene") {
          poseBuffer.prev = null;
          poseBuffer.latest = null;
          set({ scene: msg as SceneMsg });
        }
      } else {
        const frame = decodePoseFrame(ev.data as ArrayBuffer);
        if (frame) {
          poseBuffer.prev = poseBuffer.latest;
          poseBuffer.latest = frame;
          set({ simTime: frame.simTime });
        }
      }
    };

    ws.onclose = () => {
      set({ conn: "disconnected" });
      if (wantConnected) {
        reconnectTimer = setTimeout(open, 2000); // auto-reconnect
      }
    };
    ws.onerror = () => ws?.close();
  };

  return {
    url: defaultWsUrl,
    setUrl: (url) => set({ url }),
    conn: "disconnected",
    scene: null,
    simTime: 0,
    showCollision: false,
    toggleCollision: () => set((s) => ({ showCollision: !s.showCollision })),
    connect: () => {
      wantConnected = true;
      open();
    },
    disconnect: () => {
      wantConnected = false;
      if (reconnectTimer) clearTimeout(reconnectTimer);
      ws?.close();
      set({ conn: "disconnected", scene: null });
    },
    sendCommand: (cmd) => {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(cmd));
      }
    },
  };
});
