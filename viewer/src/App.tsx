import { useEffect } from "react";
import { Canvas } from "@react-three/fiber";
import { Grid, OrbitControls } from "@react-three/drei";
import { AutoFrame } from "./AutoFrame";
import { BodyMeshes } from "./BodyMeshes";
import { JointPanel } from "./JointPanel";
import { useViewer } from "./store";

const STATUS_COLOR = {
  disconnected: "#e05c7e",
  connecting: "#e0b84b",
  connected: "#5fbf7a",
} as const;

function Hud() {
  const { url, setUrl, conn, scene, simTime, connect, disconnect, showCollision, toggleCollision } =
    useViewer();
  return (
    <div
      style={{
        position: "absolute",
        top: 12,
        left: 12,
        zIndex: 10,
        display: "flex",
        gap: 8,
        alignItems: "center",
        fontFamily: "ui-monospace, monospace",
        fontSize: 13,
        color: "#cdd3dc",
        background: "rgba(21, 23, 28, 0.85)",
        border: "1px solid #2a2e37",
        borderRadius: 8,
        padding: "8px 12px",
      }}
    >
      <span
        style={{
          width: 10,
          height: 10,
          borderRadius: "50%",
          background: STATUS_COLOR[conn],
          display: "inline-block",
        }}
      />
      <input
        value={url}
        onChange={(e) => setUrl(e.target.value)}
        disabled={conn !== "disconnected"}
        style={{
          background: "#1e2128",
          color: "#cdd3dc",
          border: "1px solid #2a2e37",
          borderRadius: 4,
          padding: "4px 8px",
          width: 220,
          fontFamily: "inherit",
          fontSize: "inherit",
        }}
      />
      {conn === "disconnected" ? (
        <button onClick={connect} style={buttonStyle}>
          Connect
        </button>
      ) : (
        <button onClick={disconnect} style={buttonStyle}>
          Disconnect
        </button>
      )}
      {scene && (
        <span style={{ opacity: 0.75 }}>
          {scene.bodies.length} bodies | t={simTime.toFixed(2)}s
        </span>
      )}
      {scene && (
        <label style={{ display: "flex", alignItems: "center", gap: 4, cursor: "pointer" }}>
          <input type="checkbox" checked={showCollision} onChange={toggleCollision} />
          collision
        </label>
      )}
    </div>
  );
}

const buttonStyle: React.CSSProperties = {
  background: "#2b3242",
  color: "#cdd3dc",
  border: "1px solid #3a4152",
  borderRadius: 4,
  padding: "4px 12px",
  cursor: "pointer",
  fontFamily: "inherit",
  fontSize: "inherit",
};

export default function App() {
  const scene = useViewer((s) => s.scene);
  const connect = useViewer((s) => s.connect);
  // Auto-connect on load; the store keeps retrying every 2s until a server appears.
  useEffect(() => {
    connect();
  }, [connect]);
  return (
    <div style={{ width: "100%", height: "100%", position: "relative" }}>
      <Hud />
      <JointPanel />
      <Canvas
        shadows
        camera={{ position: [2.5, -2.5, 1.8], up: [0, 0, 1], fov: 45 }}
      >
        <color attach="background" args={["#15171c"]} />
        <ambientLight intensity={0.55} />
        <directionalLight
          position={[4, -3, 8]}
          intensity={1.4}
          castShadow
          shadow-mapSize={[2048, 2048]}
        />
        {/* Z-up: drei's Grid lives in the XZ plane, rotate it into XY (z=0) */}
        <Grid
          rotation={[Math.PI / 2, 0, 0]}
          args={[20, 20]}
          cellColor="#2a2e37"
          sectionColor="#3a4152"
          infiniteGrid
          fadeDistance={25}
        />
        <axesHelper args={[0.5]} />
        {scene && <BodyMeshes bodies={scene.bodies} />}
        <AutoFrame />
        <OrbitControls makeDefault target={[0.5, 0, 0.3]} />
      </Canvas>
    </div>
  );
}
