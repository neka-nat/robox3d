import { useState } from "react";
import { useViewer } from "./store";
import type { RobotDesc } from "./types";

function Slider({ robot, jointIndex }: { robot: RobotDesc; jointIndex: number }) {
  const joint = robot.joints[jointIndex];
  const sendCommand = useViewer((s) => s.sendCommand);
  const [value, setValue] = useState(joint.value);

  const onChange = (v: number) => {
    setValue(v);
    sendCommand({ type: "set_target", robot: robot.name, joint: joint.name, value: v });
  };

  return (
    <div style={{ display: "flex", alignItems: "center", gap: 8 }}>
      <span style={{ width: 110, overflow: "hidden", textOverflow: "ellipsis" }}>
        {joint.name}
      </span>
      <input
        type="range"
        min={joint.lower}
        max={joint.upper}
        step={0.01}
        value={value}
        onChange={(e) => onChange(parseFloat(e.target.value))}
        style={{ flex: 1 }}
      />
      <span style={{ width: 48, textAlign: "right" }}>
        {value.toFixed(2)}
      </span>
    </div>
  );
}

export function JointPanel() {
  const scene = useViewer((s) => s.scene);
  if (!scene || scene.robots.length === 0) return null;

  return (
    <div
      style={{
        position: "absolute",
        top: 12,
        right: 12,
        zIndex: 10,
        width: 300,
        display: "flex",
        flexDirection: "column",
        gap: 10,
        fontFamily: "ui-monospace, monospace",
        fontSize: 12,
        color: "#cdd3dc",
        background: "rgba(21, 23, 28, 0.85)",
        border: "1px solid #2a2e37",
        borderRadius: 8,
        padding: "10px 14px",
      }}
    >
      {scene.robots.map((robot) => (
        <div key={robot.name} style={{ display: "flex", flexDirection: "column", gap: 6 }}>
          <div style={{ fontWeight: 600, opacity: 0.85 }}>{robot.name}</div>
          {robot.joints.map((j) => (
            <Slider key={j.name} robot={robot} jointIndex={j.index} />
          ))}
        </div>
      ))}
    </div>
  );
}
