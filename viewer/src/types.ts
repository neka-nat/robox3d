// Protocol types mirroring robox3d.viz.protocol

export type ShapeDesc =
  | { kind: "sphere"; center: [number, number, number]; radius: number }
  | {
      kind: "capsule";
      p1: [number, number, number];
      p2: [number, number, number];
      radius: number;
    }
  | { kind: "hull"; points: [number, number, number][] };

export interface XF {
  p: [number, number, number];
  q: [number, number, number, number]; // xyzw
}

export type VisualDesc = { xf: XF; color: string | null } & (
  | { kind: "box"; size: [number, number, number] }
  | { kind: "sphere"; radius: number }
  | { kind: "cylinder"; radius: number; length: number }
  | { kind: "mesh"; glb: string } // base64 GLB
);

export interface BodyDesc {
  i: number;
  name: string;
  kind: "static" | "dynamic";
  color: string | null;
  shapes: ShapeDesc[];
  visuals: VisualDesc[];
}

export interface JointDesc {
  name: string;
  index: number;
  kind: "revolute" | "prismatic";
  lower: number;
  upper: number;
  value: number;
}

export interface RobotDesc {
  name: string;
  joints: JointDesc[];
}

export interface SceneMsg {
  type: "scene";
  version: number;
  bodies: BodyDesc[];
  robots: RobotDesc[];
}

// Binary pose frame: [u8 msgType=1][u8 x3][f64 simTime][f32 x 7 x n]
export const POSE_HEADER_BYTES = 12;

export interface PoseFrame {
  simTime: number;
  poses: Float32Array; // n x 7 (x, y, z, qx, qy, qz, qw)
  recvTime: number; // performance.now() [ms]
}

export function decodePoseFrame(buf: ArrayBuffer): PoseFrame | null {
  const view = new DataView(buf);
  if (view.getUint8(0) !== 1) return null;
  return {
    simTime: view.getFloat64(4, true),
    poses: new Float32Array(buf, POSE_HEADER_BYTES),
    recvTime: performance.now(),
  };
}

export function base64ToArrayBuffer(b64: string): ArrayBuffer {
  const bin = atob(b64);
  const bytes = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) bytes[i] = bin.charCodeAt(i);
  return bytes.buffer;
}
