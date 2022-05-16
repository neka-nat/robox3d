import { useEffect, useRef } from "react";
import { useFrame, useThree } from "@react-three/fiber";
import * as THREE from "three";
import { poseBuffer, useViewer } from "./store";
import type { BodyDesc, SceneMsg, ShapeDesc } from "./types";

// Diagonal view direction (Z-up), matching the default camera's look.
const VIEW_DIR = new THREE.Vector3(1.3, -1.3, 0.9).normalize();
const UNIT_SCALE = new THREE.Vector3(1, 1, 1);

interface ControlsLike {
  target: THREE.Vector3;
  update: () => void;
  addEventListener: (type: string, listener: () => void) => void;
  removeEventListener: (type: string, listener: () => void) => void;
}

function expandByShape(shape: ShapeDesc, box: THREE.Box3): void {
  if (shape.kind === "sphere") {
    const c = new THREE.Vector3(...shape.center);
    box.expandByPoint(c.clone().addScalar(-shape.radius));
    box.expandByPoint(c.clone().addScalar(shape.radius));
  } else if (shape.kind === "capsule") {
    for (const end of [shape.p1, shape.p2]) {
      const v = new THREE.Vector3(...end);
      box.expandByPoint(v.clone().addScalar(-shape.radius));
      box.expandByPoint(v.clone().addScalar(shape.radius));
    }
  } else {
    for (const p of shape.points) box.expandByPoint(new THREE.Vector3(...p));
  }
}

/** World-space AABB of the given bodies' collision shapes at the given poses. */
function worldBox(bodies: BodyDesc[], poses: Float32Array): THREE.Box3 | null {
  const out = new THREE.Box3();
  const local = new THREE.Box3();
  const xf = new THREE.Matrix4();
  const p = new THREE.Vector3();
  const q = new THREE.Quaternion();
  for (const body of bodies) {
    const o = body.i * 7;
    if (body.shapes.length === 0 || o + 7 > poses.length) continue;
    local.makeEmpty();
    for (const shape of body.shapes) expandByShape(shape, local);
    p.set(poses[o], poses[o + 1], poses[o + 2]);
    q.set(poses[o + 3], poses[o + 4], poses[o + 5], poses[o + 6]);
    xf.compose(p, q, UNIT_SCALE);
    out.union(local.clone().applyMatrix4(xf));
  }
  return out.isEmpty() ? null : out;
}

/**
 * Keep the camera framed on the moving scene. The frame accumulates the
 * volume dynamic bodies have visited (so a swinging pendulum or a falling box
 * smoothly zooms the view out) and re-aims with a gentle lerp. The first
 * camera interaction (orbit/zoom/pan) hands control to the user for good.
 */
export function AutoFrame() {
  const scene = useViewer((s) => s.scene);
  const camera = useThree((s) => s.camera);
  const controls = useThree((s) => s.controls) as ControlsLike | null;

  const st = useRef({
    scene: null as SceneMsg | null,
    accum: new THREE.Box3(),
    framedRadius: 0,
    desiredPos: new THREE.Vector3(),
    desiredTarget: new THREE.Vector3(),
    hasDesired: false,
    snapNextFrame: true,
    userDrove: false,
  });

  // The user taking the camera (orbit/zoom/pan) disables auto-framing.
  useEffect(() => {
    if (!controls) return;
    const onStart = () => {
      st.current.userDrove = true;
    };
    controls.addEventListener("start", onStart);
    return () => controls.removeEventListener("start", onStart);
  }, [controls]);

  useFrame(() => {
    const s = st.current;
    if (!scene || !controls) return;
    if (s.scene !== scene) {
      s.scene = scene;
      s.accum.makeEmpty();
      s.framedRadius = 0;
      s.hasDesired = false;
      s.snapNextFrame = true;
      s.userDrove = false;
    }
    if (s.userDrove) return;
    const latest = poseBuffer.latest;
    if (!latest) return;

    const dynamic = scene.bodies.filter((b) => b.kind === "dynamic");
    const box = worldBox(dynamic.length > 0 ? dynamic : scene.bodies, latest.poses);
    if (!box) return;
    // Z-up scenes: keep the ground plane in frame.
    box.min.z = Math.min(box.min.z, 0);
    s.accum.union(box);

    const center = s.accum.getCenter(new THREE.Vector3());
    const radius = Math.max(
      s.accum.getBoundingSphere(new THREE.Sphere()).radius,
      0.05,
    );
    // Re-aim only when the scene outgrows the last framing (5% hysteresis).
    if (!s.hasDesired || radius > s.framedRadius * 1.05) {
      s.framedRadius = radius;
      // 4.2x the bounding radius reads well for robots: close enough to see
      // detail, enough margin that links stay in frame while moving.
      const dist = Math.max(4.2 * radius, 0.35);
      s.desiredPos.copy(center).addScaledVector(VIEW_DIR, dist);
      s.desiredTarget.copy(center);
      s.hasDesired = true;
      if (camera instanceof THREE.PerspectiveCamera) {
        camera.near = Math.max(dist / 100, 0.001);
        camera.far = Math.max(dist * 100, 50);
        camera.updateProjectionMatrix();
      }
    }

    if (s.snapNextFrame) {
      // First framing: jump straight there instead of flying in from afar.
      camera.position.copy(s.desiredPos);
      controls.target.copy(s.desiredTarget);
      s.snapNextFrame = false;
    } else {
      camera.position.lerp(s.desiredPos, 0.08);
      controls.target.lerp(s.desiredTarget, 0.08);
    }
    controls.update();
  });

  return null;
}
