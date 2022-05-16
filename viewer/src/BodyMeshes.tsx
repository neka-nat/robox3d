import { useEffect, useMemo, useRef, useState } from "react";
import { useFrame } from "@react-three/fiber";
import * as THREE from "three";
import { ConvexGeometry, GLTFLoader } from "three-stdlib";
import { poseBuffer, useViewer } from "./store";
import type { BodyDesc, ShapeDesc, VisualDesc } from "./types";
import { base64ToArrayBuffer } from "./types";

const PALETTE = [
  "#4c9be8", "#e8734c", "#5fbf7a", "#c46be0",
  "#e0b84b", "#5bc8c4", "#e05c7e", "#8a9bb0",
];
const STATIC_COLOR = "#5a6068";

function bodyColor(body: BodyDesc): string {
  if (body.color) return body.color;
  if (body.kind === "static") return STATIC_COLOR;
  return PALETTE[body.i % PALETTE.length];
}

function Shape({ desc, color }: { desc: ShapeDesc; color: string }) {
  const material = <meshStandardMaterial color={color} roughness={0.55} metalness={0.1} />;

  if (desc.kind === "sphere") {
    return (
      <mesh position={desc.center} castShadow receiveShadow>
        <sphereGeometry args={[desc.radius, 32, 16]} />
        {material}
      </mesh>
    );
  }

  if (desc.kind === "capsule") {
    const p1 = new THREE.Vector3(...desc.p1);
    const p2 = new THREE.Vector3(...desc.p2);
    const dir = new THREE.Vector3().subVectors(p2, p1);
    const length = dir.length();
    const mid = new THREE.Vector3().addVectors(p1, p2).multiplyScalar(0.5);
    // CapsuleGeometry is Y-aligned; rotate Y onto the p1→p2 direction
    const quat = new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dir.clone().normalize(),
    );
    return (
      <mesh position={mid} quaternion={quat} castShadow receiveShadow>
        <capsuleGeometry args={[desc.radius, length, 8, 24]} />
        {material}
      </mesh>
    );
  }

  // hull: convex hull (box/cylinder/mesh all end up here)
  return <HullShape points={desc.points} color={color} />;
}

function HullShape({ points, color }: { points: [number, number, number][]; color: string }) {
  const geometry = useMemo(() => {
    const vecs = points.map((p) => new THREE.Vector3(...p));
    return new ConvexGeometry(vecs);
  }, [points]);
  return (
    <mesh geometry={geometry} castShadow receiveShadow>
      <meshStandardMaterial color={color} roughness={0.55} metalness={0.1} />
    </mesh>
  );
}

function GlbVisual({ glb }: { glb: string }) {
  const [object, setObject] = useState<THREE.Group | null>(null);
  useEffect(() => {
    const loader = new GLTFLoader();
    let disposed = false;
    loader.parse(
      base64ToArrayBuffer(glb),
      "",
      (gltf) => {
        if (disposed) return;
        gltf.scene.traverse((o) => {
          if (o instanceof THREE.Mesh) {
            o.castShadow = true;
            o.receiveShadow = true;
          }
        });
        setObject(gltf.scene);
      },
      (err) => console.error("Failed to load GLB:", err),
    );
    return () => {
      disposed = true;
    };
  }, [glb]);
  return object ? <primitive object={object} /> : null;
}

function Visual({ desc, fallbackColor }: { desc: VisualDesc; fallbackColor: string }) {
  const color = desc.color ?? fallbackColor;
  const material = <meshStandardMaterial color={color} roughness={0.55} metalness={0.1} />;
  const { p, q } = desc.xf;

  let inner: React.ReactNode;
  if (desc.kind === "box") {
    inner = (
      <mesh castShadow receiveShadow>
        <boxGeometry args={desc.size} />
        {material}
      </mesh>
    );
  } else if (desc.kind === "sphere") {
    inner = (
      <mesh castShadow receiveShadow>
        <sphereGeometry args={[desc.radius, 32, 16]} />
        {material}
      </mesh>
    );
  } else if (desc.kind === "cylinder") {
    // three.js CylinderGeometry is Y-aligned; URDF cylinders are Z-aligned
    inner = (
      <mesh rotation={[Math.PI / 2, 0, 0]} castShadow receiveShadow>
        <cylinderGeometry args={[desc.radius, desc.radius, desc.length, 32]} />
        {material}
      </mesh>
    );
  } else {
    inner = <GlbVisual glb={desc.glb} />;
  }

  return (
    <group position={p} quaternion={new THREE.Quaternion(...q)}>
      {inner}
    </group>
  );
}

/** Meshes for all bodies. Poses are interpolated in useFrame and written directly to refs. */
export function BodyMeshes({ bodies }: { bodies: BodyDesc[] }) {
  const groupRefs = useRef<(THREE.Group | null)[]>([]);
  const qPrev = useMemo(() => new THREE.Quaternion(), []);
  const qLatest = useMemo(() => new THREE.Quaternion(), []);

  useFrame(() => {
    const { prev, latest } = poseBuffer;
    if (!latest) return;

    // Interpolate prev→latest delayed by one receive interval (smoothness first)
    let alpha = 1.0;
    if (prev && latest.recvTime > prev.recvTime) {
      alpha = (performance.now() - latest.recvTime) / (latest.recvTime - prev.recvTime);
      alpha = Math.min(Math.max(alpha, 0), 1);
    }

    const n = latest.poses.length / 7;
    for (let i = 0; i < n; i++) {
      const g = groupRefs.current[i];
      if (!g) continue;
      const o = i * 7;
      const lp = latest.poses;
      if (prev && alpha < 1 && prev.poses.length === latest.poses.length) {
        const pp = prev.poses;
        g.position.set(
          pp[o] + (lp[o] - pp[o]) * alpha,
          pp[o + 1] + (lp[o + 1] - pp[o + 1]) * alpha,
          pp[o + 2] + (lp[o + 2] - pp[o + 2]) * alpha,
        );
        qPrev.set(pp[o + 3], pp[o + 4], pp[o + 5], pp[o + 6]);
        qLatest.set(lp[o + 3], lp[o + 4], lp[o + 5], lp[o + 6]);
        g.quaternion.slerpQuaternions(qPrev, qLatest, alpha);
      } else {
        g.position.set(lp[o], lp[o + 1], lp[o + 2]);
        g.quaternion.set(lp[o + 3], lp[o + 4], lp[o + 5], lp[o + 6]);
      }
    }
  });

  const showCollision = useViewer((s) => s.showCollision);

  return (
    <>
      {bodies.map((body) => {
        const useVisuals = body.visuals.length > 0 && !showCollision;
        return (
          <group
            key={body.i}
            ref={(el) => {
              groupRefs.current[body.i] = el;
            }}
          >
            {useVisuals
              ? body.visuals.map((v, vi) => (
                  <Visual key={vi} desc={v} fallbackColor={bodyColor(body)} />
                ))
              : body.shapes.map((shape, si) => (
                  <Shape key={si} desc={shape} color={bodyColor(body)} />
                ))}
          </group>
        );
      })}
    </>
  );
}
