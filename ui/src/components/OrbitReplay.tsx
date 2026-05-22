import { Canvas } from "@react-three/fiber";
import { Line, OrbitControls, Sphere } from "@react-three/drei";
import { useMemo } from "react";
import type { TrajectoryPoint } from "../types";
import { EARTH_RADIUS_M } from "../api";

const SCENE_SCALE = 1 / 1_000_000;

interface OrbitReplayProps {
  trajectory: TrajectoryPoint[];
  frameIndex: number;
}

export function OrbitReplay({ trajectory, frameIndex }: OrbitReplayProps) {
  const points = useMemo(
    () =>
      trajectory.map(
        (point) =>
          [
            point.pos[0] * SCENE_SCALE,
            point.pos[2] * SCENE_SCALE,
            point.pos[1] * SCENE_SCALE,
          ] as [number, number, number],
      ),
    [trajectory],
  );

  const current = trajectory[frameIndex] ?? trajectory[0];
  const spacecraftPosition = current
    ? ([
        current.pos[0] * SCENE_SCALE,
        current.pos[2] * SCENE_SCALE,
        current.pos[1] * SCENE_SCALE,
      ] as [number, number, number])
    : ([0, 0, 0] as [number, number, number]);

  return (
    <Canvas camera={{ position: [0, 8, 12], fov: 45 }}>
      <color attach="background" args={["#05070d"]} />
      <ambientLight intensity={0.35} />
      <directionalLight position={[5, 5, 5]} intensity={1.2} />
      <Sphere args={[EARTH_RADIUS_M * SCENE_SCALE, 64, 64]}>
        <meshStandardMaterial color="#1d4ed8" />
      </Sphere>
      {points.length > 1 ? (
        <Line points={points} color="#93c5fd" lineWidth={1} />
      ) : null}
      <Sphere args={[0.08, 16, 16]} position={spacecraftPosition}>
        <meshStandardMaterial color="#f97316" emissive="#ea580c" />
      </Sphere>
      <OrbitControls makeDefault />
    </Canvas>
  );
}
