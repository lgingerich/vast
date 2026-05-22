import { Canvas } from "@react-three/fiber";
import { Suspense } from "react";
import type { RunManifest, TrajectoryPoint } from "../types";
import type { CameraMode } from "./scene/SceneCameraRig";
import { OrbitScene } from "./scene/OrbitScene";

interface OrbitReplayProps {
  trajectory: TrajectoryPoint[];
  frameIndex: number;
  manifest: RunManifest | null;
  cameraMode: CameraMode;
  fitKey: string;
  overviewCenter: [number, number, number] | null;
  overviewRadiusM: number;
  showGrid: boolean;
  showTriad: boolean;
}

export function OrbitReplay({
  trajectory,
  frameIndex,
  manifest,
  cameraMode,
  fitKey,
  overviewCenter,
  overviewRadiusM,
  showGrid,
  showTriad,
}: OrbitReplayProps) {
  return (
    <Canvas camera={{ position: [0, 8, 14], fov: 45 }} dpr={[1, 2]}>
      <Suspense fallback={null}>
        <OrbitScene
          trajectory={trajectory}
          frameIndex={frameIndex}
          manifest={manifest}
          cameraMode={cameraMode}
          fitKey={fitKey}
          overviewCenter={overviewCenter}
          overviewRadiusM={overviewRadiusM}
          showGrid={showGrid}
          showTriad={showTriad}
        />
      </Suspense>
    </Canvas>
  );
}
