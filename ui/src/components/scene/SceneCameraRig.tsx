import { OrbitControls } from "@react-three/drei";
import { useFrame, useThree } from "@react-three/fiber";
import { useEffect, useRef } from "react";
import type { OrbitControls as OrbitControlsImpl } from "three-stdlib";
import type { TrajectoryPoint } from "../../types";
import { SCENE_SCALE } from "../../viz/constants";
import { eciToScene } from "../../viz/coordinates";

export type CameraMode = "free" | "earth" | "spacecraft" | "overview";

interface SceneCameraRigProps {
  mode: CameraMode;
  spacecraftPoint: TrajectoryPoint | null;
  fitKey: string;
  overviewCenter: [number, number, number] | null;
  overviewRadiusM: number;
}

export function SceneCameraRig({
  mode,
  spacecraftPoint,
  fitKey,
  overviewCenter,
  overviewRadiusM,
}: SceneCameraRigProps) {
  const controlsRef = useRef<OrbitControlsImpl>(null);
  const { camera } = useThree();
  const initialFitDone = useRef<string | null>(null);

  useEffect(() => {
    if (mode !== "overview" || !overviewCenter || initialFitDone.current === fitKey) {
      return;
    }

    const controls = controlsRef.current;
    if (!controls) {
      return;
    }

    const center = eciToScene(overviewCenter);
    const distance = overviewRadiusM * SCENE_SCALE * 1.75;
    camera.position.set(
      center[0] + distance * 0.35,
      center[1] + distance * 0.55,
      center[2] + distance,
    );
    controls.target.set(center[0], center[1], center[2]);
    controls.update();
    initialFitDone.current = fitKey;
  }, [camera, fitKey, mode, overviewCenter, overviewRadiusM]);

  useEffect(() => {
    const controls = controlsRef.current;
    if (!controls) {
      return;
    }

    if (mode === "earth") {
      controls.target.set(0, 0, 0);
      camera.position.set(0, 0.8, 2.8);
      controls.update();
      return;
    }

    if (mode === "spacecraft" && spacecraftPoint) {
      const position = eciToScene(spacecraftPoint.pos);
      controls.target.set(position[0], position[1], position[2]);
      camera.position.set(
        position[0] + 0.45,
        position[1] + 0.35,
        position[2] + 0.55,
      );
      controls.update();
    }
  }, [camera, mode, spacecraftPoint]);

  useFrame(() => {
    if (mode !== "spacecraft" || !spacecraftPoint) {
      return;
    }

    const controls = controlsRef.current;
    if (!controls) {
      return;
    }

    const position = eciToScene(spacecraftPoint.pos);
    controls.target.x += (position[0] - controls.target.x) * 0.12;
    controls.target.y += (position[1] - controls.target.y) * 0.12;
    controls.target.z += (position[2] - controls.target.z) * 0.12;
    controls.update();
  });

  return (
    <OrbitControls
      ref={controlsRef}
      makeDefault
      enableDamping
      dampingFactor={0.08}
      rotateSpeed={0.65}
      zoomSpeed={1.1}
      panSpeed={0.8}
      minDistance={0.05}
      maxDistance={500}
      enablePan
    />
  );
}
