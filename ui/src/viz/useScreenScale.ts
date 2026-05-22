import { useFrame } from "@react-three/fiber";
import type { RefObject } from "react";
import type { Group } from "three";

/** Keeps a world object near constant on-screen size as the camera moves. */
export function useScreenScale(groupRef: RefObject<Group | null>, pixels = 14) {
  useFrame(({ camera, size }) => {
    const group = groupRef.current;
    if (!group) {
      return;
    }

    const distance = camera.position.distanceTo(group.position);
    const perspective = "fov" in camera ? camera : null;
    const vFov = perspective ? (perspective.fov * Math.PI) / 180 : Math.PI / 4;
    const worldHeight = 2 * Math.tan(vFov / 2) * distance;
    const worldPerPixel = worldHeight / Math.max(size.height, 1);
    const scale = Math.max(worldPerPixel * pixels, worldPerPixel * 8);
    group.scale.setScalar(scale);
  });
}
