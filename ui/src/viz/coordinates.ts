import { SCENE_SCALE } from "./constants";
import type { TrajectoryPoint } from "../types";

/** Converts inertial ECI meters to the Three.js scene frame (Y-up, scaled). */
export function eciToScene(pos: [number, number, number]): [number, number, number] {
  return [pos[0] * SCENE_SCALE, pos[2] * SCENE_SCALE, pos[1] * SCENE_SCALE];
}

export function altitudeMeters(point: TrajectoryPoint, earthRadiusM: number): number {
  const [x, y, z] = point.pos;
  return Math.hypot(x, y, z) - earthRadiusM;
}

export function speedMetersPerSecond(point: TrajectoryPoint): number {
  const [x, y, z] = point.vel;
  return Math.hypot(x, y, z);
}

export function radiusMeters(point: TrajectoryPoint): number {
  const [x, y, z] = point.pos;
  return Math.hypot(x, y, z);
}
