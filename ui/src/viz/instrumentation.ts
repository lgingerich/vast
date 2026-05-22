import { EARTH_RADIUS_M, MU_EARTH } from "./constants";
import type { TrajectoryPoint } from "../types";
import { altitudeMeters, speedMetersPerSecond } from "./coordinates";

export interface FrameInstrumentation {
  timeS: number;
  altitudeKm: number;
  radiusKm: number;
  speedKmS: number;
  radialSpeedKmS: number;
  flightPathAngleDeg: number;
  specificEnergyMJPerKg: number;
  angularMomentumMagnitude: number;
  segmentIndex?: number;
}

export function frameInstrumentation(
  point: TrajectoryPoint,
  earthRadiusM = EARTH_RADIUS_M,
): FrameInstrumentation {
  const [x, y, z] = point.pos;
  const [vx, vy, vz] = point.vel;
  const radiusM = Math.hypot(x, y, z);
  const speedMps = speedMetersPerSecond(point);

  const radialSpeedMps =
    radiusM > 0 ? (x * vx + y * vy + z * vz) / radiusM : 0;
  const tangentialSpeedMps = Math.sqrt(
    Math.max(speedMps * speedMps - radialSpeedMps * radialSpeedMps, 0),
  );
  const flightPathAngleDeg =
    tangentialSpeedMps > 0
      ? (Math.atan2(radialSpeedMps, tangentialSpeedMps) * 180) / Math.PI
      : 0;

  const specificEnergyJPerKg = 0.5 * speedMps * speedMps - MU_EARTH / radiusM;
  const angularMomentumMagnitude = Math.hypot(
    y * vz - z * vy,
    z * vx - x * vz,
    x * vy - y * vx,
  );

  return {
    timeS: point.t,
    altitudeKm: altitudeMeters(point, earthRadiusM) / 1000,
    radiusKm: radiusM / 1000,
    speedKmS: speedMps / 1000,
    radialSpeedKmS: radialSpeedMps / 1000,
    flightPathAngleDeg,
    specificEnergyMJPerKg: specificEnergyJPerKg / 1_000_000,
    angularMomentumMagnitude,
    segmentIndex: point.segment_index,
  };
}

export interface OrbitBounds {
  minX: number;
  maxX: number;
  minY: number;
  maxY: number;
  minZ: number;
  maxZ: number;
  center: [number, number, number];
  radius: number;
}

export function trajectoryBounds(trajectory: TrajectoryPoint[]): OrbitBounds | null {
  if (trajectory.length === 0) {
    return null;
  }

  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;

  for (const point of trajectory) {
    const [x, y, z] = point.pos;
    minX = Math.min(minX, x);
    maxX = Math.max(maxX, x);
    minY = Math.min(minY, y);
    maxY = Math.max(maxY, y);
    minZ = Math.min(minZ, z);
    maxZ = Math.max(maxZ, z);
  }

  const center: [number, number, number] = [
    (minX + maxX) / 2,
    (minY + maxY) / 2,
    (minZ + maxZ) / 2,
  ];
  const radius = Math.max(
    Math.hypot(maxX - center[0], maxY - center[1], maxZ - center[2]),
    EARTH_RADIUS_M,
  );

  return { minX, maxX, minY, maxY, minZ, maxZ, center, radius };
}
