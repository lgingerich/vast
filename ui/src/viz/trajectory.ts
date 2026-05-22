import type { TrajectoryPoint } from "../types";
import { MU_EARTH } from "./constants";

export interface SegmentRange {
  index: number;
  startFrame: number;
  endFrame: number;
}

export function segmentRanges(trajectory: TrajectoryPoint[]): SegmentRange[] {
  if (trajectory.length === 0) {
    return [];
  }

  const ranges: SegmentRange[] = [];
  let currentSegment = trajectory[0]?.segment_index ?? 0;
  let startFrame = 0;

  for (let index = 1; index < trajectory.length; index += 1) {
    const segment = trajectory[index]?.segment_index ?? currentSegment;
    if (segment !== currentSegment) {
      ranges.push({ index: currentSegment, startFrame, endFrame: index - 1 });
      currentSegment = segment;
      startFrame = index;
    }
  }

  ranges.push({
    index: currentSegment,
    startFrame,
    endFrame: trajectory.length - 1,
  });

  return ranges;
}

export function hasSegmentMetadata(trajectory: TrajectoryPoint[]): boolean {
  return trajectory.some((point) => point.segment_index !== undefined);
}

export function frameForSegmentStart(
  ranges: SegmentRange[],
  segmentIndex: number,
): number | null {
  const match = ranges.find((range) => range.index === segmentIndex);
  return match?.startFrame ?? null;
}

export interface TelemetrySample {
  timeS: number;
  altitudeKm: number;
  speedKmS: number;
  specificEnergyMJPerKg: number;
}

export function telemetrySeries(
  trajectory: TrajectoryPoint[],
  earthRadiusM: number,
): TelemetrySample[] {
  return trajectory.map((point) => {
    const radiusM = Math.hypot(...point.pos);
    const speedMps = Math.hypot(...point.vel);
    const specificEnergyJPerKg =
      0.5 * speedMps * speedMps - (radiusM > 0 ? MU_EARTH / radiusM : 0);
    return {
      timeS: point.t,
      altitudeKm: (radiusM - earthRadiusM) / 1000,
      speedKmS: speedMps / 1000,
      specificEnergyMJPerKg: specificEnergyJPerKg / 1_000_000,
    };
  });
}
