import { Line } from "@react-three/drei";
import { useMemo } from "react";
import { CatmullRomCurve3, Color, Vector3 } from "three";
import type { TrajectoryPoint } from "../../types";
import { SEGMENT_COLORS } from "../../viz/constants";
import { eciToScene } from "../../viz/coordinates";
import { hasSegmentMetadata, segmentRanges } from "../../viz/trajectory";

interface TrajectoryPathsProps {
  trajectory: TrajectoryPoint[];
  frameIndex: number;
}

const FUTURE_DIM = 0.22;
const PAST_BRIGHT = 1.0;
/** Subdivisions per sample-to-sample segment for the smoothed curve. */
const SUBDIVISIONS_PER_SEGMENT = 12;

function toScenePoints(points: TrajectoryPoint[]): [number, number, number][] {
  return points.map((point) => eciToScene(point.pos));
}

/**
 * Densify a polyline with centripetal Catmull-Rom interpolation. Returns the
 * smoothed points plus, for each smoothed point, the fractional index in the
 * original polyline (0..points.length-1). The fractional index lets callers
 * map smoothed vertices back to original samples for things like past/future
 * coloring.
 */
function smoothPolyline(
  points: [number, number, number][],
  subdivisions: number,
): {
  smoothed: [number, number, number][];
  originalIndex: number[];
} {
  if (points.length < 2 || subdivisions <= 1) {
    return {
      smoothed: points,
      originalIndex: points.map((_, i) => i),
    };
  }

  const curve = new CatmullRomCurve3(
    points.map(([x, y, z]) => new Vector3(x, y, z)),
    false,
    "centripetal",
    0.5,
  );

  const totalSegments = (points.length - 1) * subdivisions;
  const smoothed: [number, number, number][] = [];
  const originalIndex: number[] = [];
  const out = new Vector3();
  for (let i = 0; i <= totalSegments; i += 1) {
    const t = i / totalSegments;
    curve.getPoint(t, out);
    smoothed.push([out.x, out.y, out.z]);
    originalIndex.push((i / subdivisions));
  }
  return { smoothed, originalIndex };
}

/**
 * Per-vertex colors that brighten the past and dim the future. The fractional
 * original index is used so the fade crosses smoothly through the spacecraft
 * marker rather than snapping at sample boundaries.
 */
function fadeVertexColors(
  baseHex: string,
  originalIndex: number[],
  frameIndex: number,
  offset = 0,
): [number, number, number][] {
  const base = new Color(baseHex);
  const colors: [number, number, number][] = [];
  for (let i = 0; i < originalIndex.length; i += 1) {
    const globalIndex = offset + originalIndex[i];
    const delta = globalIndex - frameIndex;
    let intensity: number;
    if (delta <= 0) {
      intensity = PAST_BRIGHT;
    } else if (delta >= 1) {
      intensity = FUTURE_DIM;
    } else {
      intensity = PAST_BRIGHT + (FUTURE_DIM - PAST_BRIGHT) * delta;
    }
    colors.push([base.r * intensity, base.g * intensity, base.b * intensity]);
  }
  return colors;
}

export function TrajectoryPaths({ trajectory, frameIndex }: TrajectoryPathsProps) {
  const segmented = hasSegmentMetadata(trajectory);
  const ranges = useMemo(() => segmentRanges(trajectory), [trajectory]);

  const allPoints = useMemo(() => toScenePoints(trajectory), [trajectory]);

  const smoothedAll = useMemo(
    () => smoothPolyline(allPoints, SUBDIVISIONS_PER_SEGMENT),
    [allPoints],
  );

  const unsegmentedColors = useMemo(() => {
    if (segmented) {
      return [];
    }
    return fadeVertexColors("#dbeafe", smoothedAll.originalIndex, frameIndex);
  }, [smoothedAll, frameIndex, segmented]);

  const segmentLines = useMemo(() => {
    if (!segmented) {
      return null;
    }
    return ranges.map((range, rangeIndex) => {
      // Extend the slice by one sample into the next segment so consecutive
      // segments share a boundary vertex and visually connect at the join.
      const isLast = rangeIndex === ranges.length - 1;
      const sliceEnd = isLast
        ? range.endFrame + 1
        : Math.min(range.endFrame + 2, allPoints.length);
      const slicePoints = allPoints.slice(range.startFrame, sliceEnd);
      const { smoothed, originalIndex } = smoothPolyline(
        slicePoints,
        SUBDIVISIONS_PER_SEGMENT,
      );
      const color = SEGMENT_COLORS[range.index % SEGMENT_COLORS.length];
      const colors = fadeVertexColors(
        color,
        originalIndex,
        frameIndex,
        range.startFrame,
      );
      return { key: range.index, points: smoothed, color, colors };
    });
  }, [allPoints, frameIndex, ranges, segmented]);

  if (trajectory.length < 2) {
    return null;
  }

  if (segmented && segmentLines) {
    return (
      <group>
        {segmentLines.map((line) =>
          line.points.length >= 2 ? (
            <Line
              key={line.key}
              points={line.points}
              vertexColors={line.colors}
              lineWidth={2.5}
              transparent
              opacity={0.95}
            />
          ) : null,
        )}
      </group>
    );
  }

  return (
    <Line
      points={smoothedAll.smoothed}
      vertexColors={unsegmentedColors}
      lineWidth={2.4}
      transparent
      opacity={0.95}
    />
  );
}
