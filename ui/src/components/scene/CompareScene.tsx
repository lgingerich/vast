import { Line, Stars } from "@react-three/drei";
import { useMemo } from "react";
import { CatmullRomCurve3, Color, Vector3 } from "three";
import type { RunManifest, TrajectoryPoint } from "../../types";
import { EARTH_RADIUS_M } from "../../viz/constants";
import { eciToScene } from "../../viz/coordinates";
import { EarthGlobe } from "./EarthGlobe";
import { EciAxes, EquatorialGrid } from "./EciAxes";
import { ReferenceOrbitRing } from "./ReferenceOrbitRing";
import { SceneCameraRig, type CameraMode } from "./SceneCameraRig";
import { SpacecraftMarker } from "./SpacecraftMarker";

export interface CompareEntry {
  episodeIndex: number;
  color: string;
  trajectory: TrajectoryPoint[];
}

interface CompareSceneProps {
  entries: CompareEntry[];
  frameIndex: number;
  manifest: RunManifest | null;
  cameraMode: CameraMode;
  fitKey: string;
  overviewCenter: [number, number, number] | null;
  overviewRadiusM: number;
  showGrid: boolean;
  showTriad: boolean;
  followEpisode: number | null;
}

const FUTURE_DIM = 0.22;
const PAST_BRIGHT = 1.0;
const SUBDIVISIONS_PER_SEGMENT = 12;

function smoothPolyline(
  points: [number, number, number][],
  subdivisions: number,
): { smoothed: [number, number, number][]; originalIndex: number[] } {
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
  const total = (points.length - 1) * subdivisions;
  const smoothed: [number, number, number][] = [];
  const originalIndex: number[] = [];
  const out = new Vector3();
  for (let i = 0; i <= total; i += 1) {
    const t = i / total;
    curve.getPoint(t, out);
    smoothed.push([out.x, out.y, out.z]);
    originalIndex.push(i / subdivisions);
  }
  return { smoothed, originalIndex };
}

function fadeVertexColors(
  baseHex: string,
  originalIndex: number[],
  frameIndex: number,
): [number, number, number][] {
  const base = new Color(baseHex);
  const colors: [number, number, number][] = [];
  for (let i = 0; i < originalIndex.length; i += 1) {
    const delta = originalIndex[i] - frameIndex;
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

function referenceOrbits(manifest: RunManifest | null) {
  if (!manifest) {
    return [];
  }
  const orbits: {
    radiusM: number;
    inclinationDeg: number;
    raanDeg: number;
    argPeriapsisDeg: number;
  }[] = [];
  if (manifest.scenario?.altitude_m !== undefined) {
    orbits.push({
      radiusM: EARTH_RADIUS_M + manifest.scenario.altitude_m,
      inclinationDeg: 0,
      raanDeg: 0,
      argPeriapsisDeg: 0,
    });
  }
  const initial = manifest.mission?.initial_orbit;
  if (initial) {
    let radiusM = 0;
    if (initial.semi_major_axis_m !== undefined) {
      radiusM = initial.semi_major_axis_m;
    } else if (initial.altitude_m !== undefined) {
      radiusM = EARTH_RADIUS_M + initial.altitude_m;
    }
    if (radiusM > 0) {
      orbits.push({
        radiusM,
        inclinationDeg: initial.inclination_deg ?? 0,
        raanDeg: initial.raan_deg ?? 0,
        argPeriapsisDeg: initial.arg_periapsis_deg ?? 0,
      });
    }
  }
  return orbits;
}

export function CompareScene({
  entries,
  frameIndex,
  manifest,
  cameraMode,
  fitKey,
  overviewCenter,
  overviewRadiusM,
  showGrid,
  showTriad,
  followEpisode,
}: CompareSceneProps) {
  const orbits = referenceOrbits(manifest);

  const lines = useMemo(
    () =>
      entries.map((entry) => {
        const scenePoints = entry.trajectory.map((point) => eciToScene(point.pos));
        const { smoothed, originalIndex } = smoothPolyline(
          scenePoints,
          SUBDIVISIONS_PER_SEGMENT,
        );
        return {
          key: entry.episodeIndex,
          points: smoothed,
          color: entry.color,
          colors: fadeVertexColors(entry.color, originalIndex, frameIndex),
        };
      }),
    [entries, frameIndex],
  );

  const followPoint = useMemo(() => {
    if (followEpisode === null) {
      const first = entries[0];
      if (!first) return null;
      return first.trajectory[Math.min(frameIndex, first.trajectory.length - 1)] ?? null;
    }
    const entry = entries.find((e) => e.episodeIndex === followEpisode);
    if (!entry) return null;
    return entry.trajectory[Math.min(frameIndex, entry.trajectory.length - 1)] ?? null;
  }, [entries, followEpisode, frameIndex]);

  return (
    <>
      <color attach="background" args={["#05070d"]} />
      <Stars radius={140} depth={50} count={3500} factor={2.6} saturation={0} fade speed={0.15} />
      <ambientLight intensity={0.18} />
      <directionalLight position={[12, 8, 6]} intensity={1.45} />
      <directionalLight position={[-10, -3, -10]} intensity={0.2} color="#93c5fd" />

      <EarthGlobe sunDirection={[12, 8, 6]} />
      {showGrid ? (
        <>
          <EciAxes />
          <EquatorialGrid />
        </>
      ) : null}
      {orbits.map((orbit, index) => (
        <ReferenceOrbitRing
          key={`${orbit.radiusM}-${index}`}
          radiusM={orbit.radiusM}
          color="#475569"
          inclinationDeg={orbit.inclinationDeg}
          raanDeg={orbit.raanDeg}
          argPeriapsisDeg={orbit.argPeriapsisDeg}
        />
      ))}

      {lines.map((line) =>
        line.points.length >= 2 ? (
          <Line
            key={line.key}
            points={line.points}
            vertexColors={line.colors}
            lineWidth={2.2}
            transparent
            opacity={0.95}
          />
        ) : null,
      )}

      {entries.map((entry) => {
        const point =
          entry.trajectory[Math.min(frameIndex, entry.trajectory.length - 1)];
        if (!point) {
          return null;
        }
        return (
          <SpacecraftMarker
            key={entry.episodeIndex}
            point={point}
            showTriad={
              showTriad &&
              (followEpisode === null
                ? entry.episodeIndex === entries[0]?.episodeIndex
                : entry.episodeIndex === followEpisode)
            }
          />
        );
      })}

      <SceneCameraRig
        mode={cameraMode}
        spacecraftPoint={followPoint}
        fitKey={fitKey}
        overviewCenter={overviewCenter}
        overviewRadiusM={overviewRadiusM}
      />
    </>
  );
}
