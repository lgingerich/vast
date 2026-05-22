import { Stars } from "@react-three/drei";
import type { TrajectoryPoint } from "../../types";
import type { RunManifest } from "../../types";
import { EARTH_RADIUS_M } from "../../viz/constants";
import type { CameraMode } from "./SceneCameraRig";
import { EciAxes, EquatorialGrid } from "./EciAxes";
import { EarthGlobe } from "./EarthGlobe";
import { ReferenceOrbitRing } from "./ReferenceOrbitRing";
import { SceneCameraRig } from "./SceneCameraRig";
import { SpacecraftMarker } from "./SpacecraftMarker";
import { TrajectoryPaths } from "./TrajectoryPaths";

interface OrbitSceneProps {
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

interface ReferenceOrbit {
  radiusM: number;
  inclinationDeg: number;
  raanDeg: number;
  argPeriapsisDeg: number;
}

function referenceOrbits(manifest: RunManifest | null): ReferenceOrbit[] {
  if (!manifest) {
    return [];
  }
  const orbits: ReferenceOrbit[] = [];
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
  const seen = new Set<string>();
  return orbits.filter((orbit) => {
    const key = `${orbit.radiusM.toFixed(1)}|${orbit.inclinationDeg}|${orbit.raanDeg}|${orbit.argPeriapsisDeg}`;
    if (seen.has(key)) {
      return false;
    }
    seen.add(key);
    return true;
  });
}

export function OrbitScene({
  trajectory,
  frameIndex,
  manifest,
  cameraMode,
  fitKey,
  overviewCenter,
  overviewRadiusM,
  showGrid,
  showTriad,
}: OrbitSceneProps) {
  const current = trajectory[frameIndex] ?? trajectory[0];
  const orbits = referenceOrbits(manifest);

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
          color={index === 0 ? "#64748b" : "#475569"}
          inclinationDeg={orbit.inclinationDeg}
          raanDeg={orbit.raanDeg}
          argPeriapsisDeg={orbit.argPeriapsisDeg}
        />
      ))}
      <TrajectoryPaths trajectory={trajectory} frameIndex={frameIndex} />
      {current ? <SpacecraftMarker point={current} showTriad={showTriad} /> : null}

      <SceneCameraRig
        mode={cameraMode}
        spacecraftPoint={current ?? null}
        fitKey={fitKey}
        overviewCenter={overviewCenter}
        overviewRadiusM={overviewRadiusM}
      />
    </>
  );
}
