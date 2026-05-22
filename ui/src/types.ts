export interface ScenarioConfig {
  name: string;
  altitude_m: number;
  dt_s: number;
  duration_s: number;
  integrator: string;
  force_model: string;
  sample_every_steps: number;
}

export interface MissionInitialOrbit {
  kind: string;
  altitude_m?: number;
  semi_major_axis_m?: number;
  eccentricity?: number;
  inclination_deg?: number;
  raan_deg?: number;
  arg_periapsis_deg?: number;
  true_anomaly_deg?: number;
}

export interface MissionSegmentConfig {
  stop: string | { duration_s?: number; steps?: number };
  burns?: unknown[];
}

export interface MissionScenarioConfig {
  name: string;
  kind: string;
  initial_orbit: MissionInitialOrbit;
  propagation: {
    dt_s: number;
    integrator: string;
    force_model: string;
    sample_every_steps: number;
  };
  segments: MissionSegmentConfig[];
}

export interface RunManifest {
  version: number;
  run_id: string;
  created_at: string;
  scenario_name: string;
  scenario?: ScenarioConfig | null;
  mission?: MissionScenarioConfig | null;
  episode_count: number;
}

export interface TrajectoryPoint {
  t: number;
  pos: [number, number, number];
  vel: [number, number, number];
  segment_index?: number;
}

export interface EpisodeMetric {
  episode_index: number;
  seed: number;
  steps: number;
  stop_reason: string;
  final_altitude_m: number;
  reward: number;
}

export function isMissionRun(manifest: RunManifest): boolean {
  return manifest.mission !== undefined && manifest.mission !== null;
}

export function runKindLabel(manifest: RunManifest): string {
  return isMissionRun(manifest) ? "Mission" : "Rollout";
}
