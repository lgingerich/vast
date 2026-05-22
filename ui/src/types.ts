export interface ScenarioConfig {
  name: string;
  altitude_m: number;
  dt_s: number;
  duration_s: number;
  integrator: string;
  force_model: string;
  sample_every_steps: number;
}

export interface RunManifest {
  version: number;
  run_id: string;
  created_at: string;
  scenario_name: string;
  scenario: ScenarioConfig;
  episode_count: number;
}

export interface TrajectoryPoint {
  t: number;
  pos: [number, number, number];
  vel: [number, number, number];
}
