import type { RunManifest, TrajectoryPoint } from "./types";

const SAMPLE_MANIFEST_URL = "/sample-run/manifest.json";

async function fetchJson<T>(url: string): Promise<T> {
  const response = await fetch(url);
  if (!response.ok) {
    throw new Error(`Request failed: ${response.status}`);
  }
  return response.json() as Promise<T>;
}

export async function listRuns(): Promise<RunManifest[]> {
  try {
    return await fetchJson<RunManifest[]>("/api/runs");
  } catch {
    const sample = await fetchJson<RunManifest>(SAMPLE_MANIFEST_URL);
    return [sample];
  }
}

export async function getRun(runId: string): Promise<RunManifest> {
  try {
    return await fetchJson<RunManifest>(`/api/runs/${runId}`);
  } catch {
    const sample = await fetchJson<RunManifest>(SAMPLE_MANIFEST_URL);
    if (sample.run_id !== runId) {
      throw new Error("Run not found");
    }
    return sample;
  }
}

export async function getTrajectory(
  runId: string,
  episodeIndex: number,
): Promise<TrajectoryPoint[]> {
  try {
    return await fetchJson<TrajectoryPoint[]>(
      `/api/runs/${runId}/episodes/${episodeIndex}/trajectory`,
    );
  } catch {
    const sample = await fetchJson<RunManifest>(SAMPLE_MANIFEST_URL);
    if (sample.run_id !== runId) {
      throw new Error("Run not found");
    }
    const response = await fetch(
      `/sample-run/episodes/${String(episodeIndex).padStart(4, "0")}/trajectory.jsonl`,
    );
    if (!response.ok) {
      throw new Error("Trajectory not found");
    }
    const text = await response.text();
    return text
      .trim()
      .split("\n")
      .filter(Boolean)
      .map((line) => JSON.parse(line) as TrajectoryPoint);
  }
}

export function altitudeMeters(point: TrajectoryPoint, earthRadiusM: number): number {
  const [x, y, z] = point.pos;
  return Math.hypot(x, y, z) - earthRadiusM;
}

export function speedMetersPerSecond(point: TrajectoryPoint): number {
  const [x, y, z] = point.vel;
  return Math.hypot(x, y, z);
}

const EARTH_RADIUS_M = 6_371_000;

export { EARTH_RADIUS_M };
