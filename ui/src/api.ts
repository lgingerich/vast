import type { EpisodeMetric, RunManifest, TrajectoryPoint } from "./types";

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

function parseJsonLines<T>(text: string): T[] {
  return text
    .trim()
    .split("\n")
    .filter(Boolean)
    .map((line) => JSON.parse(line) as T);
}

export async function getMetrics(runId: string): Promise<EpisodeMetric[]> {
  try {
    const response = await fetch(`/api/runs/${runId}/metrics`);
    if (!response.ok) {
      throw new Error(`Request failed: ${response.status}`);
    }
    const contentType = response.headers.get("content-type") ?? "";
    if (contentType.includes("application/json")) {
      return (await response.json()) as EpisodeMetric[];
    }
    return parseJsonLines<EpisodeMetric>(await response.text());
  } catch {
    const sample = await fetchJson<RunManifest>(SAMPLE_MANIFEST_URL);
    if (sample.run_id !== runId) {
      throw new Error("Run not found");
    }
    const response = await fetch("/sample-run/metrics.jsonl");
    if (!response.ok) {
      return [];
    }
    return parseJsonLines<EpisodeMetric>(await response.text());
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
    return parseJsonLines<TrajectoryPoint>(await response.text());
  }
}
