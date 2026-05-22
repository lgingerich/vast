import type { EpisodeMetric } from "../types";

export interface MetricsSummary {
  count: number;
  totalSteps: number;
  reward: { mean: number; median: number; std: number; min: number; max: number };
  steps: { mean: number; min: number; max: number };
  finalAltitudeKm: { mean: number; min: number; max: number };
  stopReasons: { reason: string; count: number }[];
  successRate: number | null;
}

export interface HistogramBin {
  start: number;
  end: number;
  count: number;
}

const SUCCESS_STOP_REASONS = new Set([
  "duration_met",
  "steps_completed",
  "goal_reached",
  "target_reached",
]);

function quantile(sorted: number[], q: number): number {
  if (sorted.length === 0) {
    return 0;
  }
  if (sorted.length === 1) {
    return sorted[0];
  }
  const pos = (sorted.length - 1) * q;
  const lo = Math.floor(pos);
  const hi = Math.ceil(pos);
  if (lo === hi) {
    return sorted[lo];
  }
  return sorted[lo] + (pos - lo) * (sorted[hi] - sorted[lo]);
}

function reduceStats(values: number[]): {
  mean: number;
  median: number;
  std: number;
  min: number;
  max: number;
} {
  if (values.length === 0) {
    return { mean: 0, median: 0, std: 0, min: 0, max: 0 };
  }
  let sum = 0;
  let min = Infinity;
  let max = -Infinity;
  for (const value of values) {
    sum += value;
    if (value < min) min = value;
    if (value > max) max = value;
  }
  const mean = sum / values.length;
  let sqSum = 0;
  for (const value of values) {
    const d = value - mean;
    sqSum += d * d;
  }
  const std = Math.sqrt(sqSum / values.length);
  const sorted = [...values].sort((a, b) => a - b);
  return { mean, median: quantile(sorted, 0.5), std, min, max };
}

export function summarizeMetrics(metrics: EpisodeMetric[]): MetricsSummary {
  if (metrics.length === 0) {
    return {
      count: 0,
      totalSteps: 0,
      reward: { mean: 0, median: 0, std: 0, min: 0, max: 0 },
      steps: { mean: 0, min: 0, max: 0 },
      finalAltitudeKm: { mean: 0, min: 0, max: 0 },
      stopReasons: [],
      successRate: null,
    };
  }

  const rewards = metrics.map((m) => m.reward);
  const steps = metrics.map((m) => m.steps);
  const altitudesKm = metrics.map((m) => m.final_altitude_m / 1000);

  const rewardStats = reduceStats(rewards);
  const stepStats = reduceStats(steps);
  const altStats = reduceStats(altitudesKm);

  const totalSteps = steps.reduce((acc, value) => acc + value, 0);

  const reasons = new Map<string, number>();
  let successCount = 0;
  for (const m of metrics) {
    reasons.set(m.stop_reason, (reasons.get(m.stop_reason) ?? 0) + 1);
    if (SUCCESS_STOP_REASONS.has(m.stop_reason)) {
      successCount += 1;
    }
  }
  const stopReasons = [...reasons.entries()]
    .map(([reason, count]) => ({ reason, count }))
    .sort((a, b) => b.count - a.count);

  return {
    count: metrics.length,
    totalSteps,
    reward: rewardStats,
    steps: { mean: stepStats.mean, min: stepStats.min, max: stepStats.max },
    finalAltitudeKm: { mean: altStats.mean, min: altStats.min, max: altStats.max },
    stopReasons,
    successRate: successCount / metrics.length,
  };
}

/** Sturges-rule histogram for bounded numeric series. */
export function histogram(values: number[], binCount?: number): HistogramBin[] {
  if (values.length === 0) {
    return [];
  }
  let min = Infinity;
  let max = -Infinity;
  for (const value of values) {
    if (value < min) min = value;
    if (value > max) max = value;
  }
  if (!Number.isFinite(min) || !Number.isFinite(max) || min === max) {
    return [{ start: min, end: max, count: values.length }];
  }
  const bins =
    binCount ?? Math.max(4, Math.min(20, Math.ceil(Math.log2(values.length) + 1)));
  const width = (max - min) / bins;
  const result: HistogramBin[] = Array.from({ length: bins }, (_, i) => ({
    start: min + i * width,
    end: min + (i + 1) * width,
    count: 0,
  }));
  for (const value of values) {
    let idx = Math.floor((value - min) / width);
    if (idx >= bins) {
      idx = bins - 1;
    }
    if (idx < 0) {
      idx = 0;
    }
    result[idx].count += 1;
  }
  return result;
}
