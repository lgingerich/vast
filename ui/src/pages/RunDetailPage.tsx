import { useEffect, useMemo, useState } from "react";
import { ArrowLeft, GitCompareArrows, Play } from "lucide-react";
import { Link, useNavigate, useParams } from "react-router-dom";
import { getMetrics, getRun } from "../api";
import { RunKindBadge } from "../components/RunMeta";
import { BarChart, DonutChart, HistogramChart, LineChart } from "../components/charts";
import type { EpisodeMetric, RunManifest } from "../types";
import { isMissionRun } from "../types";
import { histogram, summarizeMetrics } from "../viz/metrics";
import { formatAbsolute, formatRelative, parseCreatedAt } from "../viz/time";

const STOP_REASON_COLORS = [
  "#60a5fa",
  "#f97316",
  "#34d399",
  "#fbbf24",
  "#a78bfa",
  "#fb7185",
  "#22d3ee",
];

function formatPercent(value: number | null): string {
  if (value === null) {
    return "—";
  }
  return `${(value * 100).toFixed(0)}%`;
}

function formatNumber(value: number, fractionDigits = 2): string {
  if (!Number.isFinite(value)) {
    return "—";
  }
  return value.toFixed(fractionDigits);
}

function ScenarioSummary({ manifest }: { manifest: RunManifest }) {
  const fields: { label: string; value: string }[] = [];
  if (manifest.scenario) {
    const s = manifest.scenario;
    fields.push(
      { label: "Integrator", value: s.integrator },
      { label: "Force model", value: s.force_model },
      { label: "dt", value: `${s.dt_s.toFixed(2)} s` },
      { label: "Duration", value: `${s.duration_s.toFixed(0)} s` },
      { label: "Altitude", value: `${(s.altitude_m / 1000).toFixed(1)} km` },
      { label: "Sample every", value: `${s.sample_every_steps} steps` },
    );
  } else if (manifest.mission) {
    const m = manifest.mission;
    fields.push(
      { label: "Kind", value: m.kind },
      { label: "Integrator", value: m.propagation.integrator },
      { label: "Force model", value: m.propagation.force_model },
      { label: "dt", value: `${m.propagation.dt_s.toFixed(2)} s` },
      { label: "Sample every", value: `${m.propagation.sample_every_steps} steps` },
      { label: "Segments", value: m.segments.length.toString() },
    );
    const orbit = m.initial_orbit;
    fields.push({ label: "Initial orbit", value: orbit.kind });
    if (orbit.altitude_m !== undefined) {
      fields.push({ label: "Altitude", value: `${(orbit.altitude_m / 1000).toFixed(1)} km` });
    }
    if (orbit.semi_major_axis_m !== undefined) {
      fields.push({
        label: "SMA",
        value: `${(orbit.semi_major_axis_m / 1000).toFixed(1)} km`,
      });
    }
    if (orbit.inclination_deg !== undefined) {
      fields.push({ label: "Inclination", value: `${orbit.inclination_deg.toFixed(2)}°` });
    }
  }

  return (
    <dl className="scenario-summary">
      {fields.map((field) => (
        <div key={field.label}>
          <dt>{field.label}</dt>
          <dd>{field.value}</dd>
        </div>
      ))}
    </dl>
  );
}

export function RunDetailPage() {
  const { runId = "" } = useParams();
  const navigate = useNavigate();
  const [manifest, setManifest] = useState<RunManifest | null>(null);
  const [metrics, setMetrics] = useState<EpisodeMetric[]>([]);
  const [selection, setSelection] = useState<Set<number>>(new Set());
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    let cancelled = false;
    getRun(runId)
      .then((value) => {
        if (!cancelled) setManifest(value);
      })
      .catch((err: Error) => {
        if (!cancelled) setError(err.message);
      });
    getMetrics(runId)
      .then((value) => {
        if (!cancelled) setMetrics(value);
      })
      .catch(() => {
        // Metrics may not exist (e.g. live API returned 404). Leave empty.
      });
    return () => {
      cancelled = true;
    };
  }, [runId]);

  const summary = useMemo(() => summarizeMetrics(metrics), [metrics]);
  const hasRlSignal = useMemo(
    () => metrics.some((m) => m.reward !== 0),
    [metrics],
  );
  const rewards = useMemo(() => metrics.map((m) => m.reward), [metrics]);
  const steps = useMemo(() => metrics.map((m) => m.steps), [metrics]);
  const altitudeKm = useMemo(
    () => metrics.map((m) => m.final_altitude_m / 1000),
    [metrics],
  );
  const altitudeBins = useMemo(() => histogram(altitudeKm), [altitudeKm]);
  const stopReasonLegend = useMemo(
    () =>
      summary.stopReasons.map((entry, index) => ({
        ...entry,
        color: STOP_REASON_COLORS[index % STOP_REASON_COLORS.length],
      })),
    [summary.stopReasons],
  );
  const createdAt = manifest ? parseCreatedAt(manifest.created_at) : null;

  const toggleSelection = (index: number) => {
    setSelection((prev) => {
      const next = new Set(prev);
      if (next.has(index)) {
        next.delete(index);
      } else {
        next.add(index);
      }
      return next;
    });
  };

  const handleCompare = () => {
    if (selection.size === 0) return;
    const ids = [...selection].sort((a, b) => a - b).join(",");
    navigate(`/runs/${runId}/compare?episodes=${ids}`);
  };

  const selectionCount = selection.size;

  return (
    <main className="page">
      <header className="page-header">
        <div className="page-header-main">
          <p className="eyebrow">
            <Link to="/">
              <ArrowLeft
                size={11}
                style={{ verticalAlign: "middle", marginRight: "0.2rem" }}
                aria-hidden
              />
              Runs
            </Link>
            {" / "}
            <span className="mono">{runId}</span>
          </p>
          <div className="title-row" style={{ marginTop: "0.3rem" }}>
            <h1>{manifest?.scenario_name ?? runId}</h1>
            {manifest ? <RunKindBadge isMission={isMissionRun(manifest)} /> : null}
          </div>
          {createdAt ? (
            <p className="muted" style={{ marginTop: "0.4rem", fontSize: "0.8125rem" }}>
              Created {formatRelative(createdAt)} · {formatAbsolute(createdAt)}
            </p>
          ) : null}
        </div>
        <div className="page-header-meta">
          {manifest ? (
            <Link to={`/runs/${runId}/episodes/0`} className="row-link">
              <Play size={12} strokeWidth={2} aria-hidden />
              Replay first episode
            </Link>
          ) : null}
        </div>
      </header>

      {error ? <p className="error" style={{ marginBottom: "var(--space-4)" }}>{error}</p> : null}

      <section className="kpi-strip">
        <div className="kpi">
          <div className="kpi-label">Episodes</div>
          <div className="kpi-value">{summary.count}</div>
          <div className="kpi-trailing">{summary.totalSteps.toLocaleString()} total steps</div>
        </div>
        {hasRlSignal ? (
          <>
            <div className="kpi">
              <div className="kpi-label">Mean reward</div>
              <div className="kpi-value">{formatNumber(summary.reward.mean, 3)}</div>
              <div className="kpi-trailing">
                σ {formatNumber(summary.reward.std, 3)} · min {formatNumber(summary.reward.min, 2)}
              </div>
            </div>
            <div className="kpi">
              <div className="kpi-label">Success rate</div>
              <div className="kpi-value">{formatPercent(summary.successRate)}</div>
              <div className="kpi-trailing">
                {summary.stopReasons.slice(0, 1).map((r) => `${r.reason}: ${r.count}`).join("")}
              </div>
            </div>
          </>
        ) : null}
        <div className="kpi">
          <div className="kpi-label">Mean steps</div>
          <div className="kpi-value">{formatNumber(summary.steps.mean, 0)}</div>
          <div className="kpi-trailing">
            {summary.steps.min} – {summary.steps.max} per episode
          </div>
        </div>
      </section>

      <section className="card">
        <div className="card-header">
          <span className="card-title">Scenario</span>
          {manifest?.mission ? (
            <span className="card-subtitle">{manifest.mission.segments.length} segments</span>
          ) : null}
        </div>
        {manifest ? <ScenarioSummary manifest={manifest} /> : <p className="muted">Loading…</p>}
      </section>

      {hasRlSignal ? (
        <section className="card">
          <div className="card-header">
            <span className="card-title">Training signal</span>
            <span className="card-subtitle">per-episode metrics</span>
          </div>
          <div className="chart-grid">
            <div className="chart-card">
              <div className="chart-title">Reward per episode</div>
              <BarChart
                values={rewards}
                labels={metrics.map((m) => m.episode_index)}
                color="#60a5fa"
                yLabel="reward"
              />
            </div>
            <div className="chart-card">
              <div className="chart-title">Episode length</div>
              <LineChart values={steps} color="#22d3ee" yLabel="steps" />
            </div>
            <div className="chart-card">
              <div className="chart-title">Final altitude distribution</div>
              <HistogramChart bins={altitudeBins} color="#a78bfa" unit="km" />
            </div>
            <div className="chart-card">
              <div className="chart-title">Stop reasons</div>
              <div className="donut-grid">
                <DonutChart
                  entries={stopReasonLegend.map((entry) => ({
                    label: entry.reason,
                    value: entry.count,
                    color: entry.color,
                  }))}
                />
                <div className="donut-legend">
                  {stopReasonLegend.map((entry) => (
                    <div key={entry.reason} className="donut-legend-row">
                      <span className="swatch" style={{ background: entry.color }} />
                      <span>{entry.reason}</span>
                      <span className="count">{entry.count}</span>
                    </div>
                  ))}
                </div>
              </div>
            </div>
          </div>
        </section>
      ) : null}

      <section className="card">
        <div className="card-header">
          <span className="card-title">Episodes</span>
          <span className="card-subtitle">
            {metrics.length || manifest?.episode_count || 0} total
          </span>
        </div>
        <div className="episode-table-wrap">
          <table className="episode-table">
            <thead>
              <tr>
                <th className="selection">
                  <span className="sr-only">Select</span>
                </th>
                <th>#</th>
                <th>Seed</th>
                <th>Steps</th>
                <th>Stop reason</th>
                <th>Final alt (km)</th>
                {hasRlSignal ? <th>Reward</th> : null}
                <th aria-label="Actions" />
              </tr>
            </thead>
            <tbody>
              {metrics.length === 0 ? (
                <tr>
                  <td colSpan={hasRlSignal ? 8 : 7}>
                    <p className="empty" style={{ padding: "var(--space-4)" }}>
                      No per-episode metrics. Use the replay link to open episode 0.
                    </p>
                  </td>
                </tr>
              ) : (
                metrics.map((metric) => {
                  const replayPath = `/runs/${runId}/episodes/${metric.episode_index}`;
                  return (
                    <tr
                      key={metric.episode_index}
                      className="row-link-tr"
                      onClick={() => navigate(replayPath)}
                      onKeyDown={(event) => {
                        if (event.key === "Enter" || event.key === " ") {
                          event.preventDefault();
                          navigate(replayPath);
                        }
                      }}
                      tabIndex={0}
                      role="link"
                      aria-label={`Replay episode ${metric.episode_index}`}
                    >
                      <td
                        className="selection"
                        onClick={(event) => event.stopPropagation()}
                      >
                        <input
                          type="checkbox"
                          checked={selection.has(metric.episode_index)}
                          onChange={() => toggleSelection(metric.episode_index)}
                          aria-label={`Select episode ${metric.episode_index}`}
                        />
                      </td>
                      <td className="numeric">{metric.episode_index}</td>
                      <td className="numeric">{metric.seed}</td>
                      <td className="numeric">{metric.steps}</td>
                      <td>
                        <span className="chip">{metric.stop_reason}</span>
                      </td>
                      <td className="numeric">{(metric.final_altitude_m / 1000).toFixed(1)}</td>
                      {hasRlSignal ? (
                        <td className="numeric">{metric.reward.toFixed(3)}</td>
                      ) : null}
                      <td>
                        <div className="row-actions">
                          <Link
                            to={replayPath}
                            className="row-link"
                            onClick={(event) => event.stopPropagation()}
                          >
                            <Play size={11} strokeWidth={2} aria-hidden />
                            Replay
                          </Link>
                        </div>
                      </td>
                    </tr>
                  );
                })
              )}
            </tbody>
          </table>
          <div className="episode-actions-bar">
            <span>
              {selectionCount === 0
                ? "Select two or more episodes to compare"
                : `${selectionCount} selected`}
            </span>
            <button
              type="button"
              onClick={handleCompare}
              disabled={selectionCount < 2}
              title="Open compare view"
            >
              <GitCompareArrows
                size={14}
                strokeWidth={1.75}
                aria-hidden
                style={{ marginRight: "0.3rem", verticalAlign: "middle" }}
              />
              Compare
            </button>
          </div>
        </div>
      </section>
    </main>
  );
}
