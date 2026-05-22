import { useEffect, useMemo, useState } from "react";
import { ExternalLink, Search } from "lucide-react";
import { Link, useNavigate } from "react-router-dom";
import { listRuns } from "../api";
import { RunKindBadge } from "../components/RunMeta";
import type { RunManifest } from "../types";
import { isMissionRun } from "../types";
import { formatAbsolute, formatRelative, parseCreatedAt } from "../viz/time";

interface RunWithCreatedAt {
  manifest: RunManifest;
  createdAt: Date | null;
}

function attachCreatedAt(runs: RunManifest[]): RunWithCreatedAt[] {
  return runs.map((manifest) => ({
    manifest,
    createdAt: parseCreatedAt(manifest.created_at),
  }));
}

export function RunsPage() {
  const navigate = useNavigate();
  const [runs, setRuns] = useState<RunManifest[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [query, setQuery] = useState("");

  useEffect(() => {
    listRuns()
      .then(setRuns)
      .catch((fetchError: Error) => setError(fetchError.message));
  }, []);

  const enriched = useMemo(() => attachCreatedAt(runs), [runs]);

  const stats = useMemo(() => {
    const totals = { rollouts: 0, missions: 0, episodes: 0 };
    let latest: Date | null = null;
    for (const { manifest, createdAt } of enriched) {
      if (isMissionRun(manifest)) {
        totals.missions += 1;
      } else {
        totals.rollouts += 1;
      }
      totals.episodes += manifest.episode_count;
      if (createdAt && (!latest || createdAt > latest)) {
        latest = createdAt;
      }
    }
    return { ...totals, total: enriched.length, latest };
  }, [enriched]);

  const filtered = useMemo(() => {
    const sorted = [...enriched].sort((a, b) => {
      const ta = a.createdAt?.getTime() ?? 0;
      const tb = b.createdAt?.getTime() ?? 0;
      return tb - ta;
    });
    const trimmed = query.trim().toLowerCase();
    if (trimmed === "") {
      return sorted;
    }
    return sorted.filter(({ manifest }) =>
      [manifest.run_id, manifest.scenario_name]
        .join(" ")
        .toLowerCase()
        .includes(trimmed),
    );
  }, [enriched, query]);

  return (
    <main className="page">
      <header className="page-header">
        <div className="page-header-main">
          <p className="eyebrow">Space Sim</p>
          <h1>Simulation Runs</h1>
          <p className="muted" style={{ marginTop: "0.4rem" }}>
            Replay rollouts and mission artifacts from the Rust runner. Start the API with{" "}
            <code>cargo run --bin runner -- serve</code>.
          </p>
        </div>
      </header>

      {error ? <p className="error" style={{ marginBottom: "var(--space-4)" }}>{error}</p> : null}

      <section className="kpi-strip" aria-label="Run summary">
        <div className="kpi">
          <div className="kpi-label">Total runs</div>
          <div className="kpi-value">{stats.total}</div>
          <div className="kpi-trailing">
            {stats.rollouts} rollouts · {stats.missions} missions
          </div>
        </div>
        <div className="kpi">
          <div className="kpi-label">Episodes</div>
          <div className="kpi-value">{stats.episodes}</div>
          <div className="kpi-trailing">across all runs</div>
        </div>
        <div className="kpi">
          <div className="kpi-label">Last activity</div>
          <div className="kpi-value" style={{ fontSize: "1rem" }}>
            {stats.latest ? formatRelative(stats.latest) : "—"}
          </div>
          <div className="kpi-trailing">
            {stats.latest ? formatAbsolute(stats.latest) : "no runs yet"}
          </div>
        </div>
      </section>

      <section className="card" aria-label="Runs">
        <div className="card-header">
          <span className="card-title">All runs</span>
          <span className="card-subtitle">
            {filtered.length} {filtered.length === 1 ? "result" : "results"}
          </span>
        </div>

        <div className="runs-toolbar">
          <label className="runs-search">
            <Search size={14} strokeWidth={1.75} aria-hidden />
            <input
              type="search"
              placeholder="Search run id or scenario"
              value={query}
              onChange={(event) => setQuery(event.target.value)}
              aria-label="Search runs"
            />
          </label>
        </div>

        <div className="runs-table-wrap">
          {filtered.length === 0 ? (
            <p className="empty">
              {query ? "No runs match your search." : "No runs available yet."}
            </p>
          ) : (
            <table className="runs-table">
              <thead>
                <tr>
                  <th>Run</th>
                  <th>Kind</th>
                  <th>Scenario</th>
                  <th>Episodes</th>
                  <th>Created</th>
                  <th aria-label="Actions" />
                </tr>
              </thead>
              <tbody>
                {filtered.map(({ manifest, createdAt }) => {
                  const detailPath = `/runs/${manifest.run_id}`;
                  const replayPath = `/runs/${manifest.run_id}/episodes/0`;
                  return (
                    <tr
                      key={manifest.run_id}
                      className="row-link-tr"
                      onClick={() => navigate(detailPath)}
                      onKeyDown={(event) => {
                        if (event.key === "Enter" || event.key === " ") {
                          event.preventDefault();
                          navigate(detailPath);
                        }
                      }}
                      tabIndex={0}
                      role="link"
                      aria-label={`Open run ${manifest.run_id}`}
                    >
                      <td className="run-id">{manifest.run_id}</td>
                      <td>
                        <RunKindBadge isMission={isMissionRun(manifest)} />
                      </td>
                      <td>
                        <span className="chip scenario">{manifest.scenario_name}</span>
                      </td>
                      <td>
                        <span className="chip episodes">{manifest.episode_count}</span>
                      </td>
                      <td>
                        <div className="created-cell" title={createdAt ? formatAbsolute(createdAt) : ""}>
                          <span className="created-relative">
                            {createdAt ? formatRelative(createdAt) : "—"}
                          </span>
                          <span className="created-absolute">
                            {createdAt ? formatAbsolute(createdAt) : manifest.created_at}
                          </span>
                        </div>
                      </td>
                      <td>
                        <div className="row-actions">
                          <Link
                            to={replayPath}
                            className="row-link"
                            onClick={(event) => event.stopPropagation()}
                          >
                            Replay
                            <ExternalLink size={12} strokeWidth={1.75} aria-hidden />
                          </Link>
                        </div>
                      </td>
                    </tr>
                  );
                })}
              </tbody>
            </table>
          )}
        </div>
      </section>
    </main>
  );
}
