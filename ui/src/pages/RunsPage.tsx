import { useEffect, useState } from "react";
import { Link } from "react-router-dom";
import { listRuns } from "../api";
import type { RunManifest } from "../types";

export function RunsPage() {
  const [runs, setRuns] = useState<RunManifest[]>([]);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    listRuns()
      .then(setRuns)
      .catch((fetchError: Error) => setError(fetchError.message));
  }, []);

  return (
    <main className="page">
      <header className="header">
        <div>
          <p className="eyebrow">Space Sim</p>
          <h1>RL Rollout Runs</h1>
        </div>
        <p className="muted">
          Headless episodes from the Rust runner. Start the API with{" "}
          <code>cargo run --bin runner -- serve</code>.
        </p>
      </header>

      {error ? <p className="error">{error}</p> : null}

      <table className="runs-table">
        <thead>
          <tr>
            <th>Run</th>
            <th>Scenario</th>
            <th>Episodes</th>
            <th>Created</th>
          </tr>
        </thead>
        <tbody>
          {runs.map((run) => (
            <tr key={run.run_id}>
              <td>
                <Link to={`/runs/${run.run_id}/episodes/0`}>{run.run_id}</Link>
              </td>
              <td>{run.scenario_name}</td>
              <td>{run.episode_count}</td>
              <td>{run.created_at}</td>
            </tr>
          ))}
        </tbody>
      </table>
    </main>
  );
}
