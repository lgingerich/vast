import { useEffect, useMemo, useState } from "react";
import { Link, useParams } from "react-router-dom";
import {
  altitudeMeters,
  EARTH_RADIUS_M,
  getRun,
  getTrajectory,
  speedMetersPerSecond,
} from "../api";
import { OrbitReplay } from "../components/OrbitReplay";
import type { RunManifest, TrajectoryPoint } from "../types";

export function ReplayPage() {
  const { runId = "", episodeIndex = "0" } = useParams();
  const episode = Number.parseInt(episodeIndex, 10);
  const [manifest, setManifest] = useState<RunManifest | null>(null);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const [frameIndex, setFrameIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    Promise.all([getRun(runId), getTrajectory(runId, episode)])
      .then(([runManifest, points]) => {
        setManifest(runManifest);
        setTrajectory(points);
        setFrameIndex(0);
      })
      .catch((fetchError: Error) => setError(fetchError.message));
  }, [runId, episode]);

  useEffect(() => {
    if (!playing || trajectory.length === 0) {
      return;
    }
    const timer = window.setInterval(() => {
      setFrameIndex((current) =>
        current + 1 >= trajectory.length ? 0 : current + 1,
      );
    }, 100);
    return () => window.clearInterval(timer);
  }, [playing, trajectory.length]);

  const current = trajectory[frameIndex];
  const stats = useMemo(() => {
    if (!current) {
      return null;
    }
    return {
      epoch: current.t,
      altitudeKm: altitudeMeters(current, EARTH_RADIUS_M) / 1000,
      speedKmS: speedMetersPerSecond(current) / 1000,
    };
  }, [current]);

  return (
    <main className="page replay-page">
      <header className="header">
        <div>
          <p className="eyebrow">
            <Link to="/">Runs</Link> / {runId} / episode {episode}
          </p>
          <h1>{manifest?.scenario_name ?? "Episode Replay"}</h1>
        </div>
      </header>

      {error ? <p className="error">{error}</p> : null}

      <div className="replay-layout">
        <section className="viewport">
          <OrbitReplay trajectory={trajectory} frameIndex={frameIndex} />
        </section>
        <aside className="panel">
          <h2>Telemetry</h2>
          {stats ? (
            <dl className="stats">
              <div>
                <dt>Epoch (s)</dt>
                <dd>{stats.epoch.toFixed(1)}</dd>
              </div>
              <div>
                <dt>Altitude (km)</dt>
                <dd>{stats.altitudeKm.toFixed(2)}</dd>
              </div>
              <div>
                <dt>Speed (km/s)</dt>
                <dd>{stats.speedKmS.toFixed(3)}</dd>
              </div>
              <div>
                <dt>Frame</dt>
                <dd>
                  {frameIndex + 1} / {trajectory.length}
                </dd>
              </div>
            </dl>
          ) : (
            <p className="muted">Loading trajectory…</p>
          )}

          <div className="controls">
            <button type="button" onClick={() => setPlaying((value) => !value)}>
              {playing ? "Pause" : "Play"}
            </button>
            <input
              type="range"
              min={0}
              max={Math.max(trajectory.length - 1, 0)}
              value={frameIndex}
              onChange={(event) => setFrameIndex(Number(event.target.value))}
            />
          </div>
        </aside>
      </div>
    </main>
  );
}
