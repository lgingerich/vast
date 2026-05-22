import { Canvas } from "@react-three/fiber";
import { Suspense, useCallback, useEffect, useMemo, useRef, useState } from "react";
import { ArrowLeft } from "lucide-react";
import { Link, useParams, useSearchParams } from "react-router-dom";
import { getRun, getTrajectory } from "../api";
import { CameraToolbar } from "../components/CameraToolbar";
import { ReplayControls } from "../components/ReplayControls";
import { RunKindBadge } from "../components/RunMeta";
import { CompareScene, type CompareEntry } from "../components/scene/CompareScene";
import type { CameraMode } from "../components/scene/SceneCameraRig";
import { useKeyboardShortcuts } from "../hooks/useKeyboardShortcuts";
import type { RunManifest } from "../types";
import { isMissionRun } from "../types";
import { EARTH_RADIUS_M, EPISODE_COLORS } from "../viz/constants";
import { altitudeMeters, speedMetersPerSecond } from "../viz/coordinates";
import { useCompactLayout } from "../viz/useCompactLayout";

const BASE_FRAME_INTERVAL_MS = 100;

function parseEpisodes(value: string | null): number[] {
  if (!value) {
    return [];
  }
  return value
    .split(",")
    .map((token) => Number.parseInt(token.trim(), 10))
    .filter((index) => Number.isFinite(index) && index >= 0);
}

function combinedBounds(entries: CompareEntry[]): {
  center: [number, number, number];
  radius: number;
} | null {
  let minX = Infinity;
  let maxX = -Infinity;
  let minY = Infinity;
  let maxY = -Infinity;
  let minZ = Infinity;
  let maxZ = -Infinity;
  let count = 0;
  for (const entry of entries) {
    for (const point of entry.trajectory) {
      const [x, y, z] = point.pos;
      if (x < minX) minX = x;
      if (x > maxX) maxX = x;
      if (y < minY) minY = y;
      if (y > maxY) maxY = y;
      if (z < minZ) minZ = z;
      if (z > maxZ) maxZ = z;
      count += 1;
    }
  }
  if (count === 0) {
    return null;
  }
  const center: [number, number, number] = [
    (minX + maxX) / 2,
    (minY + maxY) / 2,
    (minZ + maxZ) / 2,
  ];
  const radius = Math.max(
    Math.hypot(maxX - center[0], maxY - center[1], maxZ - center[2]),
    EARTH_RADIUS_M,
  );
  return { center, radius };
}

export function CompareReplayPage() {
  const { runId = "" } = useParams();
  const [params] = useSearchParams();
  const stageRef = useRef<HTMLElement>(null);

  const episodes = useMemo(() => parseEpisodes(params.get("episodes")), [params]);

  const [manifest, setManifest] = useState<RunManifest | null>(null);
  const [entries, setEntries] = useState<CompareEntry[]>([]);
  const [error, setError] = useState<string | null>(null);
  const [frameIndex, setFrameIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [cameraMode, setCameraMode] = useState<CameraMode>("overview");
  const [showGrid, setShowGrid] = useState(true);
  const [showTriad, setShowTriad] = useState(false);
  const [followEpisode, setFollowEpisode] = useState<number | null>(null);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [refitCounter, setRefitCounter] = useState(0);
  const compact = useCompactLayout();

  useEffect(() => {
    let cancelled = false;
    setError(null);
    getRun(runId)
      .then((value) => {
        if (!cancelled) setManifest(value);
      })
      .catch((err: Error) => {
        if (!cancelled) setError(err.message);
      });
    Promise.all(
      episodes.map(async (episodeIndex, position) => {
        const trajectory = await getTrajectory(runId, episodeIndex);
        return {
          episodeIndex,
          trajectory,
          color: EPISODE_COLORS[position % EPISODE_COLORS.length],
        } satisfies CompareEntry;
      }),
    )
      .then((value) => {
        if (cancelled) return;
        setEntries(value);
        setFrameIndex(0);
        setPlaying(false);
      })
      .catch((err: Error) => {
        if (!cancelled) setError(err.message);
      });
    return () => {
      cancelled = true;
    };
  }, [runId, episodes]);

  const frameCount = useMemo(
    () => entries.reduce((max, entry) => Math.max(max, entry.trajectory.length), 0),
    [entries],
  );
  const endTimeS = useMemo(
    () =>
      entries.reduce((max, entry) => {
        const last = entry.trajectory[entry.trajectory.length - 1];
        return last ? Math.max(max, last.t) : max;
      }, 0),
    [entries],
  );
  const currentTimeS = useMemo(() => {
    const first = entries[0]?.trajectory;
    if (!first) return 0;
    return first[Math.min(frameIndex, first.length - 1)]?.t ?? 0;
  }, [entries, frameIndex]);

  const bounds = useMemo(() => combinedBounds(entries), [entries]);
  const fitKey = `${runId}:compare:${episodes.join("-")}:${entries.length}:${refitCounter}`;

  const handleFit = useCallback(() => {
    setCameraMode("overview");
    setRefitCounter((value) => value + 1);
  }, []);

  const stepFrame = useCallback(
    (delta: number) => {
      setFrameIndex((currentIndex) => {
        const next = currentIndex + delta;
        if (next < 0) return 0;
        if (next >= frameCount) return Math.max(frameCount - 1, 0);
        return next;
      });
    },
    [frameCount],
  );

  useEffect(() => {
    if (!playing || frameCount === 0) {
      return;
    }
    const timer = window.setInterval(() => {
      setFrameIndex((currentIndex) => {
        if (currentIndex + 1 >= frameCount) {
          return 0;
        }
        return currentIndex + 1;
      });
    }, BASE_FRAME_INTERVAL_MS / playbackSpeed);
    return () => window.clearInterval(timer);
  }, [playing, frameCount, playbackSpeed]);

  const toggleFullscreen = useCallback(async () => {
    const stage = stageRef.current;
    if (!stage) return;
    if (document.fullscreenElement === stage) {
      await document.exitFullscreen();
      return;
    }
    await stage.requestFullscreen();
  }, []);

  useKeyboardShortcuts(
    [
      { code: "Space", handler: () => setPlaying((value) => !value) },
      { code: "ArrowLeft", handler: () => stepFrame(-1) },
      { code: "ArrowRight", handler: () => stepFrame(1) },
      { code: "KeyH", handler: handleFit },
      { code: "KeyF", handler: () => void toggleFullscreen() },
    ],
    [stepFrame, toggleFullscreen, handleFit],
  );

  useEffect(() => {
    const onFullscreenChange = () => {
      setIsFullscreen(document.fullscreenElement === stageRef.current);
    };
    document.addEventListener("fullscreenchange", onFullscreenChange);
    return () => document.removeEventListener("fullscreenchange", onFullscreenChange);
  }, []);

  // When the user selects "Chase", follow the first episode by default.
  useEffect(() => {
    if (cameraMode !== "spacecraft") {
      return;
    }
    if (followEpisode === null && entries.length > 0) {
      setFollowEpisode(entries[0].episodeIndex);
    }
  }, [cameraMode, entries, followEpisode]);

  const chips = useMemo(() => {
    return entries.map((entry) => {
      const safeIndex = Math.min(frameIndex, entry.trajectory.length - 1);
      const point = entry.trajectory[safeIndex];
      const altitudeKm = point
        ? altitudeMeters(point, EARTH_RADIUS_M) / 1000
        : null;
      const speedKmS = point ? speedMetersPerSecond(point) / 1000 : null;
      return {
        episodeIndex: entry.episodeIndex,
        color: entry.color,
        length: entry.trajectory.length,
        altitudeKm,
        speedKmS,
        isFollowed:
          cameraMode === "spacecraft" && followEpisode === entry.episodeIndex,
      };
    });
  }, [entries, frameIndex, followEpisode, cameraMode]);

  return (
    <main className={`replay-shell${compact ? " compact" : ""}`}>
      <header className="replay-toolbar">
        <div className="replay-toolbar-main">
          <p className="eyebrow">
            <Link to="/">
              <ArrowLeft size={11} style={{ verticalAlign: "middle", marginRight: "0.2rem" }} />
              Runs
            </Link>
            {" / "}
            <Link to={`/runs/${runId}`}>{runId}</Link>
            {" / compare"}
          </p>
          <div className="title-row">
            <h1>
              Compare episodes{" "}
              <span style={{ color: "var(--text-2)", fontWeight: 400, fontSize: "0.95rem" }}>
                {episodes.join(", ")}
              </span>
            </h1>
            {manifest ? <RunKindBadge isMission={isMissionRun(manifest)} /> : null}
          </div>
        </div>
      </header>

      {error ? <p className="replay-error">{error}</p> : null}

      <section ref={stageRef} className="replay-stage">
        <Canvas camera={{ position: [0, 8, 14], fov: 45 }} dpr={[1, 2]}>
          <Suspense fallback={null}>
            <CompareScene
              entries={entries}
              frameIndex={frameIndex}
              manifest={manifest}
              cameraMode={cameraMode}
              fitKey={fitKey}
              overviewCenter={bounds?.center ?? null}
              overviewRadiusM={bounds?.radius ?? EARTH_RADIUS_M * 2}
              showGrid={showGrid}
              showTriad={showTriad}
              followEpisode={followEpisode}
            />
          </Suspense>
        </Canvas>

        <div className="compare-strip">
          {chips.map((chip) => (
            <button
              key={chip.episodeIndex}
              type="button"
              className="compare-chip"
              onClick={() => setFollowEpisode(chip.episodeIndex)}
              style={{
                borderColor: chip.isFollowed ? chip.color : undefined,
                boxShadow: chip.isFollowed ? `inset 0 0 0 1px ${chip.color}` : undefined,
                pointerEvents: "auto",
              }}
              title={
                cameraMode === "spacecraft"
                  ? "Chase this episode"
                  : `Episode ${chip.episodeIndex}`
              }
            >
              <span className="swatch" style={{ background: chip.color }} />
              <span className="episode-label">EP {chip.episodeIndex}</span>
              <span className="alt">
                {chip.altitudeKm !== null ? chip.altitudeKm.toFixed(1) : "—"}
                <span style={{ color: "var(--text-muted)" }}> km</span>
              </span>
              <span className="spd">
                {chip.speedKmS !== null ? chip.speedKmS.toFixed(2) : "—"}
                <span style={{ color: "var(--text-muted)" }}> km/s</span>
              </span>
            </button>
          ))}
        </div>

        <CameraToolbar
          mode={cameraMode}
          onModeChange={setCameraMode}
          onFit={handleFit}
          onToggleFullscreen={toggleFullscreen}
          isFullscreen={isFullscreen}
          showGrid={showGrid}
          onToggleGrid={() => setShowGrid((value) => !value)}
          showTriad={showTriad}
          onToggleTriad={() => setShowTriad((value) => !value)}
        />

        <div className="instrumentation-dock">
          <section className="dock-panel" aria-label="Replay controls" style={{ gridColumn: "1 / -1" }}>
            <div className="dock-panel-header">
              <span>Synchronized replay</span>
              <span>
                {entries.length} {entries.length === 1 ? "episode" : "episodes"} ·{" "}
                {(playbackSpeed === 1 ? "1.00" : playbackSpeed.toFixed(2))} x
              </span>
            </div>
            <ReplayControls
              frameIndex={frameIndex}
              frameCount={frameCount}
              currentTimeS={currentTimeS}
              endTimeS={endTimeS}
              playing={playing}
              playbackSpeed={playbackSpeed}
              segmentRanges={[]}
              onTogglePlay={() => setPlaying((value) => !value)}
              onFrameChange={setFrameIndex}
              onPlaybackSpeedChange={setPlaybackSpeed}
              onStep={stepFrame}
            />
          </section>
        </div>
      </section>
    </main>
  );
}
