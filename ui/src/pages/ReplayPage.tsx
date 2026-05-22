import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { Link, useParams } from "react-router-dom";
import { getRun, getTrajectory } from "../api";
import type { CameraMode } from "../components/scene/SceneCameraRig";
import { CameraToolbar } from "../components/CameraToolbar";
import { InstrumentationDock } from "../components/InstrumentationDock";
import { OrbitReplay } from "../components/OrbitReplay";
import { EpisodePicker, RunKindBadge } from "../components/RunMeta";
import { ViewportHud } from "../components/ViewportHud";
import { useKeyboardShortcuts } from "../hooks/useKeyboardShortcuts";
import type { RunManifest, TrajectoryPoint } from "../types";
import { isMissionRun } from "../types";
import { frameInstrumentation, trajectoryBounds } from "../viz/instrumentation";
import {
  hasSegmentMetadata,
  segmentRanges,
  telemetrySeries,
} from "../viz/trajectory";
import { EARTH_RADIUS_M } from "../viz/constants";
import { useCompactLayout } from "../viz/useCompactLayout";

const BASE_FRAME_INTERVAL_MS = 100;

export function ReplayPage() {
  const { runId = "", episodeIndex = "0" } = useParams();
  const episode = Number.parseInt(episodeIndex, 10);
  const stageRef = useRef<HTMLElement>(null);

  const [manifest, setManifest] = useState<RunManifest | null>(null);
  const [trajectory, setTrajectory] = useState<TrajectoryPoint[]>([]);
  const [frameIndex, setFrameIndex] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [playbackSpeed, setPlaybackSpeed] = useState(1);
  const [cameraMode, setCameraMode] = useState<CameraMode>("overview");
  const [showGrid, setShowGrid] = useState(true);
  const [showTriad, setShowTriad] = useState(false);
  const [refitCounter, setRefitCounter] = useState(0);
  const compact = useCompactLayout();
  const [dockCollapsed, setDockCollapsed] = useState(compact);
  const [isFullscreen, setIsFullscreen] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    Promise.all([getRun(runId), getTrajectory(runId, episode)])
      .then(([runManifest, points]) => {
        setManifest(runManifest);
        setTrajectory(points);
        setFrameIndex(0);
        setPlaying(false);
        setCameraMode("overview");
      })
      .catch((fetchError: Error) => setError(fetchError.message));
  }, [runId, episode]);

  useEffect(() => {
    if (compact) {
      setDockCollapsed(true);
    }
  }, [compact]);

  const frameCount = trajectory.length;
  const endTimeS = trajectory[trajectory.length - 1]?.t ?? 0;
  const current = trajectory[frameIndex];
  const segmentMetadata = hasSegmentMetadata(trajectory);
  const ranges = useMemo(() => segmentRanges(trajectory), [trajectory]);
  const series = useMemo(
    () => telemetrySeries(trajectory, EARTH_RADIUS_M),
    [trajectory],
  );
  const altitudeKm = useMemo(() => series.map((p) => p.altitudeKm), [series]);
  const speedKmS = useMemo(() => series.map((p) => p.speedKmS), [series]);
  const specificEnergyMJPerKg = useMemo(
    () => series.map((p) => p.specificEnergyMJPerKg),
    [series],
  );
  const timesS = useMemo(() => series.map((p) => p.timeS), [series]);
  const bounds = useMemo(() => trajectoryBounds(trajectory), [trajectory]);
  const instrumentation = useMemo(
    () => (current ? frameInstrumentation(current) : null),
    [current],
  );
  const fitKey = `${runId}:${episode}:${trajectory.length}:${refitCounter}`;

  const handleFit = useCallback(() => {
    setCameraMode("overview");
    setRefitCounter((value) => value + 1);
  }, []);

  const stepFrame = useCallback(
    (delta: number) => {
      setFrameIndex((currentIndex) => {
        const next = currentIndex + delta;
        if (next < 0) {
          return 0;
        }
        if (next >= frameCount) {
          return Math.max(frameCount - 1, 0);
        }
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
    if (!stage) {
      return;
    }
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

  return (
    <main className={`replay-shell${compact ? " compact" : ""}`}>
      <header className="replay-toolbar">
        <div className="replay-toolbar-main">
          <p className="eyebrow">
            <Link to="/">Runs</Link>
            {" / "}
            <Link to={`/runs/${runId}`}>{runId}</Link>
            {" / "}
            episode {episode}
          </p>
          <div className="title-row">
            <h1>{manifest?.scenario_name ?? "Episode Replay"}</h1>
            {manifest ? <RunKindBadge isMission={isMissionRun(manifest)} /> : null}
          </div>
        </div>
        {manifest ? (
          <EpisodePicker
            runId={runId}
            episodeCount={manifest.episode_count}
            currentEpisode={episode}
          />
        ) : null}
      </header>

      {error ? <p className="replay-error">{error}</p> : null}

      <section ref={stageRef} className="replay-stage">
        <OrbitReplay
          trajectory={trajectory}
          frameIndex={frameIndex}
          manifest={manifest}
          cameraMode={cameraMode}
          fitKey={fitKey}
          overviewCenter={bounds?.center ?? null}
          overviewRadiusM={bounds?.radius ?? EARTH_RADIUS_M * 2}
          showGrid={showGrid}
          showTriad={showTriad}
        />

        <ViewportHud
          instrumentation={instrumentation}
          manifest={manifest}
          isMission={manifest ? isMissionRun(manifest) : false}
          frameIndex={frameIndex}
          frameCount={frameCount}
        />

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

        <InstrumentationDock
          collapsed={dockCollapsed}
          onToggleCollapsed={() => setDockCollapsed((value) => !value)}
          altitudeKm={altitudeKm}
          speedKmS={speedKmS}
          specificEnergyMJPerKg={specificEnergyMJPerKg}
          timesS={timesS}
          frameIndex={frameIndex}
          frameCount={frameCount}
          currentTimeS={current?.t ?? 0}
          endTimeS={endTimeS}
          playing={playing}
          playbackSpeed={playbackSpeed}
          onTogglePlay={() => setPlaying((value) => !value)}
          onFrameChange={setFrameIndex}
          onPlaybackSpeedChange={setPlaybackSpeed}
          onStep={stepFrame}
          segmentRanges={ranges}
          showSegments={segmentMetadata}
        />
      </section>
    </main>
  );
}
