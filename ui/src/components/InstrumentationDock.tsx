import { ChevronDown, ChevronUp, Gauge } from "lucide-react";
import { MissionTimeline } from "./MissionTimeline";
import { ReplayControls } from "./ReplayControls";
import { TelemetryCharts } from "./TelemetryCharts";
import type { SegmentRange } from "../viz/trajectory";

interface InstrumentationDockProps {
  collapsed: boolean;
  onToggleCollapsed: () => void;
  altitudeKm: number[];
  speedKmS: number[];
  specificEnergyMJPerKg: number[];
  timesS: number[];
  frameIndex: number;
  frameCount: number;
  currentTimeS: number;
  endTimeS: number;
  playing: boolean;
  playbackSpeed: number;
  onTogglePlay: () => void;
  onFrameChange: (frameIndex: number) => void;
  onPlaybackSpeedChange: (speed: number) => void;
  onStep: (delta: number) => void;
  segmentRanges: SegmentRange[];
  showSegments: boolean;
}

export function InstrumentationDock({
  collapsed,
  onToggleCollapsed,
  altitudeKm,
  speedKmS,
  specificEnergyMJPerKg,
  timesS,
  frameIndex,
  frameCount,
  currentTimeS,
  endTimeS,
  playing,
  playbackSpeed,
  onTogglePlay,
  onFrameChange,
  onPlaybackSpeedChange,
  onStep,
  segmentRanges,
  showSegments,
}: InstrumentationDockProps) {
  if (collapsed) {
    return (
      <div className="instrumentation-dock collapsed">
        <button
          type="button"
          className="dock-toggle"
          onClick={onToggleCollapsed}
          aria-expanded={false}
        >
          <Gauge size={14} strokeWidth={1.75} aria-hidden />
          <span>Show instrumentation</span>
          <ChevronUp size={14} strokeWidth={1.75} aria-hidden />
        </button>
      </div>
    );
  }

  return (
    <div className="instrumentation-dock">
      <button
        type="button"
        className="dock-toggle"
        onClick={onToggleCollapsed}
        aria-expanded
      >
        <ChevronDown size={14} strokeWidth={1.75} aria-hidden />
        <span>Hide</span>
      </button>

      <section className="dock-panel" aria-label="Telemetry">
        <div className="dock-panel-header">
          <span>Telemetry</span>
          <span>{frameCount > 0 ? `${frameCount} samples` : "no data"}</span>
        </div>
        <TelemetryCharts
          altitudeKm={altitudeKm}
          speedKmS={speedKmS}
          specificEnergyMJPerKg={specificEnergyMJPerKg}
          timesS={timesS}
          frameIndex={frameIndex}
        />
      </section>

      <section className="dock-panel" aria-label="Replay controls">
        <div className="dock-panel-header">
          <span>Replay</span>
          <span>
            {(playbackSpeed === 1 ? "1.00" : playbackSpeed.toFixed(2))} x
          </span>
        </div>
        {showSegments ? (
          <MissionTimeline
            ranges={segmentRanges}
            frameIndex={frameIndex}
            onJump={onFrameChange}
          />
        ) : null}
        <ReplayControls
          frameIndex={frameIndex}
          frameCount={frameCount}
          currentTimeS={currentTimeS}
          endTimeS={endTimeS}
          playing={playing}
          playbackSpeed={playbackSpeed}
          segmentRanges={segmentRanges}
          onTogglePlay={onTogglePlay}
          onFrameChange={onFrameChange}
          onPlaybackSpeedChange={onPlaybackSpeedChange}
          onStep={onStep}
        />
      </section>
    </div>
  );
}
