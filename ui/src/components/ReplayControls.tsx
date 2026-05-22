import { useMemo, useState } from "react";
import { Pause, Play, SkipBack, SkipForward } from "lucide-react";
import { PLAYBACK_SPEEDS, SEGMENT_COLORS } from "../viz/constants";
import type { SegmentRange } from "../viz/trajectory";

interface ReplayControlsProps {
  frameIndex: number;
  frameCount: number;
  currentTimeS: number;
  endTimeS: number;
  playing: boolean;
  playbackSpeed: number;
  segmentRanges: SegmentRange[];
  onTogglePlay: () => void;
  onFrameChange: (frameIndex: number) => void;
  onPlaybackSpeedChange: (speed: number) => void;
  onStep: (delta: number) => void;
}

function formatSeconds(value: number): string {
  if (!Number.isFinite(value)) {
    return "0.0s";
  }
  if (value >= 60) {
    const minutes = Math.floor(value / 60);
    const seconds = (value % 60).toFixed(1).padStart(4, "0");
    return `${minutes}:${seconds}`;
  }
  return `${value.toFixed(1)}s`;
}

export function ReplayControls({
  frameIndex,
  frameCount,
  currentTimeS,
  endTimeS,
  playing,
  playbackSpeed,
  segmentRanges,
  onTogglePlay,
  onFrameChange,
  onPlaybackSpeedChange,
  onStep,
}: ReplayControlsProps) {
  const [hoverInfo, setHoverInfo] = useState<{ x: number; time: number; frame: number } | null>(
    null,
  );

  const maxFrame = Math.max(frameCount - 1, 0);
  const fillPercent = maxFrame > 0 ? (frameIndex / maxFrame) * 100 : 0;

  const ticks = useMemo(() => {
    if (segmentRanges.length <= 1 || maxFrame <= 0) {
      return [];
    }
    return segmentRanges.slice(0, -1).map((range) => ({
      key: range.index,
      percent: ((range.endFrame + 1) / maxFrame) * 100,
      color: SEGMENT_COLORS[range.index % SEGMENT_COLORS.length],
    }));
  }, [segmentRanges, maxFrame]);

  const handleHover = (event: React.MouseEvent<HTMLDivElement>) => {
    if (maxFrame <= 0) {
      return;
    }
    const rail = event.currentTarget.getBoundingClientRect();
    const ratio = Math.min(Math.max((event.clientX - rail.left) / rail.width, 0), 1);
    const frame = Math.round(ratio * maxFrame);
    const time = endTimeS * ratio;
    setHoverInfo({ x: ratio * 100, time, frame });
  };

  return (
    <div className="replay-controls">
      <div className="transport" role="group" aria-label="Playback transport">
        <button
          type="button"
          className="icon-button"
          onClick={() => onStep(-1)}
          disabled={frameIndex <= 0}
          title="Previous frame (Left arrow)"
          aria-label="Previous frame"
        >
          <SkipBack size={16} strokeWidth={1.75} aria-hidden />
        </button>
        <button
          type="button"
          className="icon-button primary"
          onClick={onTogglePlay}
          title={playing ? "Pause (Space)" : "Play (Space)"}
          aria-label={playing ? "Pause" : "Play"}
        >
          {playing ? (
            <Pause size={18} strokeWidth={2} aria-hidden />
          ) : (
            <Play size={18} strokeWidth={2} aria-hidden style={{ marginLeft: 2 }} />
          )}
        </button>
        <button
          type="button"
          className="icon-button"
          onClick={() => onStep(1)}
          disabled={frameIndex >= maxFrame}
          title="Next frame (Right arrow)"
          aria-label="Next frame"
        >
          <SkipForward size={16} strokeWidth={1.75} aria-hidden />
        </button>
      </div>

      <div className="transport-hints">
        <span className="kbd">Space</span>
        <span>play</span>
        <span className="kbd">←</span>
        <span className="kbd">→</span>
        <span>step</span>
      </div>

      <div className="scrubber">
        <div
          className="scrubber-rail"
          onMouseMove={handleHover}
          onMouseLeave={() => setHoverInfo(null)}
        >
          <div className="scrubber-track">
            <div className="scrubber-fill" style={{ width: `${fillPercent}%` }} />
          </div>
          <div className="scrubber-ticks">
            {ticks.map((tick) => (
              <span
                key={tick.key}
                className="scrubber-tick"
                style={{ left: `${tick.percent}%`, background: tick.color }}
              />
            ))}
          </div>
          <input
            type="range"
            min={0}
            max={maxFrame}
            value={frameIndex}
            onChange={(event) => onFrameChange(Number(event.target.value))}
            aria-label="Trajectory scrubber"
          />
          {hoverInfo ? (
            <div
              style={{
                position: "absolute",
                left: `${hoverInfo.x}%`,
                bottom: "100%",
                transform: "translate(-50%, -6px)",
                padding: "0.2rem 0.45rem",
                background: "var(--panel-bg-strong)",
                border: "1px solid var(--border)",
                borderRadius: "var(--radius-sm)",
                color: "var(--text-1)",
                fontFamily: "var(--font-mono)",
                fontSize: "0.625rem",
                pointerEvents: "none",
                whiteSpace: "nowrap",
                zIndex: 5,
              }}
            >
              t = {formatSeconds(hoverInfo.time)} · f{hoverInfo.frame + 1}
            </div>
          ) : null}
        </div>
        <div className="scrubber-meta">
          <span>{formatSeconds(currentTimeS)} / {formatSeconds(endTimeS)}</span>
          <span>
            frame {frameIndex + 1} / {frameCount}
          </span>
        </div>
      </div>

      <div className="speed-row">
        <span>Playback</span>
        <div className="segmented" role="group" aria-label="Playback speed">
          {PLAYBACK_SPEEDS.map((speed) => (
            <button
              key={speed}
              type="button"
              className={speed === playbackSpeed ? "active" : ""}
              onClick={() => onPlaybackSpeedChange(speed)}
              aria-pressed={speed === playbackSpeed}
            >
              {speed}x
            </button>
          ))}
        </div>
      </div>
    </div>
  );
}
