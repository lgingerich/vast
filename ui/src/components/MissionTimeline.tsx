import type { SegmentRange } from "../viz/trajectory";
import { SEGMENT_COLORS } from "../viz/constants";

interface MissionTimelineProps {
  ranges: SegmentRange[];
  frameIndex: number;
  onJump: (frameIndex: number) => void;
}

export function MissionTimeline({ ranges, frameIndex, onJump }: MissionTimelineProps) {
  if (ranges.length <= 1) {
    return null;
  }

  const totalFrames = ranges[ranges.length - 1]?.endFrame ?? 0;
  const activeIndex = ranges.findIndex(
    (range) => frameIndex >= range.startFrame && frameIndex <= range.endFrame,
  );

  return (
    <div className="mission-timeline">
      <div className="mission-timeline-header">
        <h3>Segments</h3>
        <span
          style={{
            fontFamily: "var(--font-mono)",
            fontSize: "0.625rem",
            color: "var(--text-muted)",
            letterSpacing: "0.06em",
          }}
        >
          {activeIndex >= 0
            ? `Active ${activeIndex + 1} / ${ranges.length}`
            : `${ranges.length} segments`}
        </span>
      </div>
      <div className="segment-track">
        {ranges.map((range, index) => {
          const span = range.endFrame - range.startFrame + 1;
          const widthPercent = (span / Math.max(totalFrames + 1, 1)) * 100;
          const active = index === activeIndex;
          const color = SEGMENT_COLORS[range.index % SEGMENT_COLORS.length];

          return (
            <button
              key={range.index}
              type="button"
              className={`segment-chip${active ? " active" : ""}`}
              style={{
                flexBasis: `${widthPercent}%`,
                borderColor: color,
                color,
                background: active ? `${color}1f` : "transparent",
              }}
              onClick={() => onJump(range.startFrame)}
              aria-pressed={active}
            >
              <span className="segment-chip-label">Segment {range.index + 1}</span>
              <span className="segment-chip-meta">
                f {range.startFrame + 1}-{range.endFrame + 1}
              </span>
            </button>
          );
        })}
      </div>
    </div>
  );
}
