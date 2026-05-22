import { useEffect, useRef, useState } from "react";
import {
  Compass,
  Crosshair,
  Focus,
  Globe2,
  Grid3x3,
  Keyboard,
  Maximize2,
  Minimize2,
  Move,
  Orbit,
  Target,
} from "lucide-react";
import type { CameraMode } from "./scene/SceneCameraRig";

interface CameraToolbarProps {
  mode: CameraMode;
  onModeChange: (mode: CameraMode) => void;
  onFit: () => void;
  onToggleFullscreen: () => void;
  isFullscreen: boolean;
  showGrid: boolean;
  onToggleGrid: () => void;
  showTriad: boolean;
  onToggleTriad: () => void;
}

interface ModeEntry {
  id: CameraMode;
  label: string;
  Icon: typeof Globe2;
}

const MODES: ModeEntry[] = [
  { id: "overview", label: "Overview", Icon: Orbit },
  { id: "earth", label: "Earth", Icon: Globe2 },
  { id: "spacecraft", label: "Chase", Icon: Crosshair },
  { id: "free", label: "Free", Icon: Move },
];

const SHORTCUTS: { keys: string[]; label: string }[] = [
  { keys: ["Space"], label: "Play / pause" },
  { keys: ["←", "→"], label: "Step frame" },
  { keys: ["H"], label: "Fit camera to trajectory" },
  { keys: ["F"], label: "Toggle full screen" },
  { keys: ["?"], label: "Toggle this help" },
];

export function CameraToolbar({
  mode,
  onModeChange,
  onFit,
  onToggleFullscreen,
  isFullscreen,
  showGrid,
  onToggleGrid,
  showTriad,
  onToggleTriad,
}: CameraToolbarProps) {
  const [showHelp, setShowHelp] = useState(false);
  const helpRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (!showHelp) {
      return;
    }
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setShowHelp(false);
      }
    };
    const onClickOutside = (event: MouseEvent) => {
      if (!helpRef.current?.contains(event.target as Node)) {
        setShowHelp(false);
      }
    };
    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("mousedown", onClickOutside);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("mousedown", onClickOutside);
    };
  }, [showHelp]);

  return (
    <div className="camera-toolbar">
      <div className="toolbar-group">
        {MODES.map((entry) => {
          const { Icon } = entry;
          return (
            <button
              key={entry.id}
              type="button"
              className={`icon-button${mode === entry.id ? " active" : ""}`}
              onClick={() => onModeChange(entry.id)}
              title={entry.label}
              aria-label={entry.label}
            >
              <Icon size={16} strokeWidth={1.75} aria-hidden />
            </button>
          );
        })}
      </div>

      <div className="toolbar-group">
        <button
          type="button"
          className="icon-button"
          onClick={onFit}
          title="Fit camera to trajectory (H)"
          aria-label="Fit camera to trajectory"
        >
          <Focus size={16} strokeWidth={1.75} aria-hidden />
        </button>
        <button
          type="button"
          className={`icon-button${showGrid ? " active" : ""}`}
          onClick={onToggleGrid}
          title="Toggle ECI axes and equator"
          aria-label="Toggle grid"
        >
          <Grid3x3 size={16} strokeWidth={1.75} aria-hidden />
        </button>
        <button
          type="button"
          className={`icon-button${showTriad ? " active" : ""}`}
          onClick={onToggleTriad}
          title="Toggle LVLH triad (radial / along-track / cross-track)"
          aria-label="Toggle LVLH triad"
        >
          <Target size={16} strokeWidth={1.75} aria-hidden />
        </button>
        <span className="toolbar-divider" />
        <button
          type="button"
          className="icon-button"
          onClick={onToggleFullscreen}
          title={isFullscreen ? "Exit full screen" : "Enter full screen"}
          aria-label={isFullscreen ? "Exit full screen" : "Enter full screen"}
        >
          {isFullscreen ? (
            <Minimize2 size={16} strokeWidth={1.75} aria-hidden />
          ) : (
            <Maximize2 size={16} strokeWidth={1.75} aria-hidden />
          )}
        </button>
        <button
          type="button"
          className={`icon-button${showHelp ? " active" : ""}`}
          onClick={() => setShowHelp((value) => !value)}
          title="Keyboard shortcuts"
          aria-label="Keyboard shortcuts"
        >
          <Keyboard size={16} strokeWidth={1.75} aria-hidden />
        </button>
      </div>

      {showHelp ? (
        <div ref={helpRef} className="shortcut-popover" role="dialog" aria-label="Keyboard shortcuts">
          <span className="shortcut-popover-title">Keyboard</span>
          {SHORTCUTS.map((entry) => (
            <div key={entry.label} style={{ display: "contents" }}>
              <dt>
                {entry.keys.map((key) => (
                  <span key={key} className="kbd">
                    {key}
                  </span>
                ))}
              </dt>
              <dd>{entry.label}</dd>
            </div>
          ))}
          <span className="shortcut-popover-title" style={{ marginTop: "0.4rem" }}>
            Mouse
          </span>
          <dt>
            <Compass size={14} aria-hidden />
          </dt>
          <dd>Drag to rotate, wheel to zoom, right-drag to pan (free mode)</dd>
        </div>
      ) : null}
    </div>
  );
}
