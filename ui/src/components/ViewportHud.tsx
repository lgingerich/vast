import type { FrameInstrumentation } from "../viz/instrumentation";
import type { RunManifest } from "../types";

interface ViewportHudProps {
  instrumentation: FrameInstrumentation | null;
  manifest: RunManifest | null;
  isMission: boolean;
  frameIndex: number;
  frameCount: number;
}

interface InstrumentRow {
  label: string;
  value: string;
  unit: string;
  tone?: "amber" | "cyan";
}

function formatTimeS(timeS: number): string {
  if (!Number.isFinite(timeS) || timeS < 0) {
    return "0.0";
  }
  if (timeS >= 3600) {
    const hours = Math.floor(timeS / 3600);
    const minutes = Math.floor((timeS % 3600) / 60);
    const seconds = (timeS % 60).toFixed(0).padStart(2, "0");
    return `${hours}:${String(minutes).padStart(2, "0")}:${seconds}`;
  }
  if (timeS >= 60) {
    const minutes = Math.floor(timeS / 60);
    const seconds = (timeS % 60).toFixed(1).padStart(4, "0");
    return `${minutes}:${seconds}`;
  }
  return timeS.toFixed(1);
}

function InstrumentBlock({
  header,
  trailing,
  rows,
  single,
}: {
  header: string;
  trailing?: string;
  rows: InstrumentRow[];
  single?: boolean;
}) {
  return (
    <div className="instrument">
      <div className="instrument-header">
        <span>{header}</span>
        {trailing ? <span className="instrument-header-trailing">{trailing}</span> : null}
      </div>
      <div className={`instrument-grid${single ? " single" : ""}`}>
        {rows.map((row) => (
          <div className="instrument-row" key={row.label}>
            <span className="instrument-label">{row.label}</span>
            <span className={`instrument-value${row.tone === "cyan" ? " cyan" : ""}`}>
              {row.value}
              <span className="instrument-unit">{row.unit}</span>
            </span>
          </div>
        ))}
      </div>
    </div>
  );
}

export function ViewportHud({
  instrumentation,
  manifest,
  isMission,
  frameIndex,
  frameCount,
}: ViewportHudProps) {
  if (!instrumentation) {
    return null;
  }

  const stateRows: InstrumentRow[] = [
    {
      label: "Radius",
      value: instrumentation.radiusKm.toFixed(1),
      unit: "km",
    },
    {
      label: "Altitude",
      value: instrumentation.altitudeKm.toFixed(2),
      unit: "km",
    },
    {
      label: "Speed",
      value: instrumentation.speedKmS.toFixed(3),
      unit: "km/s",
      tone: "cyan",
    },
  ];

  const dynamicsRows: InstrumentRow[] = [
    {
      label: "Radial v",
      value: instrumentation.radialSpeedKmS.toFixed(3),
      unit: "km/s",
      tone: "cyan",
    },
    {
      label: "Flight path",
      value: instrumentation.flightPathAngleDeg.toFixed(2),
      unit: "deg",
    },
    {
      label: "Spec energy",
      value: instrumentation.specificEnergyMJPerKg.toFixed(2),
      unit: "MJ/kg",
    },
  ];

  return (
    <div className="viewport-hud">
      <div className="hud-stripe">
        <span className="hud-stripe-scenario">
          {manifest?.scenario_name ?? "Replay"}
        </span>
        <span className="hud-stripe-dot" />
        <span>{isMission ? "Mission" : "Rollout"}</span>
        <span className="hud-stripe-dot" />
        <span>
          {frameCount > 0 ? `${frameIndex + 1} / ${frameCount}` : "0 / 0"}
        </span>
      </div>

      <div className="hud-instrument-stack">
        <InstrumentBlock
          header="Time"
          trailing="MET"
          single
          rows={[
            {
              label: "Mission elapsed",
              value: formatTimeS(instrumentation.timeS),
              unit: instrumentation.timeS >= 60 ? "" : "s",
              tone: "cyan",
            },
          ]}
        />
        <InstrumentBlock header="State" rows={stateRows} />
        <InstrumentBlock header="Dynamics" rows={dynamicsRows} />
        {instrumentation.segmentIndex !== undefined ? (
          <div className="instrument">
            <div className="instrument-segment">
              Segment {instrumentation.segmentIndex + 1}
            </div>
          </div>
        ) : null}
      </div>
    </div>
  );
}
