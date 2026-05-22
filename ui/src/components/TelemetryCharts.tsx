interface SparklineProps {
  label: string;
  unit: string;
  values: number[];
  timesS: number[];
  frameIndex: number;
  color: string;
  baseline?: number;
}

function minMax(values: number[]): { min: number; max: number } {
  if (values.length === 0) {
    return { min: 0, max: 1 };
  }
  let min = Infinity;
  let max = -Infinity;
  for (let i = 0; i < values.length; i += 1) {
    const v = values[i];
    if (!Number.isFinite(v)) {
      continue;
    }
    if (v < min) {
      min = v;
    }
    if (v > max) {
      max = v;
    }
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    return { min: 0, max: 1 };
  }
  return { min, max };
}

function formatValue(value: number): string {
  if (!Number.isFinite(value)) {
    return "—";
  }
  const abs = Math.abs(value);
  if (abs >= 1000) {
    return value.toFixed(0);
  }
  if (abs >= 1) {
    return value.toFixed(2);
  }
  return value.toFixed(3);
}

export function Sparkline({
  label,
  unit,
  values,
  timesS,
  frameIndex,
  color,
  baseline,
}: SparklineProps) {
  if (values.length === 0) {
    return null;
  }

  const width = 320;
  const height = 96;
  const padX = 28;
  const padY = 12;

  const { min, max } = minMax(values);
  const span = Math.max(max - min, 1e-6);
  const yFor = (v: number) =>
    height - padY - ((v - min) / span) * (height - padY * 2);
  const xFor = (i: number) =>
    padX + (i / Math.max(values.length - 1, 1)) * (width - padX - padY);

  const points = values
    .map((value, index) => `${xFor(index)},${yFor(value)}`)
    .join(" ");

  const markerIndex = Math.min(Math.max(frameIndex, 0), values.length - 1);
  const markerX = xFor(markerIndex);
  const currentValue = values[markerIndex];
  const currentTime = timesS[markerIndex] ?? timesS[timesS.length - 1];

  const baselineY =
    baseline !== undefined && baseline >= min && baseline <= max
      ? yFor(baseline)
      : null;

  return (
    <div className="sparkline">
      <div className="sparkline-header">
        <span className="sparkline-label">{label}</span>
        <span className="sparkline-current">
          {formatValue(currentValue)}
          <span className="unit">{unit}</span>
        </span>
      </div>
      <svg
        viewBox={`0 0 ${width} ${height}`}
        preserveAspectRatio="none"
        role="img"
        aria-label={`${label} over time`}
      >
        <line
          x1={padX}
          x2={width - padY}
          y1={height - padY}
          y2={height - padY}
          stroke="rgba(148,163,184,0.15)"
          strokeWidth="1"
        />
        {baselineY !== null ? (
          <line
            x1={padX}
            x2={width - padY}
            y1={baselineY}
            y2={baselineY}
            stroke="rgba(148,163,184,0.25)"
            strokeWidth="1"
            strokeDasharray="3 3"
          />
        ) : null}
        <text
          x={padX - 4}
          y={padY + 4}
          fill="rgba(148,163,184,0.65)"
          fontSize="9"
          fontFamily="var(--font-mono)"
          textAnchor="end"
        >
          {formatValue(max)}
        </text>
        <text
          x={padX - 4}
          y={height - padY + 2}
          fill="rgba(148,163,184,0.65)"
          fontSize="9"
          fontFamily="var(--font-mono)"
          textAnchor="end"
        >
          {formatValue(min)}
        </text>
        <polyline
          fill="none"
          stroke={color}
          strokeWidth="1.75"
          strokeLinejoin="round"
          strokeLinecap="round"
          points={points}
        />
        <line
          x1={markerX}
          x2={markerX}
          y1={padY}
          y2={height - padY}
          stroke="#f8fafc"
          strokeWidth="1"
          strokeDasharray="3 3"
          opacity="0.7"
        />
        <circle
          cx={markerX}
          cy={yFor(currentValue)}
          r="3"
          fill={color}
          stroke="#f8fafc"
          strokeWidth="1"
        />
      </svg>
      <div className="sparkline-footer">
        <span>t = {currentTime !== undefined ? currentTime.toFixed(1) : "—"} s</span>
        <span>
          {formatValue(min)} → {formatValue(max)} {unit}
        </span>
      </div>
    </div>
  );
}

interface TelemetryChartsProps {
  altitudeKm: number[];
  speedKmS: number[];
  specificEnergyMJPerKg: number[];
  timesS: number[];
  frameIndex: number;
}

export function TelemetryCharts({
  altitudeKm,
  speedKmS,
  specificEnergyMJPerKg,
  timesS,
  frameIndex,
}: TelemetryChartsProps) {
  return (
    <div className="telemetry-charts">
      <Sparkline
        label="Altitude"
        unit="km"
        values={altitudeKm}
        timesS={timesS}
        frameIndex={frameIndex}
        color="#60a5fa"
      />
      <Sparkline
        label="Speed"
        unit="km/s"
        values={speedKmS}
        timesS={timesS}
        frameIndex={frameIndex}
        color="#22d3ee"
      />
      <Sparkline
        label="Spec energy"
        unit="MJ/kg"
        values={specificEnergyMJPerKg}
        timesS={timesS}
        frameIndex={frameIndex}
        color="#fbbf24"
      />
    </div>
  );
}
