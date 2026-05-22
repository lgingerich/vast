import type { ReactNode } from "react";
import { useMemo } from "react";

interface ChartProps {
  width?: number;
  height?: number;
}

interface BarChartProps extends ChartProps {
  values: number[];
  labels?: (string | number)[];
  color?: string;
  yLabel?: string;
  baseline?: number;
  highlightIndex?: number;
}

function minMax(values: number[]): { min: number; max: number } {
  let min = Infinity;
  let max = -Infinity;
  for (const v of values) {
    if (!Number.isFinite(v)) continue;
    if (v < min) min = v;
    if (v > max) max = v;
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) {
    return { min: 0, max: 1 };
  }
  return { min, max };
}

function niceNumber(value: number): string {
  if (!Number.isFinite(value)) return "—";
  const abs = Math.abs(value);
  if (abs >= 1000) return value.toFixed(0);
  if (abs >= 10) return value.toFixed(1);
  if (abs >= 1) return value.toFixed(2);
  return value.toFixed(3);
}

const AXIS_FONT = "var(--font-mono)";
const AXIS_COLOR = "rgba(148,163,184,0.55)";
const GRID_COLOR = "rgba(148,163,184,0.12)";

function ChartFrame({
  width,
  height,
  padTop = 12,
  padBottom = 22,
  padLeft = 34,
  padRight = 12,
  yMin,
  yMax,
  yTicks = 3,
  children,
}: {
  width: number;
  height: number;
  padTop?: number;
  padBottom?: number;
  padLeft?: number;
  padRight?: number;
  yMin: number;
  yMax: number;
  yTicks?: number;
  children: ReactNode;
}) {
  const innerHeight = height - padTop - padBottom;
  const span = Math.max(yMax - yMin, 1e-6);
  const yFor = (v: number) => padTop + innerHeight - ((v - yMin) / span) * innerHeight;
  const tickValues = Array.from({ length: yTicks }, (_, i) =>
    yMin + (i / Math.max(yTicks - 1, 1)) * span,
  );

  return (
    <svg viewBox={`0 0 ${width} ${height}`} preserveAspectRatio="none">
      {tickValues.map((value) => (
        <g key={value}>
          <line
            x1={padLeft}
            x2={width - padRight}
            y1={yFor(value)}
            y2={yFor(value)}
            stroke={GRID_COLOR}
            strokeWidth="1"
          />
          <text
            x={padLeft - 4}
            y={yFor(value) + 3}
            fill={AXIS_COLOR}
            fontFamily={AXIS_FONT}
            fontSize="9"
            textAnchor="end"
          >
            {niceNumber(value)}
          </text>
        </g>
      ))}
      {children}
    </svg>
  );
}

export function BarChart({
  values,
  labels,
  width = 420,
  height = 180,
  color = "#60a5fa",
  yLabel,
  baseline,
  highlightIndex,
}: BarChartProps) {
  const padTop = 12;
  const padBottom = 22;
  const padLeft = 34;
  const padRight = 12;

  const { min, max } = useMemo(() => {
    const stats = minMax(values);
    let lo = stats.min;
    let hi = stats.max;
    if (baseline !== undefined) {
      lo = Math.min(lo, baseline);
      hi = Math.max(hi, baseline);
    }
    if (lo === hi) {
      const pad = Math.max(Math.abs(lo) * 0.1, 1);
      lo -= pad;
      hi += pad;
    } else if (lo > 0) {
      lo = 0;
    }
    return { min: lo, max: hi };
  }, [values, baseline]);

  const innerWidth = width - padLeft - padRight;
  const innerHeight = height - padTop - padBottom;
  const span = Math.max(max - min, 1e-6);
  const yFor = (v: number) => padTop + innerHeight - ((v - min) / span) * innerHeight;
  const baselineY = baseline !== undefined ? yFor(baseline) : yFor(0);

  const barCount = Math.max(values.length, 1);
  const barWidth = innerWidth / barCount;
  const gap = Math.min(barWidth * 0.18, 4);

  return (
    <ChartFrame
      width={width}
      height={height}
      padTop={padTop}
      padBottom={padBottom}
      padLeft={padLeft}
      padRight={padRight}
      yMin={min}
      yMax={max}
    >
      {yLabel ? (
        <text
          x={4}
          y={padTop + 2}
          fill={AXIS_COLOR}
          fontFamily={AXIS_FONT}
          fontSize="9"
        >
          {yLabel}
        </text>
      ) : null}
      <line
        x1={padLeft}
        x2={width - padRight}
        y1={baselineY}
        y2={baselineY}
        stroke="rgba(148,163,184,0.35)"
        strokeWidth="1"
      />
      {values.map((value, index) => {
        const x = padLeft + index * barWidth + gap / 2;
        const w = Math.max(barWidth - gap, 1);
        const y = value >= (baseline ?? 0) ? yFor(value) : baselineY;
        const h = Math.abs(yFor(value) - baselineY);
        const isHighlight = index === highlightIndex;
        return (
          <rect
            key={index}
            x={x}
            y={y}
            width={w}
            height={Math.max(h, 1)}
            fill={isHighlight ? "#f8fafc" : color}
            opacity={isHighlight ? 1 : 0.85}
            rx={1.5}
          >
            <title>
              {(labels?.[index] ?? `#${index}`).toString()}: {niceNumber(value)}
            </title>
          </rect>
        );
      })}
      {labels && labels.length > 0 ? (
        <>
          <text
            x={padLeft}
            y={height - 6}
            fill={AXIS_COLOR}
            fontFamily={AXIS_FONT}
            fontSize="9"
          >
            {labels[0]}
          </text>
          <text
            x={width - padRight}
            y={height - 6}
            fill={AXIS_COLOR}
            fontFamily={AXIS_FONT}
            fontSize="9"
            textAnchor="end"
          >
            {labels[labels.length - 1]}
          </text>
        </>
      ) : null}
    </ChartFrame>
  );
}

interface LineChartProps extends ChartProps {
  values: number[];
  color?: string;
  yLabel?: string;
}

export function LineChart({
  values,
  width = 420,
  height = 180,
  color = "#22d3ee",
  yLabel,
}: LineChartProps) {
  const padTop = 12;
  const padBottom = 22;
  const padLeft = 34;
  const padRight = 12;

  const { min, max } = useMemo(() => minMax(values), [values]);
  const lo = min === max ? min - 1 : min;
  const hi = min === max ? max + 1 : max;
  const innerWidth = width - padLeft - padRight;
  const innerHeight = height - padTop - padBottom;
  const span = Math.max(hi - lo, 1e-6);
  const xFor = (i: number) =>
    padLeft + (i / Math.max(values.length - 1, 1)) * innerWidth;
  const yFor = (v: number) =>
    padTop + innerHeight - ((v - lo) / span) * innerHeight;

  const path = values
    .map((value, index) => `${xFor(index)},${yFor(value)}`)
    .join(" ");

  return (
    <ChartFrame
      width={width}
      height={height}
      padTop={padTop}
      padBottom={padBottom}
      padLeft={padLeft}
      padRight={padRight}
      yMin={lo}
      yMax={hi}
    >
      {yLabel ? (
        <text
          x={4}
          y={padTop + 2}
          fill={AXIS_COLOR}
          fontFamily={AXIS_FONT}
          fontSize="9"
        >
          {yLabel}
        </text>
      ) : null}
      <polyline
        fill="none"
        stroke={color}
        strokeWidth="1.5"
        strokeLinejoin="round"
        strokeLinecap="round"
        points={path}
      />
      {values.map((value, index) => (
        <circle key={index} cx={xFor(index)} cy={yFor(value)} r={2} fill={color} />
      ))}
    </ChartFrame>
  );
}

interface HistogramChartProps extends ChartProps {
  bins: { start: number; end: number; count: number }[];
  color?: string;
  unit?: string;
}

export function HistogramChart({
  bins,
  width = 420,
  height = 180,
  color = "#a78bfa",
  unit = "",
}: HistogramChartProps) {
  const padTop = 12;
  const padBottom = 26;
  const padLeft = 34;
  const padRight = 12;

  const counts = bins.map((b) => b.count);
  const maxCount = Math.max(...counts, 1);

  const innerWidth = width - padLeft - padRight;
  const innerHeight = height - padTop - padBottom;
  const barWidth = innerWidth / Math.max(bins.length, 1);
  const yFor = (count: number) =>
    padTop + innerHeight - (count / maxCount) * innerHeight;

  return (
    <ChartFrame
      width={width}
      height={height}
      padTop={padTop}
      padBottom={padBottom}
      padLeft={padLeft}
      padRight={padRight}
      yMin={0}
      yMax={maxCount}
    >
      {bins.map((bin, index) => {
        const x = padLeft + index * barWidth + 1;
        const y = yFor(bin.count);
        const h = padTop + innerHeight - y;
        return (
          <rect
            key={index}
            x={x}
            y={y}
            width={Math.max(barWidth - 2, 1)}
            height={Math.max(h, 0)}
            fill={color}
            opacity={0.85}
            rx={1.5}
          >
            <title>
              [{niceNumber(bin.start)}, {niceNumber(bin.end)}] {unit}: {bin.count}
            </title>
          </rect>
        );
      })}
      {bins.length > 0 ? (
        <>
          <text
            x={padLeft}
            y={height - 6}
            fill={AXIS_COLOR}
            fontFamily={AXIS_FONT}
            fontSize="9"
          >
            {niceNumber(bins[0].start)}
          </text>
          <text
            x={width - padRight}
            y={height - 6}
            fill={AXIS_COLOR}
            fontFamily={AXIS_FONT}
            fontSize="9"
            textAnchor="end"
          >
            {niceNumber(bins[bins.length - 1].end)} {unit}
          </text>
        </>
      ) : null}
    </ChartFrame>
  );
}

interface DonutChartProps {
  entries: { label: string; value: number; color: string }[];
  size?: number;
}

export function DonutChart({ entries, size = 140 }: DonutChartProps) {
  const total = entries.reduce((acc, entry) => acc + entry.value, 0);
  const radius = size / 2 - 6;
  const inner = radius * 0.6;
  const center = size / 2;

  if (total === 0) {
    return (
      <svg viewBox={`0 0 ${size} ${size}`} width={size} height={size}>
        <circle
          cx={center}
          cy={center}
          r={radius}
          fill="none"
          stroke={GRID_COLOR}
          strokeWidth={radius - inner}
        />
      </svg>
    );
  }

  let startAngle = -Math.PI / 2;
  return (
    <svg viewBox={`0 0 ${size} ${size}`} width={size} height={size}>
      {entries.map((entry) => {
        const portion = entry.value / total;
        const endAngle = startAngle + portion * Math.PI * 2;
        const x0 = center + radius * Math.cos(startAngle);
        const y0 = center + radius * Math.sin(startAngle);
        const x1 = center + radius * Math.cos(endAngle);
        const y1 = center + radius * Math.sin(endAngle);
        const x0i = center + inner * Math.cos(startAngle);
        const y0i = center + inner * Math.sin(startAngle);
        const x1i = center + inner * Math.cos(endAngle);
        const y1i = center + inner * Math.sin(endAngle);
        const largeArc = portion > 0.5 ? 1 : 0;
        const d = [
          `M ${x0} ${y0}`,
          `A ${radius} ${radius} 0 ${largeArc} 1 ${x1} ${y1}`,
          `L ${x1i} ${y1i}`,
          `A ${inner} ${inner} 0 ${largeArc} 0 ${x0i} ${y0i}`,
          "Z",
        ].join(" ");
        const segment = (
          <path key={entry.label} d={d} fill={entry.color}>
            <title>
              {entry.label}: {entry.value}
            </title>
          </path>
        );
        startAngle = endAngle;
        return segment;
      })}
      <text
        x={center}
        y={center - 3}
        textAnchor="middle"
        fill="#cbd5e1"
        fontFamily={AXIS_FONT}
        fontSize="11"
      >
        {total}
      </text>
      <text
        x={center}
        y={center + 10}
        textAnchor="middle"
        fill={AXIS_COLOR}
        fontFamily={AXIS_FONT}
        fontSize="8"
        letterSpacing="0.1em"
      >
        TOTAL
      </text>
    </svg>
  );
}
