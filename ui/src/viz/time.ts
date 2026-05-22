/**
 * Parses a created-at string that may be either an ISO-8601 timestamp
 * (e.g. "2025-12-01T12:34:56Z") or a unix epoch in seconds (e.g. "1779411251").
 */
export function parseCreatedAt(value: string | undefined | null): Date | null {
  if (!value) {
    return null;
  }
  const trimmed = value.trim();
  if (trimmed === "") {
    return null;
  }
  // Numeric-only: treat as unix seconds (or milliseconds if very large).
  if (/^-?\d+(?:\.\d+)?$/.test(trimmed)) {
    const numeric = Number.parseFloat(trimmed);
    if (!Number.isFinite(numeric)) {
      return null;
    }
    const ms = numeric > 1e12 ? numeric : numeric * 1000;
    const date = new Date(ms);
    return Number.isNaN(date.getTime()) ? null : date;
  }
  const date = new Date(trimmed);
  return Number.isNaN(date.getTime()) ? null : date;
}

const RELATIVE_UNITS: { ms: number; unit: Intl.RelativeTimeFormatUnit }[] = [
  { ms: 365 * 24 * 60 * 60 * 1000, unit: "year" },
  { ms: 30 * 24 * 60 * 60 * 1000, unit: "month" },
  { ms: 7 * 24 * 60 * 60 * 1000, unit: "week" },
  { ms: 24 * 60 * 60 * 1000, unit: "day" },
  { ms: 60 * 60 * 1000, unit: "hour" },
  { ms: 60 * 1000, unit: "minute" },
  { ms: 1000, unit: "second" },
];

const RTF = new Intl.RelativeTimeFormat(undefined, { numeric: "auto" });

export function formatRelative(date: Date, now: Date = new Date()): string {
  const diffMs = date.getTime() - now.getTime();
  const absMs = Math.abs(diffMs);
  if (absMs < 60_000) {
    return diffMs <= 0 ? "just now" : "in a moment";
  }
  for (const { ms, unit } of RELATIVE_UNITS) {
    if (absMs >= ms) {
      const value = Math.round(diffMs / ms);
      return RTF.format(value, unit);
    }
  }
  return RTF.format(Math.round(diffMs / 1000), "second");
}

const ABSOLUTE_FORMATTER = new Intl.DateTimeFormat(undefined, {
  year: "numeric",
  month: "short",
  day: "2-digit",
  hour: "2-digit",
  minute: "2-digit",
});

export function formatAbsolute(date: Date): string {
  return ABSOLUTE_FORMATTER.format(date);
}
