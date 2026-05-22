import type { ReactNode } from "react";
import { Link, useNavigate } from "react-router-dom";

interface EpisodePickerProps {
  runId: string;
  episodeCount: number;
  currentEpisode: number;
}

export function EpisodePicker({
  runId,
  episodeCount,
  currentEpisode,
}: EpisodePickerProps) {
  const navigate = useNavigate();

  if (episodeCount <= 1) {
    return null;
  }

  return (
    <label className="episode-picker">
      Episode
      <select
        value={currentEpisode}
        onChange={(event) => {
          const nextEpisode = Number(event.target.value);
          navigate(`/runs/${runId}/episodes/${nextEpisode}`);
        }}
      >
        {Array.from({ length: episodeCount }, (_, index) => (
          <option key={index} value={index}>
            Episode {index}
          </option>
        ))}
      </select>
    </label>
  );
}

interface RunKindBadgeProps {
  isMission: boolean;
}

export function RunKindBadge({ isMission }: RunKindBadgeProps) {
  return (
    <span className={`run-badge ${isMission ? "mission" : "rollout"}`}>
      {isMission ? "Mission" : "Rollout"}
    </span>
  );
}

interface RunLinkProps {
  runId: string;
  children: ReactNode;
}

export function RunLink({ runId, children }: RunLinkProps) {
  return <Link to={`/runs/${runId}`}>{children}</Link>;
}
