import { Line } from "@react-three/drei";
import { EARTH_RADIUS_M, SCENE_SCALE } from "../../viz/constants";

const AXIS_LENGTH_M = EARTH_RADIUS_M * 1.35;

const AXES = [
  { color: "#ef4444", end: [AXIS_LENGTH_M, 0, 0] as [number, number, number], label: "X" },
  { color: "#22c55e", end: [0, AXIS_LENGTH_M, 0] as [number, number, number], label: "Z" },
  { color: "#3b82f6", end: [0, 0, AXIS_LENGTH_M] as [number, number, number], label: "Y" },
] as const;

function toScene(pos: [number, number, number]): [number, number, number] {
  return [pos[0] * SCENE_SCALE, pos[2] * SCENE_SCALE, pos[1] * SCENE_SCALE];
}

export function EciAxes() {
  return (
    <group>
      {AXES.map((axis) => (
        <Line
          key={axis.label}
          points={[toScene([0, 0, 0]), toScene(axis.end)]}
          color={axis.color}
          lineWidth={1.5}
        />
      ))}
    </group>
  );
}

export function EquatorialGrid() {
  const radiusM = EARTH_RADIUS_M * 1.05;
  const segments = 72;
  const points: [number, number, number][] = [];
  for (let index = 0; index <= segments; index += 1) {
    const angle = (index / segments) * Math.PI * 2;
    points.push(toScene([radiusM * Math.cos(angle), radiusM * Math.sin(angle), 0]));
  }

  return (
    <Line
      points={points}
      color="#334155"
      lineWidth={1}
      transparent
      opacity={0.35}
    />
  );
}
