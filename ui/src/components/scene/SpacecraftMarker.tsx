import { Line } from "@react-three/drei";
import { useMemo, useRef } from "react";
import type { Group } from "three";
import type { TrajectoryPoint } from "../../types";
import { eciToScene } from "../../viz/coordinates";
import { useScreenScale } from "../../viz/useScreenScale";

interface SpacecraftMarkerProps {
  point: TrajectoryPoint;
  showTriad?: boolean;
}

const ARROW_BACK = 0.35;
const ARROW_SIDE = 0.18;

/** Cross product on flat tuples. */
function cross(
  a: [number, number, number],
  b: [number, number, number],
): [number, number, number] {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function normalize(v: [number, number, number]): [number, number, number] {
  const len = Math.hypot(v[0], v[1], v[2]);
  if (len <= 0) {
    return [0, 0, 0];
  }
  return [v[0] / len, v[1] / len, v[2] / len];
}

export function SpacecraftMarker({
  point,
  showTriad = false,
}: SpacecraftMarkerProps) {
  const groupRef = useRef<Group>(null);
  const position = useMemo(() => eciToScene(point.pos), [point.pos]);

  useScreenScale(groupRef, 18);

  const velocityArrow = useMemo(() => {
    const speed = Math.hypot(...point.vel);
    if (speed <= 0) {
      return null;
    }
    const scale = 0.00006;
    const tipEci: [number, number, number] = [
      point.pos[0] + point.vel[0] * scale,
      point.pos[1] + point.vel[1] * scale,
      point.pos[2] + point.vel[2] * scale,
    ];
    const tipScene = eciToScene(tipEci);
    const tipLocal: [number, number, number] = [
      tipScene[0] - position[0],
      tipScene[1] - position[1],
      tipScene[2] - position[2],
    ];
    const tipLen = Math.hypot(...tipLocal);
    if (tipLen <= 0) {
      return null;
    }
    const dir = normalize(tipLocal);

    // Build an orthogonal basis for the arrowhead barbs
    const fallback: [number, number, number] =
      Math.abs(dir[1]) < 0.95 ? [0, 1, 0] : [1, 0, 0];
    const side = normalize(cross(dir, fallback));
    const back: [number, number, number] = [
      tipLocal[0] - dir[0] * ARROW_BACK,
      tipLocal[1] - dir[1] * ARROW_BACK,
      tipLocal[2] - dir[2] * ARROW_BACK,
    ];
    const barbA: [number, number, number] = [
      back[0] + side[0] * ARROW_SIDE,
      back[1] + side[1] * ARROW_SIDE,
      back[2] + side[2] * ARROW_SIDE,
    ];
    const barbB: [number, number, number] = [
      back[0] - side[0] * ARROW_SIDE,
      back[1] - side[1] * ARROW_SIDE,
      back[2] - side[2] * ARROW_SIDE,
    ];

    return {
      shaft: [[0, 0, 0], tipLocal] as [
        [number, number, number],
        [number, number, number],
      ],
      head: [barbA, tipLocal, barbB] as [
        [number, number, number],
        [number, number, number],
        [number, number, number],
      ],
    };
  }, [point.pos, point.vel, position]);

  const triad = useMemo(() => {
    if (!showTriad) {
      return null;
    }
    const r = normalize(eciToScene(point.pos));
    const v = normalize(eciToScene(point.vel));
    if (Math.hypot(...r) === 0 || Math.hypot(...v) === 0) {
      return null;
    }
    const h = normalize(cross(r, v));
    const along = normalize(cross(h, r));
    const len = 1.2;
    return {
      radial: [
        [0, 0, 0],
        [r[0] * len, r[1] * len, r[2] * len],
      ] as [[number, number, number], [number, number, number]],
      along: [
        [0, 0, 0],
        [along[0] * len, along[1] * len, along[2] * len],
      ] as [[number, number, number], [number, number, number]],
      cross: [
        [0, 0, 0],
        [h[0] * len, h[1] * len, h[2] * len],
      ] as [[number, number, number], [number, number, number]],
    };
  }, [point.pos, point.vel, showTriad]);

  return (
    <group ref={groupRef} position={position}>
      <mesh>
        <sphereGeometry args={[1, 24, 24]} />
        <meshStandardMaterial
          color="#fb923c"
          emissive="#f97316"
          emissiveIntensity={1.4}
          toneMapped={false}
        />
      </mesh>
      <mesh>
        <sphereGeometry args={[1.85, 16, 16]} />
        <meshBasicMaterial color="#fdba74" transparent opacity={0.22} depthWrite={false} />
      </mesh>
      {velocityArrow ? (
        <>
          <Line points={velocityArrow.shaft} color="#fed7aa" lineWidth={2.5} />
          <Line points={velocityArrow.head} color="#fed7aa" lineWidth={2.5} />
        </>
      ) : null}
      {triad ? (
        <>
          <Line points={triad.radial} color="#ef4444" lineWidth={1.8} />
          <Line points={triad.along} color="#22c55e" lineWidth={1.8} />
          <Line points={triad.cross} color="#3b82f6" lineWidth={1.8} />
        </>
      ) : null}
    </group>
  );
}
