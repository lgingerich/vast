import { Line } from "@react-three/drei";
import { useMemo } from "react";
import { eciToScene } from "../../viz/coordinates";

interface ReferenceOrbitRingProps {
  radiusM: number;
  color: string;
  /** Inclination in degrees (rotation about the ECI X axis). */
  inclinationDeg?: number;
  /** RAAN in degrees (rotation about the ECI Z axis). */
  raanDeg?: number;
  /** Argument of periapsis in degrees (rotation in the orbital plane). */
  argPeriapsisDeg?: number;
}

const DEG = Math.PI / 180;

/** Apply 3-1-3 Euler rotation (RAAN, inclination, argPeriapsis) to an ECI point. */
function rotateToOrbitPlane(
  x: number,
  y: number,
  z: number,
  raan: number,
  inc: number,
  argP: number,
): [number, number, number] {
  // Rotate by argPeriapsis about local Z (in plane)
  const cosA = Math.cos(argP);
  const sinA = Math.sin(argP);
  let px = x * cosA - y * sinA;
  let py = x * sinA + y * cosA;
  let pz = z;

  // Rotate by inclination about X
  const cosI = Math.cos(inc);
  const sinI = Math.sin(inc);
  const ix = px;
  const iy = py * cosI - pz * sinI;
  const iz = py * sinI + pz * cosI;
  px = ix;
  py = iy;
  pz = iz;

  // Rotate by RAAN about Z
  const cosR = Math.cos(raan);
  const sinR = Math.sin(raan);
  const rx = px * cosR - py * sinR;
  const ry = px * sinR + py * cosR;
  const rz = pz;

  return [rx, ry, rz];
}

export function ReferenceOrbitRing({
  radiusM,
  color,
  inclinationDeg = 0,
  raanDeg = 0,
  argPeriapsisDeg = 0,
}: ReferenceOrbitRingProps) {
  const points = useMemo(() => {
    const segments = 128;
    const inc = inclinationDeg * DEG;
    const raan = raanDeg * DEG;
    const argP = argPeriapsisDeg * DEG;
    const result: [number, number, number][] = [];
    for (let index = 0; index <= segments; index += 1) {
      const angle = (index / segments) * Math.PI * 2;
      const x = radiusM * Math.cos(angle);
      const y = radiusM * Math.sin(angle);
      const rotated = rotateToOrbitPlane(x, y, 0, raan, inc, argP);
      result.push(eciToScene(rotated));
    }
    return result;
  }, [radiusM, inclinationDeg, raanDeg, argPeriapsisDeg]);

  return <Line points={points} color={color} lineWidth={1} transparent opacity={0.4} />;
}
