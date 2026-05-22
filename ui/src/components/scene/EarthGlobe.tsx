import { useMemo, useRef } from "react";
import { BackSide, type Mesh, type ShaderMaterial } from "three";
import { useFrame } from "@react-three/fiber";
import { EARTH_RADIUS_M, SCENE_SCALE } from "../../viz/constants";

const EARTH_VERT = /* glsl */ `
  varying vec3 vNormal;
  varying vec3 vWorldPos;
  varying vec2 vUv;
  void main() {
    vNormal = normalize(normalMatrix * normal);
    vec4 worldPos = modelMatrix * vec4(position, 1.0);
    vWorldPos = worldPos.xyz;
    vUv = uv;
    gl_Position = projectionMatrix * viewMatrix * worldPos;
  }
`;

const EARTH_FRAG = /* glsl */ `
  precision highp float;
  varying vec3 vNormal;
  varying vec3 vWorldPos;
  varying vec2 vUv;
  uniform vec3 uSunDir;
  uniform vec3 uOceanDeep;
  uniform vec3 uOceanShallow;
  uniform vec3 uLand;
  uniform vec3 uIce;
  uniform vec3 uCityLight;
  uniform float uTime;

  // Smooth value-noise-ish hash for stylized continents
  float hash(vec3 p) {
    return fract(sin(dot(p, vec3(127.1, 311.7, 74.7))) * 43758.5453);
  }

  float noise(vec3 p) {
    vec3 i = floor(p);
    vec3 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);
    float n000 = hash(i + vec3(0.0, 0.0, 0.0));
    float n100 = hash(i + vec3(1.0, 0.0, 0.0));
    float n010 = hash(i + vec3(0.0, 1.0, 0.0));
    float n110 = hash(i + vec3(1.0, 1.0, 0.0));
    float n001 = hash(i + vec3(0.0, 0.0, 1.0));
    float n101 = hash(i + vec3(1.0, 0.0, 1.0));
    float n011 = hash(i + vec3(0.0, 1.0, 1.0));
    float n111 = hash(i + vec3(1.0, 1.0, 1.0));
    float nx00 = mix(n000, n100, f.x);
    float nx10 = mix(n010, n110, f.x);
    float nx01 = mix(n001, n101, f.x);
    float nx11 = mix(n011, n111, f.x);
    float nxy0 = mix(nx00, nx10, f.y);
    float nxy1 = mix(nx01, nx11, f.y);
    return mix(nxy0, nxy1, f.z);
  }

  float fbm(vec3 p) {
    float v = 0.0;
    float amp = 0.5;
    for (int i = 0; i < 5; i++) {
      v += amp * noise(p);
      p *= 2.0;
      amp *= 0.5;
    }
    return v;
  }

  void main() {
    vec3 n = normalize(vNormal);
    vec3 sun = normalize(uSunDir);
    float ndl = dot(n, sun);
    float day = smoothstep(-0.18, 0.25, ndl);

    // Stylized continents via fbm against a normalized world direction
    vec3 dir = normalize(vWorldPos);
    float continents = fbm(dir * 2.8);
    float coastline = smoothstep(0.48, 0.55, continents);
    float landMask = smoothstep(0.52, 0.62, continents);

    // Latitude-based ice caps
    float lat = abs(dir.y);
    float ice = smoothstep(0.78, 0.92, lat) * landMask;

    vec3 ocean = mix(uOceanDeep, uOceanShallow, smoothstep(0.45, 0.55, continents));
    vec3 surface = mix(ocean, uLand, coastline);
    surface = mix(surface, uIce, ice);

    // Subtle highlights on the bright side
    float specular = pow(max(ndl, 0.0), 8.0) * (1.0 - landMask) * 0.35;

    // City lights on the night side — sparse, follow continents
    float cityNoise = fbm(dir * 18.0);
    float cityMask = smoothstep(0.62, 0.74, cityNoise) * landMask;
    float night = 1.0 - day;
    vec3 cityLights = uCityLight * cityMask * night * 0.85;

    // Darken night side ambient
    vec3 dayColor = surface * (0.4 + 0.7 * day) + vec3(specular);
    vec3 color = dayColor + cityLights;

    // Subtle rim/atmosphere on the day side
    vec3 viewDir = normalize(cameraPosition - vWorldPos);
    float rim = pow(1.0 - max(dot(n, viewDir), 0.0), 3.0);
    color += vec3(0.18, 0.32, 0.55) * rim * day * 0.8;

    gl_FragColor = vec4(color, 1.0);
  }
`;

const ATMOSPHERE_VERT = /* glsl */ `
  varying vec3 vNormal;
  varying vec3 vWorldPos;
  void main() {
    vNormal = normalize(normalMatrix * normal);
    vec4 worldPos = modelMatrix * vec4(position, 1.0);
    vWorldPos = worldPos.xyz;
    gl_Position = projectionMatrix * viewMatrix * worldPos;
  }
`;

const ATMOSPHERE_FRAG = /* glsl */ `
  precision highp float;
  varying vec3 vNormal;
  varying vec3 vWorldPos;
  uniform vec3 uSunDir;
  uniform vec3 uAtmoColor;

  void main() {
    vec3 viewDir = normalize(cameraPosition - vWorldPos);
    vec3 n = normalize(vNormal);
    float rim = pow(1.0 - max(dot(viewDir, n), 0.0), 2.5);
    float sunLit = clamp(dot(n, normalize(uSunDir)) + 0.25, 0.0, 1.0);
    vec3 color = uAtmoColor * rim * (0.45 + 0.85 * sunLit);
    float alpha = rim * (0.55 + 0.45 * sunLit);
    gl_FragColor = vec4(color, alpha);
  }
`;

interface EarthGlobeProps {
  sunDirection?: [number, number, number];
}

export function EarthGlobe({ sunDirection = [12, 8, 6] }: EarthGlobeProps) {
  const radius = EARTH_RADIUS_M * SCENE_SCALE;
  const planetRef = useRef<Mesh>(null);
  const earthMatRef = useRef<ShaderMaterial>(null);
  const atmoMatRef = useRef<ShaderMaterial>(null);

  const earthUniforms = useMemo(
    () => ({
      uSunDir: { value: sunDirection },
      uOceanDeep: { value: [0.04, 0.09, 0.22] },
      uOceanShallow: { value: [0.09, 0.22, 0.42] },
      uLand: { value: [0.18, 0.32, 0.18] },
      uIce: { value: [0.85, 0.92, 1.0] },
      uCityLight: { value: [1.0, 0.78, 0.36] },
      uTime: { value: 0 },
    }),
    [sunDirection],
  );

  const atmoUniforms = useMemo(
    () => ({
      uSunDir: { value: sunDirection },
      uAtmoColor: { value: [0.32, 0.58, 1.0] },
    }),
    [sunDirection],
  );

  useFrame((_, dt) => {
    if (planetRef.current) {
      planetRef.current.rotation.y += dt * 0.012;
    }
    if (earthMatRef.current) {
      earthMatRef.current.uniforms.uTime.value += dt;
    }
  });

  return (
    <group>
      <mesh ref={planetRef}>
        <sphereGeometry args={[radius, 96, 96]} />
        <shaderMaterial
          ref={earthMatRef}
          vertexShader={EARTH_VERT}
          fragmentShader={EARTH_FRAG}
          uniforms={earthUniforms}
        />
      </mesh>
      <mesh scale={1.028}>
        <sphereGeometry args={[radius, 64, 64]} />
        <shaderMaterial
          ref={atmoMatRef}
          vertexShader={ATMOSPHERE_VERT}
          fragmentShader={ATMOSPHERE_FRAG}
          uniforms={atmoUniforms}
          transparent
          depthWrite={false}
          side={BackSide}
        />
      </mesh>
    </group>
  );
}
