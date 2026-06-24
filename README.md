# VAST — Vectorized Astrodynamics Simulation Toolkit

VAST is a Rust orbital simulation platform built primarily as a
**reinforcement-learning training environment**: run large numbers of
deterministic, loose- to medium-fidelity physics rollouts in parallel, train
control policies, then hand promising trajectories to high-fidelity tools such as
GMAT for detailed mission analysis.

The long-term product is not a GMAT replacement or a visualization demo. It is
**fast, composable RL infrastructure for space** — scenarios, propagators,
actions, observations, rewards, and termination conditions that engineers can
scale to massive batch training and later validate precisely elsewhere.

## Direction

- **RL-first product**: The core API target is `reset` / `step(action)` over
  orbital environments, not one-off propagation scripts.
- **Train at scale, validate elsewhere**: Prefer fixed-step, batch-friendly
  physics that is good enough to learn from; defer mission-analysis fidelity
  (adaptive integrators, high-degree gravity, ephemerides) to external tools.
- **Readable physics core**: Keep equations visible so engineers trust rollouts
  enough to drill down in GMAT or similar when a policy looks promising.
- **Scenario-driven design**: Compose initial conditions, forces, controls,
  rewards, and stopping rules as reusable environment definitions.
- **Performance-aware foundation**: SoA state, allocation-free stepping, and
  batched backends to support many parallel environments.

## Documentation

- **[Architecture](docs/architecture.md)** — layered simulator model, physics vs
  simulation vs recording boundaries, and design targets
- **[Roadmap](ROADMAP.md)** — prioritized unimplemented work

The physics core, simulation orchestration, and an **RL Ops MVP** are
implemented: scenario TOML (rollouts and multi-segment missions), a
single-body fixed-step propagator, duration/step/orbital termination,
impulsive burns, `LeoCircularEnv` (`reset` / bounded `step(action)` with a
low-thrust inertial acceleration command), run artifact recording, the `runner`
CLI/API, and a React replay/compare UI. Batched training, reward functions, and
boundary checks are next — see the architecture doc and roadmap.

## Capabilities

- **Physics core**: typed SI domain values, SoA `CartesianStateBuffer<N>`
  storage, two-body / J2 / exponential-atmosphere drag / composite dynamics,
  Euler, velocity Verlet, leapfrog, and RK4 fixed-step integrators, the `Orbit`
  enum and singular-safe `OrbitalElements`, and `Sim`/`SimBuilder` for
  fixed-duration single-body propagation.
- **Simulation layer**: TOML scenario DTOs, validated runtime plans, a
  `Propagator` with duration/step-count/apoapsis/periapsis termination,
  impulsive burns, a GMAT-style `run_mission` segment runner, and
  `LeoCircularEnv`.
- **Recording**: `RunRecorder` writes per-run manifests, `metrics.jsonl`, and
  per-episode `trajectory.jsonl` under `runs/<run_id>/`.
- **Examples**: `orbit_viz`, `orbit_period`, `inclined_orbit`, `j2_precession`,
  `drag_decay`, `force_model_compare`, `mission_transfer`, `env_rollout`.

## Running

### Examples

```bash
cargo run --example orbit_viz
cargo run --example env_rollout            # RL environment rollout
cargo run --example mission_transfer       # Hohmann mission via run_mission
cargo run --example force_model_compare    # two-body vs J2 vs drag vs J2+drag
```

### RL Ops MVP (rollouts + missions + replay UI)

```bash
# 1. Run headless episodes → writes runs/<run_id>/
cargo run --bin runner -- run --scenario scenarios/leo_circular.toml --episodes 3

# Mission scenarios use the same runner:
cargo run --bin runner -- run --scenario scenarios/hohmann_transfer.toml --episodes 1

# 2. Serve run artifacts to the UI
cargo run --bin runner -- serve --runs runs --port 3001

# 3. Start the web UI (proxies /api to the runner)
cd ui && npm install && npm run dev
```

Open the Vite dev server URL (default `http://localhost:5173`). The UI falls back
to bundled sample data under `ui/public/sample-run/` when the API is offline.
Routes: `/` (runs list), `/runs/:runId` (run detail with metrics), and
`/runs/:runId/episodes/:episodeIndex` (3D replay) with a compare view for
multiple episodes.

### Scenarios

Rollout scenarios (`scenarios/leo_circular.toml`, `leo_j2.toml`, `leo_drag.toml`,
`leo_circular_short.toml`) drive `LeoCircularEnv`; mission scenarios
(`hohmann_transfer.toml`, `bielliptic_leo_to_geo.toml`) drive `run_mission` with
ordered segments, stops, and burns.

## License

[TBD]
