# Project Memory

## 2026-06-11 — Physics hot path: no per-step finiteness checks

**Decision:** The integrator/dynamics stepping loop operates on raw SoA `f64`
slices and `Vector3<f64>`. `Dynamics::acceleration_at` takes position/velocity
vectors and returns acceleration as `Vector3<f64>`. Finiteness and domain
constraints are enforced only at scenario/boundary constructors (`Position`,
`Velocity`, `TimeStep`, `write_from_state`, etc.). `Epoch` propagation inside
the step loop uses `const fn` arithmetic on already-validated `Epoch` and
`TimeStep` values (`advance_by`, `TimeStep::scaled_by` for RK4 stages) — no
`is_finite` per step.

**Rejected:** `_unchecked` component constructors as a hot-path workaround;
re-validating finiteness when reading/writing the SoA buffer each step.

## 2025-05-21 — Simulation layer architecture

**Decision:** Document the simulator as layered architecture in
`docs/architecture.md`. Physics core (`Dynamics`, `Integrator`, SoA state) stays
separate from simulation orchestration (scenario, propagator, termination,
maneuvers, events, boundaries, environment).

**Influences:** GMAT (init + mission sequence, propagate + stop conditions,
maneuvers not in force models); Basilisk (stop time, terminal events) — but
fixed-step, step-grid event snapping for batch/GPU determinism.

**Rejected for now:** Adaptive integrators on the production path; root finding
for exact event times; mission-script DSL; putting control inputs on
`Dynamics::acceleration_at`.

## 2025-05-21 — RL-first product goal

**Decision:** Primary product is an RL training environment for space — massive
parallel loose-fidelity rollouts, with GMAT-class tools for downstream validation.

## 2025-05-21 — RL Ops MVP contracts

**Decision:** MVP uses the following shapes (implemented in library + runner + UI):

| Area | MVP choice |
|------|------------|
| Scenario | TOML → `ScenarioConfig` (LEO circular, two-body, duration stop) |
| Termination | Closed enum: `Duration`, `StepCount` |
| Environment | `Environment` trait: `reset(seed)`, `step(action)` |
| Action | `[f64; 3]` inertial thrust accel — accepted, ignored until low thrust |
| Observation | `[f64; 8]`: pos(3), vel(3), altitude, speed |
| Reward / done | Reward `0.0`; one rollout per `step`; `done=true` when duration met |
| Artifacts | `runs/<run_id>/manifest.json`, `metrics.jsonl`, `episodes/NNNN/trajectory.jsonl` |
| UI | React + Vite + R3F; Rust `runner` CLI + axum read API |

**Rejected for MVP:** policy training loop, scenario editor UI, metrics charts,
GMAT export, multi-step RL horizon per env step.

## 2025-05-21 — Mission runner vs RL env

**Decision:** Split rollout environments from mission execution:

- `LeoCircularEnv` / future RL envs → `reset` / `step(action)` for training.
- `MissionRunner` → `run(plan) -> MissionResult` for GMAT-like segment sequences.

Both share `Propagator`, `TerminationCondition`, impulsive burns, and artifact
types. Mission scenarios use TOML `[[segments]]` with orbital stops and burns;
`runner` detects mission vs rollout via `LoadedScenario`.

**First version:** apo/peri stops on the fixed step grid (radial-velocity crossing
with angle fallback for circular orbits), inertial/prograde/circularize burns,
classical element initial orbits, Hohmann LEO→GEO demo scenario.

**Deferred:** `TerminationCondition::AnyOf`, low-thrust action application,
multi-step env horizons.
