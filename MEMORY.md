# Project Memory

## 2026-06-13 — `Angle` is always normalized to `[0, 2π)`

**Decision:** `Angle` normalizes to the half-open turn `[0, 2π)` at construction
(`from_radians`/`from_degrees` apply `rem_euclid(2π)` after the finite check), so
all `Angle` arithmetic wraps. Added a `degrees()` reader to pair with
`from_degrees`. Removed the redundant `normalize_angle`/`angle_from_radians`
helpers in `elements.rs`; element extraction now relies on the type's own
normalization. Narrower-range quantities (inclination in `[0, π]`) are unaffected
because wrapping `[0, 2π)` leaves them unchanged.

**Update (2026-06-13):** `Angle` `Add`/`Sub` are now infallible (`Output = Angle`).
See the operator-validation contract entry below — a total operation must not
advertise a failure it cannot produce.

## 2026-06-13 — Constructor-vs-operator validation contract

**Decision:** Validation lives at the raw-`f64` boundary (constructors), not in
domain arithmetic. Each operator's signature reflects its single real failure
mode:

- **Total operations** (`Angle::add`/`sub`): infallible, return the bare type. A
  private `Angle::wrapped` folds a known-finite value via `rem_euclid`;
  `from_radians` = finite check + `wrapped`.
- **Overflow-only operations** (`TimeStep`/`Distance`/`Speed`/`Duration` `add`,
  `Epoch::advance_by`): infallible. The sum's sign is provable, so the only
  failure is `f64` overflow to `±inf` — a programming error, guarded by
  `debug_assert!(result.is_finite())` rather than `Result`.
- **Genuinely fallible operations** (`Duration::sub`): keep `Result`. Subtracting
  a larger duration is an input-dependent, recoverable outcome
  (`NumericalError::Negative`); finiteness is provable, so it stays a debug
  backstop.

This matches the `AGENTS.md` rule "guard `Add`/`Sub` with
`debug_assert!(result.is_finite())`" and "do not re-assert constraints the type
proves." Constructors keep full `Result` validation since their inputs are
arbitrary.

**Rejected:** Routing operators through full constructors (re-runs dead
finite/sign checks and forces callers to `?`-handle impossible errors); adding
`_unchecked` constructors (the operator owns and documents its own invariant
inline instead).

**Migration:** Dropped `?` at every `advance_by`/operator call site across
`integrators`, `dynamics`, `elements`, `propagator`, and examples; overflow tests
became `#[cfg(debug_assertions)] #[should_panic]`.

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
