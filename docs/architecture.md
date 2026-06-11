# Architecture

Living architecture reference for VAST. This document describes what the
simulator is today, what it is becoming, and the boundaries between layers.

For planned work, see [ROADMAP.md](../ROADMAP.md). For agent and contribution
conventions, see [AGENTS.md](../AGENTS.md).

---

## Purpose

VAST is an idiomatic Rust astrodynamics simulator growing from a readable
physics core into reusable simulation APIs for scenarios, environments,
visualization, and automated experiments.

The long-term goal is not just to integrate forces. It is to provide a
**simulation contract**: named setups, controlled time evolution, stopping rules,
maneuvers, observations, and repeatable rollouts — the non-physics layers that
make propagation useful for mission analysis, experiments, and control research.

---

## Layered Model

The crate is organized in layers. Lower layers must not depend on higher ones.

```text
┌──────────────────────────────────────────────────────────────┐
│  Environment (optional)                                      │
│  reset · step(action) · observation · reward · done          │
│  For RL, sweeps, curriculum, automated experiments           │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  Mission / Segments (optional, later)                        │
│  Propagate → Maneuver → Propagate sequences                  │
│  GMAT-style mission timeline over a propagator               │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  Scenario (static configuration)                             │
│  central body · spacecraft ICs · forces · dt · burns ·       │
│  termination · events · output cadence                       │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  Propagator (runtime orchestrator)                           │
│  epoch · buffer · step loop · apply burns/thrust ·           │
│  check termination · detect events · sample outputs          │
└────────────────────────────┬─────────────────────────────────┘
                             │
┌────────────────────────────▼─────────────────────────────────┐
│  Physics core (implemented today)                            │
│  state · dynamics · integrators · domain types · diagnostics │
└──────────────────────────────────────────────────────────────┘
```

Examples and visualization binaries are **thin consumers** of these layers.
They should not reimplement propagation loops once a shared propagator exists.

---

## What Exists Today

### Physics core (implemented)

| Module | Responsibility |
|--------|----------------|
| `state` | SoA `CartesianStateBuffer`, boundary `CartesianState`, indexed views |
| `dynamics` | Force models and `Dynamics` trait |
| `integrators` | Fixed-step integrators and `Integrator` trait |
| `types` | SI domain newtypes with validation |
| `orbits` | `EllipticOrbit` initial-condition construction (circular via `e=0`) |
| `elements` | Cartesian → classical elements, J2 secular helpers |
| `diagnostics` | Two-body orbital invariants |
| `constants` | Earth μ, radius, J2 |

### Core traits

**`Dynamics`** — state-only acceleration evaluation. No control inputs, no side
effects, no allocation in the hot path.

**`Integrator`** — advances one indexed body in `CartesianStatesMut` by one
fixed `TimeStep`. Caller owns epoch advancement.

### What examples do today

Examples wire physics primitives manually: build an `EllipticOrbit`, create a
`CartesianStateBuffer`, choose integrator and dynamics, then run a hand-written
`while epoch < duration` loop with optional final partial steps. Scenario structs
(e.g. `OrbitVizScenario`) exist only inside example files.

There is **no library-level** scenario, propagator, termination, event, control,
or environment API yet.

---

## Simulation vs Physics

**Physics** answers: given state and time, what is the acceleration?

**Simulation** answers: given a configured world and a goal, how does state
evolve over time, when does propagation stop, what maneuvers fire, and what
gets observed?

| Concept | Physics layer | Simulation layer |
|---------|---------------|------------------|
| State | `CartesianState`, SoA buffer | Owned by propagator/scenario |
| Forces | `Dynamics` implementations | Selected and composed in scenario |
| Time stepping | `Integrator::step` | Propagator loop, partial final step |
| Initial conditions | Orbit/state construction | Scenario setup |
| End state | — | Termination conditions |
| Maneuvers | — | Burns/thrust at step boundaries |
| Events | — | Detect + react at step boundaries |
| Boundaries | — | Impact, escape, altitude limits |
| Outputs | `OrbitalDiagnostics` (on demand) | Sampled history, reports, exports |
| Experiments | — | Resettable environment wrapper |

### External reference models

**GMAT** (mission-analysis style) separates:

1. **Initialization** — create resources (spacecraft, propagator, forces, burns)
   with literal configuration.
2. **Mission sequence** — imperative commands: `Propagate` until stop conditions,
   `Maneuver`, `Report`, etc.

Stop conditions (`ElapsedSecs`, `Apoapsis`, `Periapsis`, altitude thresholds) are
first-class. Maneuvers are discrete operations, not continuous forces.

**Basilisk** (multi-rate GNC style) uses processes, tasks, modules, message
passing, hard stop time, and events with condition/action functions and optional
`terminal` termination.

VAST aligns more closely with **GMAT's propagate + stop + maneuver**
model, adapted for **fixed-step, batch-friendly** execution. Events snap to the
step grid rather than using adaptive root finding, preserving determinism for
batched rollouts and future GPU backends.

---

## Planned Simulation Concepts

These types are design targets. Names and module layout may change; boundaries
should not.

### Scenario

Static, declarative configuration for one rollout:

- Central body (μ, radius, rotation, atmosphere parameters)
- Spacecraft initial state (classical elements or Cartesian)
- Force model stack per body
- Integrator choice and `TimeStep`
- Optional burn and thrust schedules
- Termination conditions (first match wins)
- Optional bounded event list
- Output sampling cadence

Scenarios should be constructible without running propagation. Invalid
combinations should fail at build/validation time when possible.

### Propagator

Runtime owner of the time loop:

- Holds epoch, state buffer, integrator, dynamics, and scenario-derived config
- Steps until a termination condition is met or a step budget is exhausted
- Applies burns and commanded thrust at step boundaries (not inside `Dynamics`)
- Supports final partial step to land exactly on a target duration
- Optional fixed substeps per interval for accuracy without breaking batch sync
- Invokes event checks and output sampling hooks at step boundaries
- No hidden allocation in the default step loop

### Termination conditions

End states for a propagation segment. First version targets:

- `Duration` / elapsed time
- `StepCount`
- Later: apoapsis, periapsis, altitude threshold (step-boundary detection)

Composite conditions (whichever occurs first) mirror GMAT stop lists. Root
finding for exact event times is deferred; see ROADMAP.

### Controls and maneuvers

**Not part of `Dynamics`.**

- **Impulsive burns** — Δv applied at step boundaries from an explicit schedule
- **Low thrust** — piecewise-constant commanded inertial acceleration added to
  gravity at step boundaries

This matches GMAT's separation of `Maneuver` from force models and keeps the
integrator hot path simple.

### Events

Bounded detect-and-react hooks at step boundaries:

- Kinds: apoapsis, periapsis, altitude crossing, step index, custom predicate
- Actions: apply burn, log, terminate propagation, disable self
- Step-grid snapping only in the first version (no adaptive event integrator)

### Boundaries and constraints

Physical or scenario limits that can terminate or flag invalid state:

- Surface impact (radius below body equatorial radius)
- Escape (positive specific energy for closed-orbit scenarios)
- Altitude floor/ceiling bands

Distinct from termination in intent: boundaries often mean "invalid" or
"catastrophic" rather than "mission segment complete."

### Environment

Optional wrapper for automated experiments and RL-style use:

```text
reset(seed) → initial observation
step(action) → (observation, reward, done, info)
```

Environments compose a scenario + propagator (or segment runner), map actions to
thrust commands, and expose typed observations and rewards. They belong above
the propagator, not inside dynamics.

### Mission segments (later)

A thin sequence over propagator operations:

```text
propagate_until(Apoapsis)
apply_burn(delta_v)
propagate_until(Periapsis)
```

Full mission-script or DSL support is not required for early versions. Explicit
Rust sequences over a propagator API are enough.

---

## Execution Strategy

These constraints apply across all layers:

- **Fixed-step only** on the production/batch path. Every body in a batch
  advances by the same `dt` and step count per decision interval.
- **Allocation-free stepping** in integrators and dynamics.
- **Per-body force evaluation** without mutual N-body coupling until a
  read-old/write-new update strategy exists.
- **Deterministic** rollouts for replay, tests, and batched environments.
- **Bounded work** — explicit limits on steps, events, history length, burn count.

Adaptive integrators may exist later as an **offline validation** tool only.
They must not become the default batch or GPU path.

---

## Module Boundaries

| Concern | Belongs in | Must not leak into |
|---------|------------|-------------------|
| Force evaluation | `dynamics` | propagator internals, examples |
| Numerical stepping | `integrators` | scenario config |
| Orbit/state setup | `orbits`, `elements`, future `CentralBody` | integrator hot loop |
| Time loop, stops, burns | future `propagator`, `scenario` | `Dynamics::acceleration_at` |
| Observations/rewards | future `environment` | physics core |
| Rendering | examples/tools | public simulation API |
| Visualization deps | examples (`macroquad`) | core crate public API |

Keep coordinate-system and unit changes visible at conversion boundaries. Do not
hide frame changes inside unrelated algorithms.

---

## Data Flow (Target)

```text
Scenario::build()
    → Propagator::new(scenario)
    → loop:
        apply pending burns/thrust at step boundary
        integrator.step(dynamics, states, index, epoch, dt)
        epoch += dt
        if termination_met() → break
        if event_triggered() → run action / maybe break
        if sample_due() → record diagnostics/state snapshot
    → PropagationResult { final state, epoch, stop reason, samples }
```

For environments, `step(action)` maps the action to thrust for the next
interval, calls one propagator step (or a fixed substep count), computes
observation and reward, and returns `done` from termination/boundary checks.

---

## Design Influences

| Source | What we borrow | What we defer |
|--------|----------------|---------------|
| GMAT | Init vs mission sequence, propagate + stop conditions, maneuvers separate from forces, calculation parameters | Script language, adaptive RK default, SPICE/ephemeris |
| Basilisk | Stop time, terminal events, explicit condition/action split | Multi-rate process/task/module graph, message passing |
| This project | Fixed-step batch path, SoA state, typed SI units, allocation-free hot loop | — |

---

## Open Design Questions

Some MVP decisions are recorded in [MEMORY.md](../MEMORY.md). Remaining questions:

1. **Spacecraft vs generic body** — First-class spacecraft type with mass/drag,
   or generic bodies with optional property bundles?
2. **Event model** — Fold into termination (GMAT-style) or separate condition/
   action lists (Basilisk-style)?
3. **Environment API adapters** — Gymnasium/RL crate bindings in core vs
   examples/tools?
4. **2D mode** — First-class 2D types vs 3D with `z = 0` constraint?

Resolved for MVP: TOML `ScenarioConfig`, closed `TerminationCondition` enum,
`Environment` trait with flat observation, thrust stubbed above `Dynamics`.

---

## RL Ops MVP (implemented slice)

The first product shell connects scenario files, headless rollouts, on-disk
artifacts, a local read API, and a web replay UI.

```text
scenarios/*.toml
      ↓
runner run  →  runs/<run_id>/manifest.json
               runs/<run_id>/metrics.jsonl
               runs/<run_id>/episodes/0000/trajectory.jsonl
      ↓
runner serve  →  GET /api/runs, /api/runs/:id, .../metrics, .../trajectory
      ↓
ui/ (React + R3F)  →  runs dashboard + per-run metrics + 3D replay + compare
```

### MVP contracts

| Piece | Shape |
|-------|-------|
| Scenario | TOML → `ScenarioConfig` |
| Termination | `Duration` or `StepCount` (closed enum) |
| Environment | `reset(seed)` / `step(action)`; one full rollout per step |
| Action | `[f64; 3]` thrust accel (stubbed) |
| Observation | 8 floats: position, velocity, altitude, speed |
| Artifacts | JSON manifest + JSONL metrics and trajectories |

Physics stays in the Rust library; the UI reads artifacts only (no JS integrator).

---

## Related Documents

- [ROADMAP.md](../ROADMAP.md) — prioritized unimplemented work
- [AGENTS.md](../AGENTS.md) — contribution and maintenance rules
- [MEMORY.md](../MEMORY.md) — architectural decisions log
