# Roadmap

Unimplemented work recommended for VAST, grouped by category. Each item
has a rough priority of **P1** (next), **P2** (soon after), or **P3** (later).
Only these three levels are used.

This roadmap assumes the current execution strategy: deterministic fixed-timestep
stepping, dense cache-friendly state storage, allocation-free integration paths,
and per-body force evaluation without pairwise coupling unless a future design
explicitly supports read-old/write-new stepping.

See [docs/architecture.md](docs/architecture.md) for the layered simulator model
this roadmap implements toward.

Implemented baseline (not listed below): two-body and J2 gravity, exponential
atmospheric drag, `CompositeDynamics`, fixed-step Euler/RK4/Verlet/Leapfrog
integrators, orbital diagnostics, classical element conversion (Cartesian ↔
elements), J2 secular-rate checks, SoA Cartesian state stepping, **circular
and elliptic orbit initial-condition construction (`EllipticOrbit`,
including circular via `e=0` convention) and the **`Orbit` enum convenience
variants (`Orbit::CircularEquatorial`, `Orbit::CircularInclined`,
`Orbit::Elliptic`)**.  **`Sim` builder** for fixed-duration physics-layer
propagation without termination/sampling.  **propagator
with duration/step/orbital termination and trajectory sampling**, **impulsive
burns**, **TOML scenario config**, **`LeoCircularEnv` (`reset` / bounded
`step(action)` with inertial acceleration command)**,
**mission scenarios (`MissionRunner`, segment stops/burns)**, **run artifact
recording with segment metadata**, **`runner` CLI (`run`, `serve`)**,
**React + R3F replay UI (`ui/`)**, **RL ops dashboard with per-run metrics
(reward, episode length, stop reasons, final altitude) and synchronized
multi-episode comparison view**, **two-body/J2/J2+drag force models wired from
scenario config through the `RuntimeDynamics` dispatch enum**.

---

## Simulation Layer

The physics core can integrate forces; the simulation layer orchestrates time
evolution, stopping rules, maneuvers, events, and outputs. MVP rollout +
replay is implemented; items below extend it toward full RL training.

### P2 — Environment Rewards And Seeded Perturbations

**What:** Pluggable reward functions and deterministic IC perturbation from
`reset(seed)` for training diversity.

**Why:** MVP returns zero reward and identical ICs each reset (seed stored only).

**First version scope:**

- Scenario-defined reward kind (e.g. altitude tracking error, fuel penalty stub).
- Optional small position/velocity noise from seed.

**Defer:** learned reward shaping, multi-objective rewards.

### P2 — RL Ops UI: Cross-Run Comparison And Event Markers

**What:** Build on the in-place per-run dashboard and synchronized
multi-episode compare view by adding cross-run comparison (two or more policies'
aggregate metrics side by side), burn/event markers on the replay scrubber, and
jump-to-next-event controls.

**Why:** The single-run dashboard answers "how did this policy perform?"; the
next gap is "how do two policies compare?" and "what happened when?".

**First version scope:**

- Multi-run selector on the runs page that opens a `/compare?runs=a,b` route.
- Overlay reward / length / stop-reason histograms across runs.
- Burn and segment-transition tick layer on the replay scrubber with
  next/prev-event keyboard shortcuts.

**Defer:** hosted job queue UI, scenario editor forms, ECI ↔ ECEF view toggle
(separate roadmap item), WebSocket streaming of in-progress runs.

### P2 — Boundary And Constraint Checks

**What:** Post-step checks for physically or scenario-invalid state: surface
impact, escape from closed orbit, altitude floor/ceiling violations.

**Why:** Distinct from mission termination — boundaries often mean "invalid" or
"catastrophic" rather than "segment complete". Needed for reentry, landing, and
RL done signals.

**How it helps:** Propagator can flag `StopReason::Impact` vs
`StopReason::DurationMet`; environments can assign penalty/done.

**First version scope:**

- Impact: radius below central body equatorial radius (spherical).
- Escape: specific energy > 0 for scenarios that expect closed orbits.
- Optional altitude band violation.
- Checked at step boundaries; reported in propagation result.

**Defer:** collision geometry, terrain, atmospheric entry interface, soft
boundaries.

### P2 — Event Detection At Step Boundaries

**What:** Bounded detect-and-react hooks: apoapsis, periapsis, altitude
threshold crossing, step index — inspected between fixed steps.

**Why:** Maneuver planning, curriculum triggers, and GMAT-style "propagate to
apoapsis then burn" need orbital events without adaptive integrators.

**How it helps:** Enables segment boundaries and event-triggered actions on the
fixed grid.

**First version scope:**

- Step-boundary detection only; snap events to the fixed grid.
- Explicit, bounded event list per scenario/propagator config.
- Actions: log, terminate, mark for burn application (integration with impulsive
  burns).
- One-shot vs repeating event semantics documented.

**Defer:** continuous root finding, collision-time substeps, adaptive burn timing.

---

## Propagation & Integrators

The core propagation contract is **fixed-step only**: every body in a batch advances
by the same `dt` and the same number of steps per decision interval. This keeps
CPU batching, SIMD, GPU kernels, and RL environment rollouts deterministic and
uniform. Adaptive integrators may exist later as an optional offline validation
tool, but they must not become part of the batch backend or device stepping path.

### P2 — Integrator Benchmark Suite

**What:** A bounded benchmark/example that compares fixed-step Euler, RK4,
Verlet, and Leapfrog on the same two-body and J2 scenarios using position drift,
speed error, specific energy drift, angular-momentum drift, and eccentricity
growth.

**Why:** Integrator choice depends on the force model. Conservative symplectic
integrators are best for long gravity rollouts; RK4 is better for drag, thrust,
and other non-conservative effects. Fixed-step benchmarks are the practical way
to choose production `dt` for batched execution.

**How it helps:** Documents expected trade-offs in-repo, prevents regressions when
new force models or integrators are added, and replaces adaptive error control
for the core simulator path.

**First version scope:**

- Reuse the `orbit_period`-style comparison with exact-period final partial steps.
- Report relative conserved-quantity errors, not just absolute position drift.
- Recommend production `dt` choices from convergence sweeps rather than runtime
  step-size adaptation.

**Defer:** adaptive integrator comparisons; those belong outside the batch path.

### P3 — Adaptive Runge-Kutta Integrator (Offline Validation Only)

**What:** An optional single-body analysis integrator such as Dormand-Prince 5(4)
or RKF45 with absolute/relative tolerances, min/max step bounds, and rejected-step
handling.

**Why:** Tools like GMAT use adaptive Runge-Kutta for mission analysis, and an
offline adaptive path can help validate fixed-step choices or compare one
trajectory against external tools.

**How it helps:** Useful for convergence studies and external-tool comparison, but
**not** for batched RL rollouts, GPU kernels, or any path that requires all
environments to stay phase-locked.

**Why not core:** Adaptive stepping gives different bodies different substep counts
at the same simulation time. That breaks uniform batch stepping, causes GPU warp
divergence, and conflicts with deterministic fixed-horizon environment design.

**First version scope:**

- Single-body/offline API only; never part of `step_many`, CUDA kernels, or RL
  environment stepping.
- Explicit tolerances and step bounds at the API boundary.
- Deterministic settings for replayable validation experiments.

**Defer:** multistep methods, Bulirsch-Stoer, automatic stiffness handling,
batch-capable adaptive propagation, device-side adaptive integrators.

**Do not use as default:** choose fixed-step Verlet, Leapfrog, or RK4 for the
production simulator; use benchmark sweeps to pick `dt` instead.

---

## Orbit Setup & Domain Model

### P2 — `CentralBody` Domain Type

**What:** A body type owning `mu`, equatorial radius, rotation rate, and later
gravity/atmosphere parameters, instead of scattering Earth constants through
scenarios.

**Why:** Scenarios currently assemble gravity, drag, and diagnostics from raw
constants. A body type makes Mars/Moon/Sun setups cleaner and keeps units and
invariants in one place.

**How it helps:** Simplifies scenario construction and prepares for multi-body
environments without leaking body-specific constants into examples.

**First version scope:**

- Earth-first body definition using existing constants.
- Conversion helpers to existing gravity and atmosphere models.

**Defer:** ephemeris-attached bodies, body catalogs, external SPICE/data files.

### P3 — Spacecraft Properties Bundle

**What:** Optional spacecraft metadata (mass, drag area, SRP area, identifiers)
attached to scenario bodies, distinct from pure point-mass state.

**Why:** Drag and future SRP need `AreaPerMass` and related parameters at scenario
level; conflating them with `CartesianState` obscures ownership.

**How it helps:** Scenario describes *which* spacecraft with *what* properties;
force models read immutable params per body index.

**First version scope:**

- Mass and drag parameters for single-body scenarios.
- Wired into existing `AtmosphericDrag` setup.

**Defer:** propellant tanks, thruster models, attitude-dependent areas.

---

## Control & Maneuvers

Controls and maneuvers live **above** `Dynamics`, applied at step boundaries.
Low thrust is tracked under **Simulation Layer → P1 Controlled Low Thrust**.
Impulsive burns and mission segment burns are implemented; finite-burn arcs and
Lambert targeting remain future work.

---

## Diagnostics & Validation

### P2 — Sampled Diagnostics Path

**What:** A reporting-oriented diagnostics API that computes orbital invariants
only at explicit sample boundaries, integrated with propagator/output sampling.

**Why:** Diagnostics are fine in examples today, but batch rollouts should not
pay for ecc/energy/h every step unless requested.

**How it helps:** Preserves correctness tooling for tests and analysis while
keeping core stepping lean.

**First version scope:**

- Keep `OrbitalDiagnostics::from_state` as the analysis entry point.
- Propagator/report hooks for periodic sampling.
- Prefer relative drift metrics (`ΔE/E`, `Δh/h`) in examples and tests.

**Defer:** per-step diagnostic storage, automatic event extraction from diagnostics.

---

## Backend & Acceleration

### P2 — Batched Backend Abstraction

**What:** A backend boundary for stepping many independent bodies or environments
per call, with a CPU implementation first and the existing typed state APIs kept
at the host/domain boundary.

**Why:** The current `Integrator` API steps one indexed body at a time. That is
readable and useful for tests, but GPU and large-batch CPU execution need a
coarser `step_many`-style entry point to amortize dispatch, preserve dense SoA
access, and keep the hot loop allocation-free.

**How it helps:** Creates a stable place to choose CPU, future CUDA, or other
parallel backends without leaking device APIs into orbit construction,
diagnostics, force-model setup, or examples.

**First version scope:**

- CPU backend only, implemented in terms of existing state storage, force models,
  and integrators.
- Closed backend configuration enums for integrator and force-model choices where
  dynamic Rust traits would not map cleanly to device kernels.
- Explicit host/device-neutral status reporting for invalid body states.
- Parity tests against the existing one-body `Integrator` path.

**Defer:** GPU device memory ownership, async streams, multi-GPU scheduling,
runtime code generation, and any backend-specific public state type until the CPU
backend contract is stable.

### P3 — CUDA Backend Prototype

**What:** Optional CUDA backend that mirrors `CartesianStateBuffer`'s SoA layout
in device buffers and launches one thread per independent body/environment for
closed force-model/integrator combinations.

**Why:** CUDA can accelerate large batches, but only if the simulator avoids
per-body launches and does not try to move Rust domain types, `Result`, trait
objects, or `nalgebra` operations into kernels.

**How it helps:** Gives a path to high-throughput training and large parameter
sweeps while preserving deterministic fixed-step semantics and the CPU backend as
the correctness reference. Adaptive integrators are explicitly out of scope
because variable substep counts break uniform batch/GPU execution.

**First version scope:**

- Feature-gated backend, not part of the default build.
- Start with `VelocityVerlet` plus `TwoBodyGravity` only.
- Use plain POD kernel parameters and SoA device buffers.
- Maintain a device status buffer and map kernel failures to typed host errors.
- CPU/GPU parity tests over fixed deterministic rollouts with explicit tolerances.

**Defer:** drag, J2, composite force models, adaptive integrators (offline
validation only; never on device), history sampling on device, shared-memory
optimizations, and GPU support for mutually coupled N-body dynamics.

### P3 — Rust GPU Toolchain Evaluation

**What:** Evaluate cuda-oxide or similar Rust-to-PTX tooling on a small isolated
kernel after the backend boundary exists.

**Why:** Rust-authored kernels could reduce language-boundary friction, but early
GPU compiler/toolchain APIs may change and should not shape the production core
before the simulator has a stable backend contract.

**How it helps:** Keeps the door open for Rust-native kernels while protecting
the main library from experimental toolchain churn.

**First version scope:**

- Prototype only the same `VelocityVerlet` plus `TwoBodyGravity` kernel used by
  the CUDA backend prototype.
- Compare generated-kernel behavior, launch ergonomics, build complexity, and
  CPU parity results against the plain CUDA/PTX path.
- Keep any dependency optional and outside the default feature set.

**Defer:** committing to cuda-oxide as the production backend, rewriting existing
force models for device Rust, async GPU execution graphs, and advanced CUDA
features until the toolchain is proven on this simulator's numerical workload.

---

## Perturbations

### P2 — Solar Radiation Pressure

**What:** A simple cannonball SRP model with analytical Sun direction, e.g.
`a_srp = -P_sun * CrA_over_m * sun_direction` (optionally with distance scaling).

**Why:** Local per-body acceleration, deterministic with fixed or analytically
prescribed Sun state, and batch-friendly when Sun vectors are computed once per
epoch and shared.

**How it helps:** Adds long-duration and higher-orbit realism after LEO-relevant
drag and thrust, especially for drift, orbit-raising, and debris-like dynamics.

**First version scope:**

- Fixed Sun direction or simple analytical circular Sun model.
- No eclipse shadowing or panel geometry.

**Defer:** eclipse transitions, attitude-dependent projected area, high-fidelity
ephemerides.

### P3 — Prescribed Third-Body Gravity

**What:** Perturbation from prescribed third bodies (Sun/Moon) using
`a = mu3 * ((r3 - r) / |r3 - r|^3 - r3 / |r3|^3)`, with third-body state
computed deterministically and cached per epoch.

**Why:** Each spacecraft still evaluates independently; Sun/Moon state can be
shared across a batch. Deterministic when the ephemeris model is analytical and
fixed.

**How it helps:** Improves lunar transfer, GEO, and deep-space scenarios beyond
central-body plus J2, without mutual N-body coupling.

**First version scope:**

- Fixed circular Sun/Moon approximation or other deterministic analytical
  ephemeris.
- Cached third-body vectors per timestep or epoch.

**Defer:** mutual N-body coupling, pairwise spacecraft interactions, external
ephemeris files.

---

## Gravity Field Fidelity

### P3 — Analytical J2 + J4 Gravity

**What:** Extend the current oblate gravity model with an analytical J4 term before
introducing full spherical-harmonic infrastructure.

**Why:** J2 captures most LEO secular effects, but J4 improves oblateness fidelity
with much less complexity than a harmonic field expansion.

**How it helps:** Better long-arc validation for Earth orbit scenarios while staying
allocation-free and compatible with fixed-step batch stepping.

**First version scope:**

- Extend `OblateBodyGravity` or add a sibling model with explicit J2/J4 coefficients.
- Tests against known equatorial/polar acceleration differences and secular trends.

**Defer:** tesseral terms, sectorial terms, coefficient tables.

### P3 — Truncated Spherical Harmonic Gravity (EGM2008 Reference)

**What:** A high-fidelity optional gravity model using truncated spherical
harmonics, with EGM2008 as the reference coefficient set.

**Why:** Full EGM2008 is the standard for high-precision Earth gravity in tools
like GMAT/STK/Orekit, but it is too expensive and too complex for the default
simulator hot path.

**How it helps:** Enables offline validation and mission-analysis modes for
long-arc propagation, GEO/MEO scenarios, and external-tool comparisons.

**First version scope:**

- Explicit low-degree truncation first (for example 8x8 or 16x16), not full 2190x2190.
- Separate high-fidelity mode, not the default RL/training gravity model.
- Body-fixed evaluation with explicit Earth rotation and deterministic coefficient storage.

**Defer:** full-degree EGM2008, external coefficient files, tesseral resonance
studies, surface-impact gravity modeling near singularities.

**Do not use yet as default:** EGM2008 should remain a later validation path, not
the next gravity upgrade after J2.

---

## Design Checklist

Before adding a roadmap item, verify:

- Can acceleration be evaluated independently per body?
- Does the model avoid hidden allocation in the hot path?
- Is external data fixed, deterministic, or cached per epoch?
- Does it work with fixed timesteps?
- Are units represented with domain types at public boundaries?
- Are singularities and invalid physical states explicit?
- Can tests cover a sign, magnitude, or conservation/decay property?
- Does simulation-layer code stay out of `Dynamics::acceleration_at`?
