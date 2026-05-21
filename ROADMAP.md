# Roadmap

Unimplemented work recommended for Space Sim, grouped by category. Each item
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
integrators, orbital diagnostics, classical element conversion (Cartesian →
elements), J2 secular-rate checks, and SoA Cartesian state stepping.

---

## Simulation Layer

The physics core can integrate forces; the simulation layer orchestrates time
evolution, stopping rules, maneuvers, events, and outputs. This is the main gap
between the current library and a useful simulator. See
[docs/architecture.md](docs/architecture.md).

### P1 — Simulation Layer Definition (Design)

**What:** Decide and document the first-version simulation API: how `Scenario`,
`Propagator`, termination, and optional control hooks compose; what lives in
the crate vs examples; naming and module layout.

**Why:** "Simulation" spans scenario setup, propagation orchestration, stops,
maneuvers, events, boundaries, and environment wrappers. These pieces interact.
Committing to propagator-only first without a agreed scenario/termination shape
risks rework.

**How it helps:** Gives a stable target for P1 propagator and P1/P2 control
work; resolves open questions in the architecture doc before large public API
surface lands.

**First version scope:**

- Written decision in `MEMORY.md` covering: scenario config shape, propagator
  ownership, termination representation (enum vs trait), where burns/thrust
  attach, and deferrals for environment/mission-sequence APIs.
- Update `docs/architecture.md` to match.
- No requirement to implement everything decided — this item is the design gate.

**Defer:** mission-script DSL, full environment trait design, multi-spacecraft
synchronization modes.

**Open questions to resolve:**

- One `Scenario` struct vs builder vs trait family?
- Spacecraft-specific type vs generic body with property bundles?
- Termination as closed enum (batch-friendly) vs extensible trait?
- Events folded into termination (GMAT-style) vs separate condition/action
  lists (Basilisk-style)?

### P1 — Propagator API

**What:** A reusable propagation layer that owns the repeated pattern in
examples: state buffer, epoch, integrator, dynamics, bounded stepping, optional
final partial timestep, and hooks for termination checks.

**Why:** Examples currently duplicate propagation loops. A shared API is the
foundation for scenarios, maneuvers, events, diagnostics sampling, and batch
stepping.

**How it helps:** One place to advance time; examples become thin; future
simulation features attach at step boundaries without touching `Dynamics`.

**First version scope:**

- Fixed-step loop over `CartesianStatesMut`.
- Optional shortened final step when remaining time is less than `dt`.
- Caller-provided dynamics and integrator; no hidden allocation in the step loop.
- Optional fixed substep count inside each propagation interval.
- Step budget limit (max steps) as a safety bound.
- Callback or return-value hook to evaluate `TerminationCondition` each step.
- `PropagationResult` with final epoch, state, and stop reason.

**Defer:** adaptive timestep control, event-time root finding, output
interpolation, internal event dispatch (initial hook only).

### P1 — Termination Conditions

**What:** Typed end-state rules for propagation segments: elapsed duration, step
count, and a composable "first satisfied wins" list.

**Why:** Every useful simulator stops for a reason. Today examples hard-code
`while epoch < duration`. GMAT's `Propagate {StopList}` is the reference model.

**How it helps:** Makes "propagate for one period", "propagate 86400 s", and
later "propagate to apoapsis" the same API surface.

**First version scope:**

- `Duration` / target epoch.
- `StepCount`.
- `AnyOf([...])` — stop on first match.
- Integration with propagator; stop reason reported in result.
- Tests: exact landing on duration via partial final step; step count; AnyOf
  ordering.

**Defer:** apoapsis, periapsis, altitude thresholds (see Events); backward
propagation; root finding on stop time.

### P2 — Scenario Configuration Type

**What:** A declarative static setup bundling central body (or raw constants
initially), initial state, force stack, integrator, `dt`, termination, and
optional output cadence — constructible without running propagation.

**Why:** Examples and future environments currently reassemble physics
primitives ad hoc. GMAT's Initialization section is the analogue.

**How it helps:** Named, testable, reusable rollouts; propagator constructed
from scenario; examples shrink to scenario + run + report.

**First version scope:**

- Builder or struct holding propagator inputs and termination.
- Validation at build time where possible (positive dt, finite ICs).
- Earth/two-body example scenario mirroring `orbit_period`.
- Burns/thrust slots as optional attachments (may be no-op until control items
  land).

**Defer:** serialization/loading, multi-spacecraft scenarios, scenario catalogs.

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

### P2 — Trajectory And Output Sampling

**What:** Caller-provided or fixed-capacity buffers for state and/or diagnostics
snapshots at a configured cadence (every N steps or at sample epochs).

**Why:** Visualization and analysis need history; the hot path should not grow
unbounded `Vec`s. Examples already use local trail buffers (`TRAIL_POINTS_MAX`).

**How it helps:** Separates propagation from recording; supports export and
viz without allocation in the step loop when capacity is fixed.

**First version scope:**

- Sample every N steps or on propagator sample hook.
- Store `CartesianState` and/or `OrbitalDiagnostics` in caller buffer.
- Explicit capacity; drop or stop sampling when full (documented behavior).

**Defer:** unbounded history, interpolation between samples, device-side sampling.

### P2 — Resettable Environment Wrapper

**What:** A thin experiment/RL-oriented layer: `reset(seed) → obs`,
`step(action) → (obs, reward, done, info)` composing scenario + propagator.

**Why:** README Tier 4 and automated experiments need deterministic reset and a
stable observation/action contract without polluting the physics core.

**How it helps:** Curriculum learning, parameter sweeps, and policy evaluation
share the same propagation stack as analysis examples.

**First version scope:**

- Map action to low-thrust command for one step interval.
- Deterministic reset from scenario + seed (IC perturbation or noise TBD).
- Typed observation (e.g. position, velocity, diagnostics subset).
- `done` from termination + boundary checks.
- Simple reward hook or zero reward placeholder.

**Defer:** multi-agent, vectorized env batching (see Backend), complex reward
engineering, Gymnasium/RL crate adapters in core.

### P3 — Mission Segments And Propagation Sequences

**What:** Explicit multi-segment propagation: `propagate_until(stop) →
apply_burn → propagate_until(stop)` as a small Rust API over propagator
operations.

**Why:** GMAT mission sequences are the user-facing model for transfers and
rendezvous. A DSL is not required; typed segment runners are enough initially.

**How it helps:** Hohmann transfer, plane change, and rendezvous demos without
copy-pasted example loops.

**First version scope:**

- Segment result chaining with stop reason per segment.
- Compose with impulsive burns and termination from earlier items.

**Defer:** script interpreter, GUI mission tree, synchronized multi-spacecraft
propagation.

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

### P2 — Orbit Construction From Classical Elements

**What:** Durable orbit setup APIs beyond `CircularOrbit`, using the existing
`OrbitalElements` conversions for elliptical, inclined, and retrograde cases.

**Why:** Circular orbit helpers are useful for tests and viz, but most real
scenarios need eccentricity, inclination, RAAN, argument of periapsis, and true
anomaly.

**How it helps:** Makes scenario setup readable and removes one-off state
construction from examples and future RL environments.

**First version scope:**

- Keplerian/classical element to `CartesianState` at epoch.
- Round-trip tests against diagnostics and two-body invariants.

**Defer:** mean anomaly propagation, hyperbolic/parabolic classification APIs,
coordinate-frame catalogs.

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

### P1 — Controlled Low Thrust

**What:** A fixed-step control layer that applies piecewise-constant commanded
acceleration at step boundaries, e.g. `a_total = gravity + commanded_acceleration`.

**Why:** Low thrust is the primary force model for RL actions and spacecraft
behavior. It is local, deterministic when actions are fixed per step, and
batch-friendly across independent environments.

**How it helps:** Exposes a useful control surface for policies, station-keeping
experiments, and transfer-curve curricula without waiting for full propulsion or
attitude modeling.

**First version scope:**

- Commands applied only at step boundaries.
- Inertial Cartesian commanded acceleration.
- Thrust-to-mass or direct acceleration input; no propellant depletion yet.
- Composed in propagator or a thin `ControlledDynamics` wrapper — not inside
  individual force models.

**Defer:** mass depletion, thrust pointing constraints, actuator lag, attitude
coupling, body-frame or LVLH thrust until frame APIs exist.

### P2 — Impulsive Burns

**What:** Step-boundary velocity increments from an explicit bounded burn
schedule.

**Why:** Essential for maneuver experiments. GMAT models these as `Maneuver` /
`ImpulsiveBurn`, not as continuous forces.

**How it helps:** Enables Hohmann transfers, plane changes, and rendezvous
curricula without event-time root finding or finite-burn modeling.

**First version scope:**

- Burns only at fixed step boundaries or at event-triggered step indices.
- Explicit, bounded burn schedules as scenario/control operations.
- Separate from the `Dynamics` acceleration path.

**Defer:** event-triggered burns without step grid, Lambert targeting, finite
burn arcs.

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
