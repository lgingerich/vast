# Architecture

Living architecture reference for VAST. This document describes what the
simulator is today, what it is becoming, and the boundaries between layers.

For planned work, see [ROADMAP.md](../ROADMAP.md). For agent and contribution
conventions, see [AGENTS.md](../AGENTS.md). For durable decisions that explain
why the architecture has this shape, see [MEMORY.md](../MEMORY.md).

---

## Purpose

VAST is an idiomatic Rust astrodynamics simulator growing from a readable
physics core into reusable simulation APIs for scenarios, environments,
visualization, and automated experiments.

The primary product goal is an RL training environment for space: many
deterministic, fixed-step, loose- to medium-fidelity orbital rollouts with a
clear `reset(seed)` / `step(action)` contract. VAST is not trying to replace
GMAT, STK, Orekit, or other high-fidelity mission-analysis tools. It should
train, screen, and compare policies at scale, then make it straightforward to
move promising cases into higher-fidelity validation workflows.

That goal shapes the architecture:

- The hot path stays fixed-step, deterministic, and allocation-conscious.
- Physics remains readable and testable as a standalone library layer.
- Simulation owns scenario interpretation, time-loop orchestration, stopping
  rules, maneuvers, environment stepping, and artifact generation.
- UI, CLIs, and examples are thin consumers of reusable library APIs.

---

## Layered Model

The crate is organized in layers. Lower layers must not depend on higher ones.
The current code already follows this direction: `src/physics` has no dependency
on `src/sim`, and `src/sim` uses physics through typed state, dynamics,
integrators, orbit construction, diagnostics, elements, and maneuver helpers.

```text
External consumers
  runner CLI · artifact recorder · local API · React/R3F replay UI · examples
        |
        v
Environment and mission APIs
  reset/step rollout contract · full-episode recording · mission segment runner
        |
        v
Simulation orchestration
  TOML config DTOs · validated runtime plans · propagator · termination · burns
        |
        v
Physics core
  domain types · Cartesian state storage · dynamics · integrators · orbits
  elements · diagnostics · maneuver vector math
```

The important boundary is not merely module layout. It is ownership:

- Physics answers: "given a typed state and a force model, what acceleration or
  next state follows?"
- Simulation answers: "given a configured world and an episode or mission goal,
  how should time advance, when should it stop, what commands apply, and what
  artifacts or observations should be produced?"
- Tools answer: "how should humans or automation invoke and inspect those
  reusable APIs?"

---

## Current Crate Shape

At the crate root, `src/lib.rs` exposes physics as the broad reusable API and
selectively exposes simulation contracts that are stable enough for consumers.
The `sim` module itself is `pub(crate)`, so callers see DTOs, result types, and
top-level operations, not the internal runtime plan or propagator machinery.

### Public Physics Surface

The physics layer is re-exported from the crate root:

- Constants: `MU_EARTH`, `R_EARTH`, `J2_EARTH`.
- Domain scalars and vectors: `Angle`, `AngularRate`, `Epoch`, `TimeStep`,
  `Duration`, `Distance`, `Speed`, `MassDensity`, `AreaPerMass`,
  `GravitationalParameter`, `J2Coefficient`, `Eccentricity`, `Position`,
  `Velocity`, `Acceleration`.
- State storage: `CartesianState`, `CartesianStateComponents`,
  `CartesianStateBuffer<N>`, `CartesianStates`, `CartesianStatesMut`.
- Dynamics: `Dynamics`, `TwoBodyGravity`, `OblateBodyGravity`,
  `ExponentialAtmosphere`, `AtmosphericDrag`, `CompositeDynamics<A, B>`.
- Integrators: `Integrator`, `EulerIntegrator`, `VelocityVerletIntegrator`,
  `LeapfrogIntegrator`, `RungeKutta4Integrator`.
- Orbits and elements: `EllipticOrbit`, `Orbit`, `OrbitDirection`,
  `OrbitalElements`, `OrbitalElementClassification`,
  `ClassicalOrbitalElements`, `CircularInclinedElements`,
  `CircularEquatorialElements`, `EquatorialEllipticElements`.
- Diagnostics: `OrbitalDiagnostics`, `SpecificEnergy`,
  `SpecificAngularMomentum`.
- Propagation convenience: `Sim`, `SimBuilder`, `SimResult`.

`physics::maneuvers` is a module-level API, but its helpers are not re-exported
at the crate root today. Simulation burns consume them internally, and mission
tests use them for expected Hohmann values.

### Public Simulation Surface

The simulation layer exposes:

- Scenario DTOs: `LoadedScenario`, `ScenarioConfig`,
  `MissionScenarioConfig`, `InitialOrbitConfig`, `PropagationConfig`,
  `MissionSegmentConfig`, `StopConfig`, `BurnConfig`, `IntegratorKind`,
  `ForceModelKind`.
- Environment API: `Environment`, `LeoCircularEnv`, `Action`, `Observation`,
  `StepResult`, `RolloutEpisodeResult`.
- Mission API: `run_mission`, `MissionResult`, `MissionSegmentResult`.
- Propagation outputs: `PropagationResult`, `TrajectorySample`.
- Stop reporting: `StopReason`.

The following are intentionally crate-private:

- `RolloutPlan`, `MissionPlan`, `PropagationPlan`, and `MissionSegmentPlan`.
- `Propagator`, `AdvanceResult`, `AccelerationCommand`, and
  `ControlledDynamics`.
- `RuntimeDynamics`.
- `BurnCommand`.
- `TerminationCondition`, `TerminationTracker`, and `TerminationContext`.

This split keeps TOML compatibility and user-facing results public while
preserving freedom to reshape runtime internals as the simulator matures.

---

## Physics Core

`src/physics` is the lower layer. It is library-first, deterministic, and mostly
free of I/O, serialization, or scenario concepts. It owns units, state
representation, force models, numerical stepping, orbit construction and
inspection, diagnostics, and maneuver vector math.

### Physics Module Map

| Module | Responsibility |
| --- | --- |
| `constants` | Earth constants plus classification tolerances for orbital elements. |
| `types` | SI domain newtypes and vector wrappers with constructor invariants. |
| `state` | Boundary `CartesianState`, flat `CartesianStateComponents`, and SoA buffers/views. |
| `dynamics` | Allocation-free `Dynamics` trait and built-in acceleration models. |
| `integrators` | Fixed-step `Integrator` trait and Euler, Verlet, Leapfrog, RK4 implementations. |
| `orbits` | Forward elliptic orbit construction from Keplerian inputs. |
| `elements` | Cartesian-to-elements conversion with singular-safe element variants. |
| `diagnostics` | Two-body orbital invariants and bound-orbit radii. |
| `maneuvers` | State-dependent impulsive delta-v vector helpers and analytical transfer formulas. |
| `sim` | Convenience builder for fixed-duration single-body propagation. |

The modules are deliberately flat. Their conceptual dependency direction is:

```text
constants
types
  -> state
      -> dynamics
      -> diagnostics
      -> integrators
  -> orbits/elements
  -> maneuvers
```

`orbits` and `elements` share an internal `EllipticElementSet` bridge for
perifocal-to-inertial conversion. `maneuvers` sits at the edge: it takes typed
states and returns typed velocity delta-v vectors, but it does not know when a
burn should occur.

### Domain Types

`types.rs` uses `#[repr(transparent)]` scalar newtypes and typed vector wrappers
to make units and invariants visible at API boundaries. Constructors validate
arbitrary `f64` inputs; hot-path arithmetic trusts values that have already
crossed a boundary.

| Type | Unit / meaning | Invariant |
| --- | --- | --- |
| `Angle` | radians | finite, wrapped to `[0, 2pi)` |
| `AngularRate` | rad/s | finite |
| `Epoch` | seconds since simulation epoch | finite |
| `TimeStep` | seconds | finite and strictly positive |
| `Duration` | seconds | finite and non-negative |
| `Distance` | meters | finite and strictly positive |
| `MassDensity` | kg/m^3 | finite and non-negative |
| `AreaPerMass` | m^2/kg | finite and non-negative |
| `Speed` | m/s | finite and non-negative |
| `GravitationalParameter` | m^3/s^2 | finite and strictly positive |
| `J2Coefficient` | dimensionless | finite |
| `Eccentricity` | dimensionless | finite and non-negative |
| `Position` | Cartesian vector, meters | all components finite |
| `Velocity` | Cartesian vector, m/s | all components finite |
| `Acceleration` | Cartesian vector, m/s^2 | all components finite |

Important arithmetic contracts:

- `Angle` normalizes on construction. `Add` and `Sub` are total and return an
  `Angle`, wrapping through the same half-open turn.
- `TimeStep`, `Distance`, `Speed`, and `Duration` addition is infallible because
  the sign invariant is provable for the operation. Debug assertions guard only
  floating-point overflow.
- `Duration::checked_sub` remains fallible because subtracting a larger duration
  is a valid input-dependent outcome.
- `Epoch::advance_by` is infallible for ordinary propagation; overflow is a
  programming error guarded by `debug_assert!`.
- `Epoch::duration_since` assumes forward time and uses a debug assertion to
  catch reversed inputs in debug builds.

The design rule is: validate raw external numbers at construction, then avoid
re-validating domain facts already guaranteed by the type.

### State Storage

`state.rs` separates readable boundary state from dense hot-path storage:

- `CartesianState` is a boundary type: typed `Position` and `Velocity`. Use it
  for orbit construction, scenario setup, burns, samples, diagnostics, tests, and
  public results.
- `CartesianStateComponents` is a flat scalar representation for one state. It
  lets dynamics and integrators read `px_m`, `vy_mps`, and related components
  without rebuilding domain vectors on every access.
- `CartesianStateBuffer<const N: usize>` is fixed-capacity structure-of-arrays
  storage for many active bodies.
- `CartesianStates<'a>` and `CartesianStatesMut<'a>` are borrowed views over the
  active prefix of a buffer.

The public names stay domain-oriented even though the internal layout is SoA.
This is important for future batch stepping: the core can operate on cache-
friendly arrays while callers still pass and receive explicit orbital states.

Current write and read policy:

- `write_from_state` accepts a typed `CartesianState`, converts it to
  components, and writes raw arrays.
- `read_state` rebuilds a typed `CartesianState`, re-validating components
  through `Position` and `Velocity`.
- Integrators write through `write_integrated_vectors`, which validates new
  position and velocity vectors before storing them.
- Index and active-length errors use `StateError`.

Simulation currently uses `CartesianStateBuffer<1>` internally, but the physics
storage API is already shaped for `N > 1` and indexed stepping.

### Dynamics

`dynamics.rs` owns the acceleration contract:

```rust
pub trait Dynamics {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        t: Epoch,
    ) -> Result<Acceleration, DynamicsError>;
}
```

The trait takes flat components by value, receives an `Epoch` for future
time-dependent models, and returns a typed `Acceleration`. Implementations should
not allocate in the stepping path.

Built-in models:

- `TwoBodyGravity` computes point-mass central gravity `-mu r / |r|^3` and
  rejects the central singularity with `DynamicsError::PositionAtGravityCenter`.
- `OblateBodyGravity` adds analytical J2 perturbation using a gravitational
  parameter, equatorial radius, and `J2Coefficient`.
- `ExponentialAtmosphere` models a spherical atmosphere with reference radius,
  reference altitude, density, scale height, and body rotation rate.
- `AtmosphericDrag` is a cannonball drag model over an `ExponentialAtmosphere`
  and `AreaPerMass`; it computes relative velocity against the co-rotating
  atmosphere.
- `CompositeDynamics<A, B>` sums two dynamics models without heap allocation.
  More than two models are represented by nesting.

`Dynamics` does not own control actions, burn schedules, termination conditions,
scenario config, or artifact sampling. The one current low-thrust path is the
simulation-local `ControlledDynamics` adapter in `engine.rs`; it adds an inertial
acceleration command above the selected gravity model for a bounded environment
step.

### Integrators

`integrators.rs` owns fixed-step numerical stepping:

```rust
pub trait Integrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), PropagationError>;
}
```

The caller owns epoch advancement and chooses the body index. Integrators mutate
only the selected indexed body in a borrowed state view.

Implemented integrators:

- `EulerIntegrator`: semi-implicit Euler baseline.
- `VelocityVerletIntegrator`: symplectic default for long conservative orbital
  rollouts.
- `LeapfrogIntegrator`: kick-drift-kick form, equivalent to velocity Verlet for
  position-dependent dynamics and useful for future splitting/control insertion.
- `RungeKutta4Integrator`: fourth-order RK, useful for comparison and
  dissipative or controlled dynamics such as drag and low thrust.

Every integrator uses `write_integrated_vectors` to convert raw computed vectors
back into `Position` and `Velocity`. Non-finite results become
`PropagationError::Numerical`; dynamics and state errors propagate through the
error chain.

Production simulation remains fixed-step. A partial final step is allowed only
as a simulation-layer convenience for landing exactly on a duration stop, not as
adaptive step-size control.

### Orbits And Elements

`orbits.rs` and `elements.rs` handle forward and inverse two-body orbit
representations.

`EllipticOrbit` is the forward initial-condition builder:

- Represents bound ellipses only: eccentricity must satisfy `0 <= e < 1`.
- `e = 0` is the circular convention; argument of periapsis is conventional in
  that case.
- Stores semi-major axis, eccentricity, gravitational parameter, inclination,
  RAAN, argument of periapsis, true anomaly, and `OrbitDirection`.
- Computes instantaneous speed and period.
- Converts to inertial `CartesianState` through the shared internal element-set
  conversion.

`Orbit` is a companion enum that expresses orbital regimes without exposing
the full 8-argument `EllipticOrbit::new` constructor:

- `Orbit::CircularEquatorial { altitude }` — Earth circular equatorial orbit.
- `Orbit::CircularInclined { altitude, inclination, raan }` — Earth circular
  inclined orbit.
- `Orbit::Elliptic { ... }` — all classical elements.

Converts to `EllipticOrbit` via `TryFrom` (Earth gravitational parameter,
convention-filled zero angles for circular variants).

`OrbitalElements` is the inverse and round-trip representation for bound states.
It is an enum because classical elements are singular for circular and
equatorial cases:

- `Classical`: eccentric and inclined; all classical angles are defined.
- `CircularInclined`: circular inclined; argument of periapsis is undefined, so
  argument of latitude is exposed.
- `CircularEquatorial`: circular equatorial; RAAN and argument of periapsis are
  undefined, so true longitude is exposed.
- `EquatorialElliptic`: eccentric equatorial; RAAN is undefined, so longitude of
  periapsis is exposed.

`OrbitalElementClassification` owns the tolerances for deciding when to use a
singular-safe variant. Defaults come from `constants.rs`:

- `DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE = 1.0e-6`.
- `DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD = 1.0e-10`.

The inverse path rejects unbound states and zero angular momentum. Simulation
uses `OrbitalElements::orbital_phase_angle()` for apoapsis and periapsis
detection; it does not need to know which element variant was selected.

### Diagnostics

`diagnostics.rs` computes two-body orbital invariants from either a
`CartesianState` or an indexed `CartesianStates` view:

- radius and inertial speed
- specific orbital energy
- specific angular momentum vector and magnitude
- eccentricity vector and scalar eccentricity
- optional semi-major axis, periapsis radius, and apoapsis radius for bound
  elliptical states

Diagnostics are analysis tools, not implicit work in every propagation step.
Tests and examples use them to check conservation, drag decay, and orbital
conversion behavior. Future sampled diagnostics should remain explicit at sample
boundaries so batch rollouts do not pay diagnostic cost unless requested.

### Maneuvers

`maneuvers.rs` owns vector math for impulsive delta-v calculations:

- `prograde_delta_v(delta_v_mps, state)` returns a velocity vector aligned with
  current inertial velocity and rejects zero speed.
- `circularize_delta_v(state, mu)` removes radial velocity and matches local
  circular speed in the current tangential direction.
- `hohmann_delta_v(mu, departure_radius, arrival_radius)` returns analytical
  two-body circular-orbit transfer burns when finite.
- `bielliptic_delta_v(mu, departure_radius, arrival_radius, intermediate_radius)`
  returns analytical bi-elliptic burns when the intermediate radius is valid.

This module intentionally does not decide burn timing. `src/sim/burns.rs` maps
scenario burn commands to these vector helpers and applies them at simulation
step boundaries.

---

## Simulation Layer

`src/sim` turns physics capabilities into repeatable rollouts and missions. It
owns file-format DTOs, validation into runtime plans, a fixed-step single-body
propagator, step-boundary burns, stop conditions, a mission runner, and the MVP
RL environment.

### Simulation Module Map

| Module | Visibility | Responsibility |
| --- | --- | --- |
| `config` | public DTOs | TOML-facing scenario and mission shapes plus `LoadedScenario` detection. |
| `plan` | crate-private | Converts config into validated runtime plans using domain types. |
| `engine` | mixed | Public propagation results; crate-private single-body `Propagator`. |
| `termination` | mixed | Public `StopReason`; crate-private runtime stop conditions and trackers. |
| `burns` | crate-private | Runtime `BurnCommand` and state update logic for impulsive burns. |
| `environment` | public API | `Environment` trait, `LeoCircularEnv`, actions, observations, step results. |
| `mission` | public API | `run_mission` and mission result types over segment plans. |
| `mod` | crate-private | Module wiring. |

The key pattern is: public config and result types at the edge; private runtime
types inside the crate.

### Scenario Config Boundary

`config.rs` is the only simulation module that owns TOML shape and serde
compatibility. It uses `deny_unknown_fields` for scenario structs and tagged
enums for structured mission concepts.

`LoadedScenario` dispatches a TOML document into:

- `LoadedScenario::Mission` when `kind = "mission"` or a top-level `segments`
  key is present.
- `LoadedScenario::Rollout` otherwise.

`ScenarioConfig` is the MVP rollout format:

- `name`
- `altitude_m`
- `dt_s`
- `duration_s`
- `integrator`
- `force_model`
- `sample_every_steps`
- `steps_per_env_step`

It always represents a circular equatorial LEO rollout around Earth today.
`steps_per_env_step` defaults to `1`, and `force_model` defaults to
`two_body`.

`MissionScenarioConfig` is the structured multi-segment mission format:

- `name`
- `kind`
- `initial_orbit`
- shared `propagation`
- `initial_burns`
- ordered `segments`

Mission initial orbit options:

- `InitialOrbitConfig::Circular { altitude_m }`.
- `InitialOrbitConfig::Classical { semi_major_axis_m, eccentricity,
  inclination_deg, raan_deg, arg_periapsis_deg, true_anomaly_deg }`.

Shared propagation fields:

- `dt_s`
- `integrator`
- `force_model`
- `sample_every_steps`

Segment stop options:

- `StopConfig::Apoapsis`
- `StopConfig::Periapsis`
- `StopConfig::Duration { duration_s }`
- `StopConfig::StepCount { steps }`

Burn options:

- `BurnConfig::Inertial { delta_v_mps: [f64; 3] }`
- `BurnConfig::Prograde { delta_v_mps }`
- `BurnConfig::Circularize`

Supported integrators in config:

- `leapfrog`
- `velocity_verlet`
- `euler`
- `runge_kutta4` with alias `rk4`

Supported force models in config (internally tagged via `kind`):

- `two_body` — point-mass central gravity
- `j2` — oblate central gravity with J2 (parameters: `equatorial_radius_m`, `j2`)
- `j2_drag` — J2 gravity composed with exponential atmospheric drag
  (parameters: `j2` fields plus `reference_altitude_m`,
  `reference_density_kg_m3`, `scale_height_m`, `angular_rate_rad_s`,
  `area_per_mass_m2_kg`)

Each force model resolves to a variant of the closed `RuntimeDynamics` enum at
plan-build time. The propagator dispatches on this enum in `step_once`, so
each arm calls into a statically-known dynamics type with no trait objects on
the stepping path.

### Runtime Plans

`plan.rs` is the invariant gate between raw config and execution. It converts
public DTOs into crate-private runtime plans:

- `RolloutPlan`: initial state, propagation plan, duration termination, and
  `steps_per_env_step`.
- `MissionPlan`: initial state, propagation plan, initial burns, and ordered
  segment plans.
- `PropagationPlan`: typed `TimeStep`, `IntegratorKind`, `RuntimeDynamics`,
  and sample cadence.
- `MissionSegmentPlan`: runtime `TerminationCondition` and post-segment
  `BurnCommand`s.

`RuntimeDynamics` is a closed enum wrapping the concrete `Dynamics` models
(`TwoBodyGravity`, `OblateBodyGravity`, `CompositeDynamics<OblateBodyGravity,
AtmosphericDrag>`). It exposes `mu()` so burn application can resolve orbital
elements without matching on the variant.

Validation and conversion performed here:

- Scenario names must be non-empty.
- Rollout durations must be positive.
- `steps_per_env_step` must be at least `1`.
- `sample_every_steps` must be at least `1`.
- Mission `kind` must be `mission`.
- Missions must have at least one segment.
- `dt_s` becomes `TimeStep`.
- `duration_s` becomes `Duration`.
- circular altitudes become `Distance(R_EARTH + altitude_m)` and an
  `EllipticOrbit` with `e = 0`.
- classical initial orbit fields become typed `Distance`, `Eccentricity`, and
  `Angle`s, then `ClassicalOrbitalElements::to_cartesian_state`.
- burn DTOs become `BurnCommand`s.
- stop DTOs become `TerminationCondition`s.

The runtime gravitational parameter is carried by `RuntimeDynamics::mu()`;
the underlying value is Earth `MU_EARTH` for all current force models.

### Propagator Engine

`engine.rs` is the shared execution center. The public output types are:

- `TrajectorySample { epoch, state, segment_index }`.
- `PropagationResult { final_epoch, final_state, steps, stop_reason, samples }`.

The internal `Propagator` owns:

- current `Epoch`
- initial `Epoch`
- validated `PropagationPlan` (with `RuntimeDynamics`)
- `CartesianStateBuffer<1>` with body index `0`

This is intentionally a single-body simulation engine today. The physics storage
and integrator APIs can step indexed bodies, but the simulation API has not yet
introduced a batch or multi-body propagator contract.

The full segment loop is `Propagator::run_segment`:

1. Create an empty `Vec<TrajectorySample>`.
2. Initialize `TerminationTracker` from the start state.
3. Record the initial sample at step `0`.
4. At each step boundary, build a `TerminationContext`.
5. Check the termination condition before taking another step.
6. If a max-step budget exists and has been reached, return without a stop
   reason.
7. For duration stops, shorten the next step when the remaining duration is
   smaller than the configured `dt`.
8. Dispatch the selected integrator for one body at index `0`.
9. Advance epoch by the step.
10. Read and validate the updated state.
11. Advance termination tracking.
12. Record a sample when `steps == 0` or the step count is a multiple of
    `sample_every_steps`.
13. Return `PropagationResult` once a terminal stop reason is found.

`run_segment` requires a terminal stop reason. If the internal advance loop
returns without one, `PropagationError::SegmentWithoutStop` is raised.

`Propagator::advance_steps` is the bounded environment path. It advances at most
`max_steps` and returns an internal `AdvanceResult` whose `stop_reason` is
`Option<StopReason>`. Reaching the environment horizon without satisfying the
episode termination is normal and not an error.

Current dynamics selection:

- The propagator holds a `RuntimeDynamics` enum. Each `step_once` wraps the
  active variant in sim-local `ControlledDynamics`, which adds the
  piecewise-constant inertial acceleration command to the base dynamics
  acceleration. A zero command still flows through `ControlledDynamics` (the
  add of a zero vector is free), keeping one code path with no per-step
  branching on the command magnitude.
- Each match arm calls into a statically-known dynamics type, so the
  acceleration computation is monomorphized per force model with no trait
  objects on the stepping path.

Current allocation behavior:

- The integrator and dynamics stepping path is allocation-free.
- `run_segment` allocates and grows a `Vec<TrajectorySample>` for output
  recording.
- `advance_steps` does not collect trajectory samples.

### Termination

`termination.rs` separates public stop reporting from private runtime conditions.

Public `StopReason` variants:

- `DurationMet`
- `StepCountMet`
- `StepBudgetExceeded`
- `ApoapsisMet`
- `PeriapsisMet`

Current note: `StepBudgetExceeded` is part of the public vocabulary and artifact
name mapping, but the current sim path does not assign it. Bounded environment
steps return `None` when the per-step horizon is exhausted before the episode
condition is met.

Private `TerminationCondition` variants:

- `Duration(Duration)`
- `StepCount(usize)`
- `Apoapsis { mu, min_steps }`
- `Periapsis { mu, min_steps }`

Termination is evaluated at fixed step boundaries. There is no root finding or
adaptive event localization.

Duration stops are segment-relative. `run_segment` captures the segment start
epoch and compares `current_epoch.duration_since(segment_start_epoch)` against
the configured duration. Duration stops also allow a final partial step to land
exactly on the requested elapsed duration.

Step-count stops fire when the number of integrator steps taken in the segment
is at least the configured limit.

Apoapsis and periapsis stops use `TerminationTracker`:

- radial velocity is `(r dot v) / |r|`
- apoapsis can fire on outward-to-inward radial velocity sign crossing
- periapsis can fire on inward-to-outward radial velocity sign crossing
- orbital phase angle from `OrbitalElements` is a fallback, with a small angular
  tolerance, including circular orbit cases where radial velocity may be nearly
  zero
- a minimum-step guard prevents immediate stop at the starting boundary

This gives GMAT-like "propagate to apoapsis/periapsis" behavior while preserving
fixed-step determinism.

### Burns

`burns.rs` owns runtime impulsive burn commands:

- `BurnCommand::Inertial([f64; 3])`
- `BurnCommand::Prograde(f64)`
- `BurnCommand::Circularize`

Construction validates finite delta-v components or magnitudes. Application
reads the current `CartesianState`, computes a typed delta-v, adds it to the
current velocity, validates the new velocity, and writes a new
`CartesianState`.

Ownership is split intentionally:

- `physics::maneuvers` owns prograde and circularization vector math.
- `sim::burns` owns runtime command validation, timing integration, and state
  update orchestration.
- `Propagator::apply_burn` owns applying the command to body index `0` at a
  step or segment boundary.

Burns are impulsive and discrete. They are not force models, and they are not
inside `Dynamics::acceleration_at`.

### Mission Runner

`mission.rs` provides the GMAT-style sequence runner:

```text
config.build()
  -> Propagator::new(initial_state, epoch 0, propagation)
  -> apply initial_burns
  -> for each segment:
       run_segment(segment.termination, segment_index)
       apply segment.burns
  -> MissionResult
```

Public result types:

- `MissionSegmentResult { segment_index, propagation, burns_applied }`.
- `MissionResult { segments, final_state, final_epoch }`.

Segment `PropagationResult`s are the canonical owners of per-segment stop
reasons and sampled trajectories. `MissionResult` does not duplicate a flattened
sample list. Artifact recording flattens samples at the recording boundary when
needed.

Burn timing:

- `initial_burns` apply before segment `0`.
- Segment `burns` apply after that segment completes.
- The final mission state is read after all segment burns have been applied.

### Environment API

`environment.rs` is the current RL contract:

```text
reset(seed) -> Observation
step(action) -> StepResult
```

Public action and observation shapes:

- `Action` is a finite inertial acceleration vector `[f64; 3]` in m/s^2.
- `Observation` is flat and copyable:
  - inertial position `[f64; 3]` in meters
  - inertial velocity `[f64; 3]` in meters per second
  - altitude above `R_EARTH` in meters
  - inertial speed in meters per second

`StepResult` contains:

- observation after the step
- reward
- done flag
- optional stop reason
- number of integrator steps advanced by this environment step

`LeoCircularEnv` is the MVP environment:

- Built from `ScenarioConfig` via `LeoCircularEnv::from_config`.
- Uses the rollout `RolloutPlan`.
- Owns a `Propagator`.
- `reset(seed)` restores epoch and state to the validated initial state. The
  seed is accepted by the contract but currently unused.
- `step(action)` advances at most `steps_per_env_step` integrator steps with the
  action applied as piecewise-constant inertial acceleration.
- If the env is already done, `step` returns the current observation, `done =
  true`, `steps = 0`, and no new stop reason.
- Reward is `0.0` until reward functions are implemented.
- `run_episode(seed)` is the explicit full-rollout recording path. It resets,
  runs the full termination segment, ensures at least one sample, marks the env
  done, and returns `RolloutEpisodeResult`.

Full-episode artifact recording is deliberately not the `step(action)` contract.
`step` is bounded and suitable for RL loops; recording is explicit.

---

## End-To-End Data Flow

### Rollout Scenario

```text
ScenarioConfig / rollout TOML
  -> ScenarioConfig::build()
      validates fields
      resolves ForceModelKind to RuntimeDynamics
      constructs circular Earth initial state
      creates PropagationPlan
      creates Duration termination
  -> LeoCircularEnv::from_config()
      creates Propagator at epoch 0
  -> reset(seed)
      restores initial state
      returns Observation
  -> step(Action)
      validates finite inertial acceleration
      advances at most steps_per_env_step integrator steps
      returns StepResult
  -> run_episode(seed)
      full run_segment path for recording
      returns RolloutEpisodeResult with PropagationResult samples
```

### Mission Scenario

```text
MissionScenarioConfig / mission TOML
  -> MissionScenarioConfig::build()
      validates mission structure
      resolves ForceModelKind to RuntimeDynamics
      constructs circular or classical initial state
      validates shared propagation settings
      maps stops to TerminationCondition
      maps burns to BurnCommand
  -> run_mission()
      creates Propagator at epoch 0
      applies initial burns
      runs each segment to its stop condition
      applies post-segment burns
      returns MissionResult
```

### Propagation Loop

```text
Propagator::advance
  start epoch/state
  initialize TerminationTracker
  optionally record initial sample
  loop:
    build TerminationContext
    if termination.stop_reason(context):
      return final state, epoch, steps, stop reason
    if max_steps reached:
      return final state, epoch, steps, no stop reason
    choose dt or duration remainder
    wrap active RuntimeDynamics variant in ControlledDynamics
    dispatch IntegratorKind to physics integrator
    epoch = epoch.advance_by(step)
    read typed state from CartesianStateBuffer<1>
    update termination tracker
    optionally record sample
```

### Artifact And UI Flow

The RL ops shell sits above simulation:

```text
scenarios/*.toml
      |
      v
runner run
      |
      v
runs/<run_id>/manifest.json
runs/<run_id>/metrics.jsonl
runs/<run_id>/episodes/<episode>/trajectory.jsonl
      |
      v
runner serve read API
      |
      v
ui/ React + R3F replay/dashboard/compare views
```

Physics never moves into the UI. The UI reads artifacts produced by Rust.

---

## Error Architecture

Errors form a layer chain. Lower layers do not know about higher-layer context,
and `#[from]` conversions let `?` propagate failures upward.

```text
NumericalError
  -> DynamicsError
  -> StateError
  -> PropagationError
  -> MissionError
  -> EnvironmentError
```

More precisely:

- `NumericalError` represents domain validation failures:
  `NotFinite`, `NonPositive`, `Negative`, `ZeroNorm`, and
  `ConstraintViolation`.
- `DynamicsError` wraps `NumericalError` and adds
  `PositionAtGravityCenter`.
- `StateError` wraps `NumericalError` and adds invalid index/length errors.
- `PropagationError` wraps dynamics, state, and numerical errors and adds
  invalid burns plus segment-without-stop.
- `MissionError` wraps propagation and numerical errors and adds file read,
  TOML parse, and config-field validation failures.
- `EnvironmentError` wraps mission, propagation, and recording failures.

Boundary guidance:

- Domain constructors return `NumericalError`.
- Dynamics models return `DynamicsError`.
- State storage returns `StateError`.
- Integrators and propagators return `PropagationError`.
- Scenario loading/building and mission execution return `MissionError`.
- Environment APIs return `EnvironmentError`.

---

## Determinism, Allocation, And Batch Readiness

These constraints apply across the architecture:

- Production propagation is fixed-step.
- Duration termination may use one final partial step to land exactly on the
  requested duration.
- Stop and burn behavior snaps to step boundaries.
- Integrators and dynamics should not allocate in the step loop.
- State storage uses SoA layout for future batch stepping.
- Simulation currently steps one body with `CartesianStateBuffer<1>`.
- Samples allocate only on explicit recording paths.
- Environment `step(action)` is bounded by `steps_per_env_step`.
- Mission segment loops require explicit stop conditions.
- No mutual N-body or coupled dynamics are claimed until stepping uses a
  read-old/write-new strategy or another deterministic batch-safe design.

Adaptive integrators may be useful later for offline validation, but they are
not part of the production RL/batch path.

---

## Implemented Baseline

The architecture currently includes:

- typed SI domain values and finite vector wrappers
- Earth constants and orbit-classification tolerances
- boundary `CartesianState` and SoA `CartesianStateBuffer<N>`
- `Dynamics` and `Integrator` traits
- two-body, J2, drag, and composite dynamics in physics
- Euler, velocity Verlet, leapfrog, and RK4 fixed-step integrators
- elliptic orbit initial-condition construction and `Orbit` convenience enum
- singular-safe bound orbital elements and phase angle extraction
- orbital diagnostics for analysis and tests
- prograde, circularize, Hohmann, and bi-elliptic maneuver helpers
- `Sim` builder for fixed-duration single-body propagation convenience
- TOML rollout and mission scenario DTOs
- config-to-runtime plan validation
- single-body fixed-step `Propagator`
- two-body, J2, and J2+drag force models wired from scenario config through to
  the `RuntimeDynamics` dispatch enum
- duration, step-count, apoapsis, and periapsis termination
- runtime impulsive burn commands
- mission runner over ordered segments
- `LeoCircularEnv` with `reset` and bounded `step(action)`
- full rollout recording path with trajectory samples
- runner CLI, local read API, and replay/dashboard UI above the library

---

## Current Gaps And Deferred Work

These are known gaps in the current implementation, not contradictions in the
architecture:

- `Propagator` is single-body and owns `CartesianStateBuffer<1>`; batch
  simulation is future work despite physics supporting indexed buffers.
- `StepBudgetExceeded` exists as a public stop reason but is not assigned by the
  current bounded environment path.
- Environment reward is always `0.0`.
- `reset(seed)` stores the seed only in the API shape; seeded perturbations are
  not implemented.
- Boundary/constraint checks such as impact, escape, and altitude bands are not
  implemented.
- General event condition/action lists are not implemented. Apoapsis/periapsis
  stops are runtime termination conditions, not a separate event system.
- Low thrust exists only as a per-step inertial acceleration command above
  two-body gravity; there is no actuator, mass depletion, attitude model, or
  thrust profile scheduler.
- Mission scripting is structured Rust/TOML config, not a DSL.
- The UI reads saved artifacts only; it does not run physics or propagation in
  JavaScript.

See [ROADMAP.md](../ROADMAP.md) for prioritized follow-on work.

---

## Module Boundary Rules

| Concern | Belongs in | Must not leak into |
| --- | --- | --- |
| raw TOML field names and serde compatibility | `sim::config` | physics, integrators, dynamics |
| config validation into domain values | `sim::plan` | physics constructors beyond their own invariants |
| state vector math and units | `physics::*` | runner/UI code |
| force evaluation | `physics::dynamics` | scenario DTOs, artifact formats |
| numerical stepping | `physics::integrators` | TOML config, UI |
| step loop, epoch ownership, sampling | `sim::engine` | dynamics models |
| termination conditions | `sim::termination` | physics force models |
| burn timing and command application | `sim::burns`, `sim::mission`, `sim::engine` | `Dynamics::acceleration_at` |
| burn vector math | `physics::maneuvers` | config parsing |
| RL action mapping | `sim::environment`, `sim::engine` | physics integrator trait |
| mission sequencing | `sim::mission` | physics core |
| run artifacts | `recording` | physics and simulation hot path |
| visualization | examples/tools/UI | core physics and simulation APIs |

Keep frame and unit conversions visible at boundaries. Do not hide coordinate
system changes inside unrelated algorithms.

---

## Design Influences

### GMAT

VAST borrows the separation between resource/config initialization and mission
sequence execution:

- configure spacecraft/orbits/propagator/burns
- propagate until a stop condition
- apply maneuvers
- report sampled outputs

VAST does not currently borrow GMAT's script language, adaptive propagation
default, SPICE integration, or high-fidelity force catalog.

### Basilisk

VAST borrows the idea that stop times and terminal events are first-class
simulation concerns. It does not currently borrow Basilisk's multi-rate
process/task/module graph or message-passing architecture.

### VAST-Specific Constraints

VAST's north star is deterministic RL rollout throughput:

- fixed timesteps
- batch-friendly state layout
- domain typed setup and results
- explicit bounded work
- loose- to medium-fidelity force models before high-fidelity validation paths

---

## Open Design Questions

Some MVP decisions are recorded in [MEMORY.md](../MEMORY.md). Remaining
questions:

1. **Central body ownership**: introduce a first-class `CentralBody` with
   gravitational parameter, radius, J2, rotation, and atmosphere parameters, or
   keep Earth constants directly in scenario planning until non-Earth scenarios
   are active?
2. **Spacecraft properties**: introduce a spacecraft/body property bundle for
   mass, drag area, and future SRP, or keep force-model parameters external
   now that drag is wired into scenarios as `area_per_mass`?
3. **Event model**: keep apoapsis/periapsis as termination variants only, or add
   a bounded condition/action event list for logging, burns, and terminal
   actions?
4. **Environment adapters**: keep the Rust `Environment` trait as the core
   contract and put Gymnasium or other RL framework bindings in tools, or expose
   adapter types from the crate?
5. **Batch API shape**: expose a simulation-layer `step_many` over many
   independent environments, or first add a lower-level CPU backend abstraction
   over `CartesianStateBuffer<N>`?
6. **2D mode**: represent 2D as constrained 3D states with `z = 0`, or introduce
   separate first-class 2D state/orbit types if training workloads need them?

Resolved for the current MVP:

- TOML config is separate from crate-private runtime plans.
- Runtime termination is a closed enum.
- Environment `step(action)` is bounded.
- Inertial acceleration actions are applied above `Dynamics`.
- Impulsive burns are step-boundary commands, not dynamics models.
- Mission execution shares the same `Propagator` as rollout recording.
- `PropagationResult` owns segment stop reason and sampled trajectory data.
- The runner/recorder flatten or serialize simulation results at the artifact
  boundary, not inside `MissionResult`.
- The propagator dispatches on a closed `RuntimeDynamics` enum in `step_once`;
  each arm calls into a statically-known dynamics type, so the acceleration
  computation is monomorphized per force model with no trait objects on the
  stepping path.

---

## Related Documents

- [ROADMAP.md](../ROADMAP.md) - prioritized unimplemented work.
- [AGENTS.md](../AGENTS.md) - contribution and maintenance rules.
- [MEMORY.md](../MEMORY.md) - architectural decisions log.
