//! Core physics types and stepping logic for the space simulator.

mod constants;
mod diagnostics;
mod dynamics;
mod elements;
mod environment;
mod errors;
mod integrators;
mod mission;
mod orbits;
mod propagator;
mod recording;
mod scenario;
mod state;
mod termination;
mod types;

pub use constants::{J2_EARTH, MU_EARTH, R_EARTH};
pub use diagnostics::{Eccentricity, OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy};
pub use dynamics::{
    AtmosphericDrag, CompositeDynamics, Dynamics, ExponentialAtmosphere, OblateBodyGravity,
    TwoBodyGravity,
};
pub use elements::{
    CircularEquatorialElements, CircularInclinedElements, ClassicalOrbitalElements,
    EquatorialEllipticElements, J2SecularRates, OrbitalElementClassification, OrbitalElements,
    j2_circular_raan_rate, j2_circular_secular_rates,
};
pub use environment::{Action, Environment, LeoCircularEnv, Observation, StepResult};
pub use errors::{
    DynamicsError, EnvironmentError, IntegrationError, MissionError, PropagationError,
    RecordingError, ScenarioError, StateError,
};
pub use integrators::{
    EulerIntegrator, Integrator, LeapfrogIntegrator, RungeKutta4Integrator,
    VelocityVerletIntegrator,
};
pub use mission::{
    BuiltMissionScenario, BurnConfig, BurnSpec, InitialOrbitConfig, LoadedScenario, MissionResult,
    MissionRunner, MissionScenarioConfig, MissionSegment, MissionSegmentConfig,
    MissionSegmentResult, PropagationConfig as MissionPropagationConfig, StopConfig, StopTable,
    hohmann_delta_v_mps,
};
pub use orbits::{CircularOrbit, CircularOrbitOrientation, Orbit, OrbitDirection};
pub use propagator::{
    ImpulsiveBurn, PropagationResult, Propagator, SampleConfig, TrajectorySample,
};
pub use recording::{
    EpisodeMetrics, EpisodeRecord, MissionSegmentMetrics, RunManifest, RunRecorder,
    TrajectoryPoint, list_runs, read_trajectory, stop_reason_name,
};
pub use scenario::{BuiltScenario, IntegratorKind, ScenarioConfig};
pub use state::{
    CartesianAccelerationComponents, CartesianPositionComponents, CartesianState,
    CartesianStateBuffer, CartesianStateComponents, CartesianStates, CartesianStatesMut,
    CartesianVelocityComponents,
};
pub use termination::{StopReason, TerminationCondition};
pub use types::{
    Acceleration, Angle, AngularRate, AreaPerMass, Distance, Duration, Epoch,
    GravitationalParameter, MassDensity, Position, Speed, TimeStep, Velocity,
};
