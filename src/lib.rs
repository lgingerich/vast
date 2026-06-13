//! VAST — Vectorized Astrodynamics Simulation Toolkit.

mod errors;
mod physics;
mod recording;
pub(crate) mod sim;

pub use physics::{
    constants::{J2_EARTH, MU_EARTH, R_EARTH},
    diagnostics::{OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy},
    dynamics::{
        AtmosphericDrag, CompositeDynamics, Dynamics, ExponentialAtmosphere, OblateBodyGravity,
        TwoBodyGravity,
    },
    elements::{
        CircularEquatorialElements, CircularInclinedElements, ClassicalOrbitalElements,
        EquatorialEllipticElements, OrbitalElementClassification, OrbitalElements,
    },
    integrators::{
        EulerIntegrator, Integrator, LeapfrogIntegrator, RungeKutta4Integrator,
        VelocityVerletIntegrator,
    },
    orbits::EllipticOrbit,
    orbits::OrbitDirection,
    state::{
        CartesianState, CartesianStateBuffer, CartesianStateComponents, CartesianStates,
        CartesianStatesMut,
    },
    types::{
        Acceleration, Angle, AngularRate, AreaPerMass, Distance, Duration, Eccentricity, Epoch,
        GravitationalParameter, J2Coefficient, MassDensity, Position, Speed, TimeStep, Velocity,
    },
};

pub use errors::{
    DynamicsError, EnvironmentError, MissionError, NumericalError, PropagationError,
    RecordingError, StateError,
};

pub use sim::{
    environment::{Action, Environment, LeoCircularEnv, Observation, StepResult},
    mission::{
        BuiltMissionScenario, BurnConfig, BurnSpec, InitialOrbitConfig, LoadedScenario,
        MissionResult, MissionRunner, MissionScenarioConfig, MissionSegment, MissionSegmentConfig,
        MissionSegmentResult, StopConfig, StopTable,
    },
    propagator::{ImpulsiveBurn, PropagationResult, Propagator, SampleConfig, TrajectorySample},
    scenario::{BuiltScenario, IntegratorKind, ScenarioConfig},
    termination::{StopReason, TerminationCondition},
};

pub use recording::{
    EpisodeMetrics, EpisodeRecord, MissionSegmentMetrics, RunManifest, RunRecorder,
    TrajectoryPoint, list_runs, read_trajectory, stop_reason_name,
};
