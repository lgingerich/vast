//! Core physics types and stepping logic for the space simulator.

mod errors;
mod physics;
mod recording;
pub(crate) mod sim;

pub use physics::{
    constants::{J2_EARTH, MU_EARTH, R_EARTH},
    diagnostics::{Eccentricity, OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy},
    dynamics::{
        AtmosphericDrag, CompositeDynamics, Dynamics, ExponentialAtmosphere, OblateBodyGravity,
        TwoBodyGravity,
    },
    elements::{
        CircularEquatorialElements, CircularInclinedElements, ClassicalOrbitalElements,
        EquatorialEllipticElements, J2SecularRates, OrbitalElementClassification, OrbitalElements,
        j2_circular_raan_rate, j2_circular_secular_rates,
    },
    integrators::{
        EulerIntegrator, Integrator, LeapfrogIntegrator, RungeKutta4Integrator,
        VelocityVerletIntegrator,
    },
    orbits::{CircularOrbit, CircularOrbitOrientation, Orbit, OrbitDirection},
    state::{
        CartesianAccelerationComponents, CartesianPositionComponents, CartesianState,
        CartesianStateBuffer, CartesianStateComponents, CartesianStates, CartesianStatesMut,
        CartesianVelocityComponents,
    },
    types::{
        Acceleration, Angle, AngularRate, AreaPerMass, Distance, Duration, Epoch,
        GravitationalParameter, MassDensity, Position, Speed, TimeStep, Velocity,
    },
};

pub use errors::{
    DynamicsError, EnvironmentError, IntegrationError, MissionError, PropagationError,
    RecordingError, ScenarioError, StateError,
};

pub use sim::{
    environment::{Action, Environment, LeoCircularEnv, Observation, StepResult},
    mission::{
        BuiltMissionScenario, BurnConfig, BurnSpec, InitialOrbitConfig, LoadedScenario,
        MissionResult, MissionRunner, MissionScenarioConfig, MissionSegment, MissionSegmentConfig,
        MissionSegmentResult, PropagationConfig as MissionPropagationConfig, StopConfig, StopTable,
        bielliptic_delta_v_mps, hohmann_delta_v_mps,
    },
    propagator::{ImpulsiveBurn, PropagationResult, Propagator, SampleConfig, TrajectorySample},
    scenario::{BuiltScenario, IntegratorKind, ScenarioConfig},
    termination::{StopReason, TerminationCondition},
};

pub use recording::{
    EpisodeMetrics, EpisodeRecord, MissionSegmentMetrics, RunManifest, RunRecorder,
    TrajectoryPoint, list_runs, read_trajectory, stop_reason_name,
};
