//! VAST — Vectorized Astrodynamics Simulation Toolkit.

mod errors;
mod physics;
mod recording;
pub(crate) mod sim;

pub use physics::{
    constants::{J2_EARTH, MU_EARTH, R_EARTH},
    diagnostics::{DiagnosticSeries, OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy},
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
    orbits::Orbit,
    orbits::OrbitDirection,
    sim::{Sim, SimBuilder, SimResult},
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
    config::{
        BurnConfig, ForceModelKind, InitialOrbitConfig, IntegratorKind, LoadedScenario,
        MissionScenarioConfig, MissionSegmentConfig, PropagationConfig, ScenarioConfig, StopConfig,
    },
    engine::{PropagationResult, TrajectorySample},
    environment::{
        Action, Environment, LeoCircularEnv, Observation, RolloutEpisodeResult, StepResult,
    },
    mission::{MissionResult, MissionSegmentResult, run_mission},
    termination::StopReason,
};

pub use recording::{
    EpisodeMetrics, EpisodeRecord, MissionSegmentMetrics, RunManifest, RunRecorder,
    TrajectoryPoint, list_runs, read_trajectory, stop_reason_name,
};
