use thiserror::Error;

/// Error returned when a domain value cannot be constructed because it
/// violates physical finiteness or sign constraints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum DomainError {
    /// The value is NaN, +infinity, or -infinity.
    #[error("value must be finite")]
    NotFinite,
    /// The value is zero or negative but must be strictly positive.
    #[error("value must be positive")]
    NonPositive,
    /// The value is negative but must be non-negative.
    #[error("value must be non-negative")]
    Negative,
}

/// Errors produced while evaluating a dynamics model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum DynamicsError {
    /// The computed acceleration was not finite.
    #[error("computed acceleration was not finite")]
    InvalidAcceleration,
    /// A domain value was physically invalid.
    #[error(transparent)]
    Domain(#[from] DomainError),
    /// Gravity is undefined at the central body's center of mass.
    #[error("two-body gravity is undefined at zero radius")]
    PositionAtGravityCenter,
}

/// Errors produced while reading or writing Cartesian state storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum StateError {
    /// A requested body index was outside the active state range.
    #[error("body index is outside the active state range")]
    InvalidBodyIndex,
    /// A requested active length exceeded the storage capacity.
    #[error("active state length exceeds storage capacity")]
    InvalidLength,
    /// Borrowed state component slices did not have equal lengths.
    #[error("state component slices have mismatched lengths")]
    MismatchedSliceLengths,
    /// A position component was not finite.
    #[error("position component was not finite")]
    InvalidPosition,
    /// A velocity component was not finite.
    #[error("velocity component was not finite")]
    InvalidVelocity,
}

/// Errors produced while advancing a simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum IntegrationError {
    /// The dynamics model could not produce a valid acceleration.
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
    /// The state storage could not be read or written.
    #[error(transparent)]
    State(#[from] StateError),
    /// A domain value was physically invalid.
    #[error(transparent)]
    Domain(#[from] DomainError),
    /// The integrated position was not finite.
    #[error("integrated position was not finite")]
    InvalidPosition,
    /// The integrated velocity was not finite.
    #[error("integrated velocity was not finite")]
    InvalidVelocity,
    /// The integration epoch was not finite.
    #[error("integration epoch was not finite")]
    InvalidEpoch,
}

/// Errors produced while propagating a simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PropagationError {
    /// The state storage could not be read or written.
    #[error(transparent)]
    State(#[from] StateError),
    /// Integration failed during propagation.
    #[error(transparent)]
    Integration(#[from] IntegrationError),
    /// A domain value was physically invalid.
    #[error(transparent)]
    Domain(#[from] DomainError),
    /// The propagation epoch was not finite.
    #[error("propagation epoch was not finite")]
    InvalidEpoch,
    /// A computed integration step was invalid.
    #[error("integration step was invalid")]
    InvalidStep,
    /// An impulsive burn could not be applied to the current state.
    #[error("impulsive burn is invalid for the current state")]
    InvalidBurn,
}

/// Errors produced while loading or building a scenario.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum ScenarioError {
    /// The scenario file could not be read.
    #[error("failed to read scenario file: {0}")]
    ReadFile(String),
    /// The scenario file could not be parsed.
    #[error("failed to parse scenario file: {0}")]
    ParseFile(String),
    /// A scenario field was invalid.
    #[error("invalid scenario field `{field}`: {reason}")]
    InvalidField {
        /// Field name.
        field: &'static str,
        /// Why the field is invalid.
        reason: String,
    },
    /// The requested integrator is unknown.
    #[error("unknown integrator `{0}`")]
    UnknownIntegrator(String),
    /// The requested force model is unknown or unsupported.
    #[error("unknown force model `{0}`")]
    UnknownForceModel(String),
}

/// Errors produced while writing run artifacts.
#[derive(Debug, Error)]
pub enum RecordingError {
    /// I/O failed while writing artifacts.
    #[error("artifact I/O error: {0}")]
    Io(#[from] std::io::Error),
    /// JSON serialization failed.
    #[error("artifact serialization error: {0}")]
    Serialize(#[from] serde_json::Error),
}

/// Errors produced by environment rollout.
#[derive(Debug, Error)]
pub enum EnvironmentError {
    /// Scenario construction failed.
    #[error(transparent)]
    Scenario(#[from] ScenarioError),
    /// Propagation failed.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Artifact recording failed.
    #[error(transparent)]
    Recording(#[from] RecordingError),
    /// Mission execution failed.
    #[error(transparent)]
    Mission(#[from] MissionError),
}

/// Errors produced while loading or executing mission scenarios.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum MissionError {
    /// Scenario construction failed.
    #[error(transparent)]
    Scenario(#[from] ScenarioError),
    /// Propagation failed.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// A mission field was invalid.
    #[error("invalid mission field `{field}`: {reason}")]
    InvalidField {
        /// Field name.
        field: &'static str,
        /// Why the field is invalid.
        reason: String,
    },
}
