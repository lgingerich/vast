use thiserror::Error;

/// Error returned when a numerical value violates physical constraints.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum NumericalError {
    /// The value is NaN, +infinity, or -infinity.
    #[error("{field} must be finite")]
    NotFinite { field: &'static str },
    /// The value is zero or negative but must be strictly positive.
    #[error("{field} must be positive")]
    NonPositive { field: &'static str },
    /// The value is negative but must be non-negative.
    #[error("{field} must be non-negative")]
    Negative { field: &'static str },
    /// A vector quantity has zero norm when a nonzero vector is required.
    #[error("{field} has zero norm")]
    ZeroNorm { field: &'static str },
    /// A domain-specific physical constraint was violated.
    #[error("{field}: {reason}")]
    ConstraintViolation {
        /// The field or quantity name.
        field: &'static str,
        /// Why the constraint is required.
        reason: &'static str,
    },
}

/// Errors produced while evaluating a dynamics model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum DynamicsError {
    /// A numerical check failed.
    #[error(transparent)]
    Numerical(#[from] NumericalError),
    /// Gravity is undefined at the central body's center of mass.
    #[error("two-body gravity is undefined at zero radius")]
    PositionAtGravityCenter,
}

/// Errors produced while reading or writing Cartesian state storage.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum StateError {
    /// A numerical check failed.
    #[error(transparent)]
    Numerical(#[from] NumericalError),
    /// A requested body index was outside the active state range.
    #[error("body index is outside the active state range")]
    InvalidBodyIndex,
    /// A requested active length exceeded the storage capacity.
    #[error("active state length exceeds storage capacity")]
    InvalidLength,
    /// The frame is invalid.
    #[error("frame is invalid")]
    InvalidFrame,
    /// The frame axes are invalid.
    #[error("frame axes are invalid")]
    InvalidAxes,
}

/// Errors produced during propagation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PropagationError {
    /// The dynamics model could not produce a valid acceleration.
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
    /// The state storage could not be read or written.
    #[error(transparent)]
    State(#[from] StateError),
    /// A numerical check failed.
    #[error(transparent)]
    Numerical(#[from] NumericalError),
    /// An impulsive burn could not be applied to the current state.
    #[error("impulsive burn is invalid for the current state")]
    InvalidBurn,
    /// The simulation builder was used before all required fields were set.
    #[error("simulation builder is incomplete")]
    BuildIncomplete,
    /// A propagation segment ended without satisfying its termination condition.
    #[error("propagation segment ended without a stop reason")]
    SegmentWithoutStop,
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
    /// Scenario or mission construction failed.
    #[error(transparent)]
    Mission(#[from] MissionError),
    /// Propagation failed.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// Artifact recording failed.
    #[error(transparent)]
    Recording(#[from] RecordingError),
}

/// Errors produced while loading or executing scenarios and missions.
#[derive(Debug, Error)]
pub enum MissionError {
    /// The scenario file could not be read.
    #[error("failed to read scenario file")]
    ReadFile(#[from] std::io::Error),
    /// The scenario file could not be parsed.
    #[error("failed to parse scenario file")]
    ParseFile(#[from] toml::de::Error),
    /// A field was invalid.
    #[error("invalid field `{field}`: {reason}")]
    InvalidField {
        /// Field name.
        field: &'static str,
        /// Why the field is invalid.
        reason: String,
    },
    /// Propagation failed.
    #[error(transparent)]
    Propagation(#[from] PropagationError),
    /// A numerical field was physically invalid.
    #[error(transparent)]
    Numerical(#[from] NumericalError),
}
