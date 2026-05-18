use thiserror::Error;

/// Errors produced while evaluating a dynamics model.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum DynamicsError {
    /// Gravity is undefined at the central body's center of mass.
    #[error("two-body gravity is undefined at zero radius")]
    PositionAtGravityCenter,
    /// The computed acceleration was not finite.
    #[error("computed acceleration was not finite")]
    InvalidAcceleration,
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
