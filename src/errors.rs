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

/// Errors produced while advancing a simulation state.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum IntegrationError {
    /// The dynamics model could not produce a valid acceleration.
    #[error(transparent)]
    Dynamics(#[from] DynamicsError),
    /// The integrated position was not finite.
    #[error("integrated position was not finite")]
    InvalidPosition,
    /// The integrated velocity was not finite.
    #[error("integrated velocity was not finite")]
    InvalidVelocity,
}
