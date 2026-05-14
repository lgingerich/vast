pub use crate::types::{Position, Velocity};

/// Position and velocity of a body in an inertial Cartesian frame.
pub struct CartesianState {
    /// Position in meters (SI), Cartesian frame.
    pub position: Position,
    /// Velocity in meters per second (SI), Cartesian frame.
    pub velocity: Velocity,
}

impl CartesianState {
    /// Creates a Cartesian state from typed position and velocity.
    #[inline]
    pub const fn new(position: Position, velocity: Velocity) -> Self {
        Self { position, velocity }
    }
}
