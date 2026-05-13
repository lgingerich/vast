//! Core physics types and stepping logic for the space simulator.

mod constants;
mod types;

pub use constants::{MU_EARTH, R_EARTH};
pub use types::{Acceleration, Duration, Epoch, Position, Velocity};

/// Position and velocity of a body in an inertial Cartesian frame.
pub struct CartesianState {
    /// Position in meters (SI), Cartesian frame.
    pub position: Position,
    /// Velocity in meters per second (SI), Cartesian frame.
    pub velocity: Velocity,
}

/// Point-mass gravity model for a single central body.
pub struct TwoBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    pub mu: f64,
}

/// Semi-implicit Euler time integrator.
pub struct EulerIntegrator;

/// Computes acceleration for a state at a given simulation time.
pub trait Dynamics {
    /// Returns the acceleration applied to `state` at epoch `t`.
    fn acceleration(&self, state: &CartesianState, t: Epoch) -> Acceleration;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration(&self, state: &CartesianState, _t: Epoch) -> Acceleration {
        let r = state.position.0;
        let radius = r.norm();

        Acceleration(-self.mu * r / radius.powi(3))
    }
}

/// Advances a simulation state by one time step.
pub trait Integrator {
    /// Steps `state` forward by `dt` using the provided dynamics model.
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        state: &CartesianState,
        t: Epoch,
        dt: Duration,
    ) -> CartesianState;
}

impl Integrator for EulerIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        state: &CartesianState,
        t: Epoch,
        dt: Duration,
    ) -> CartesianState {
        let acceleration = dynamics.acceleration(state, t);
        let new_vel = state.velocity.0 + acceleration.0 * dt.0;
        let new_pos = state.position.0 + new_vel * dt.0;

        CartesianState {
            position: Position(new_pos),
            velocity: Velocity(new_vel),
        }
    }
}
