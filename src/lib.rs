//! Core physics types and stepping logic for the space simulator.

mod constants;
mod dynamics;
mod errors;
mod integrators;
mod orbits;
mod state;
mod types;
pub use constants::{MU_EARTH, R_EARTH};
pub use dynamics::{Dynamics, TwoBodyGravity};
pub use errors::{DynamicsError, IntegrationError};
pub use integrators::{EulerIntegrator, Integrator};
pub use orbits::{CircularOrbit, Orbit, OrbitDirection, OrbitPlane};
pub use state::CartesianState;
pub use types::{
    Acceleration, Angle, Distance, Duration, Epoch, GravitationalParameter, Position, Speed,
    TimeStep, Velocity,
};
