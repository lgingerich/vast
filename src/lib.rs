//! Core physics types and stepping logic for the space simulator.

mod constants;
mod diagnostics;
mod dynamics;
mod errors;
mod integrators;
mod orbits;
mod state;
mod types;
pub use constants::{MU_EARTH, R_EARTH};
pub use diagnostics::{Eccentricity, OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy};
pub use dynamics::{Dynamics, TwoBodyGravity};
pub use errors::{DynamicsError, IntegrationError};
pub use integrators::{EulerIntegrator, Integrator, RungeKutta4Integrator};
pub use orbits::{CircularOrbit, Orbit, OrbitDirection, OrbitPlane};
pub use state::CartesianState;
pub use types::{
    Acceleration, Angle, Distance, Duration, Epoch, GravitationalParameter, Position, Speed,
    TimeStep, Velocity,
};
