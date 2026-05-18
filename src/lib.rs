//! Core physics types and stepping logic for the space simulator.

mod constants;
mod diagnostics;
mod dynamics;
mod elements;
mod errors;
mod integrators;
mod orbits;
mod state;
mod types;
pub use constants::{J2_EARTH, MU_EARTH, R_EARTH};
pub use diagnostics::{Eccentricity, OrbitalDiagnostics, SpecificAngularMomentum, SpecificEnergy};
pub use dynamics::{Dynamics, OblateBodyGravity, TwoBodyGravity};
pub use elements::{
    CircularEquatorialElements, CircularInclinedElements, ClassicalOrbitalElements,
    EquatorialEllipticElements, J2SecularRates, OrbitalElementClassification, OrbitalElements,
    j2_circular_raan_rate, j2_circular_secular_rates,
};
pub use errors::{DynamicsError, IntegrationError, StateError};
pub use integrators::{
    EulerIntegrator, Integrator, LeapfrogIntegrator, RungeKutta4Integrator,
    VelocityVerletIntegrator,
};
pub use orbits::{CircularOrbit, CircularOrbitOrientation, Orbit, OrbitDirection};
pub use state::{
    CartesianAccelerationComponents, CartesianPositionComponents, CartesianState,
    CartesianStateBuffer, CartesianStateComponents, CartesianStates, CartesianStatesMut,
    CartesianVelocityComponents,
};
pub use types::{
    Acceleration, Angle, AngularRate, Distance, Duration, Epoch, GravitationalParameter, Position,
    Speed, TimeStep, Velocity,
};
