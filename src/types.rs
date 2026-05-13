use nalgebra::Vector3;

/// Instant in simulation time, measured in seconds from the simulation epoch.
///
/// `Epoch` represents an absolute point on the simulation timeline. Use
/// [`Duration`] for elapsed time or integration step sizes.
#[derive(Debug, Clone, Copy)]
pub struct Epoch(pub f64);

/// Elapsed simulation time, measured in seconds.
///
/// `Duration` represents an interval between two [`Epoch`] values, such as an
/// integrator step size.
#[derive(Debug, Clone, Copy)]
pub struct Duration(pub f64);

/// Position vector in a Cartesian frame, measured in meters.
#[derive(Debug, Clone, Copy)]
pub struct Position(pub Vector3<f64>);

/// Velocity vector in a Cartesian frame, measured in meters per second.
#[derive(Debug, Clone, Copy)]
pub struct Velocity(pub Vector3<f64>);

/// Acceleration vector in a Cartesian frame, measured in meters per second squared.
#[derive(Debug, Clone, Copy)]
pub struct Acceleration(pub Vector3<f64>);
