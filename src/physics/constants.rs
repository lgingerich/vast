//! Physical constants used by the simulator.

/// Earth's standard gravitational parameter in cubic meters per second squared.
///
/// Value from the IERS 2010 Conventions (Technical Note 36), Table 1.1.
///
/// <https://www.iers.org/IERS/EN/Publications/TechnicalNotes/tn36.html>
pub const MU_EARTH: f64 = 3.986004418e14;

/// Earth's equatorial radius in meters.
///
/// WGS 84 reference ellipsoid semi-major axis.
///
/// <https://nsgreg.nga.mil/doc/view?i=4085>
pub const R_EARTH: f64 = 6378137.0;

/// Earth's second zonal harmonic coefficient, dimensionless.
///
/// EGM2008 geopotential model, fully normalized value.
///
/// <https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84#egm2008>
pub const J2_EARTH: f64 = 1.08262668e-3;

/// Default tolerance for classifying an orbit as circular, dimensionless.
///
/// Chosen an order of magnitude above the cancellation floor of the
/// eccentricity vector computation.  For a near-circular orbit the two terms
/// in `e = (v × (r × v)) / μ - r̂` largely cancel; with μ ≈ 1e14 this
/// subtraction loses ~6–7 decimal digits of precision even before double
/// rounding.  A threshold of 1e-6 therefore captures physically circular
/// orbits without being so wide that it masks real eccentricity.
pub const DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE: f64 = 1.0e-6;

/// Default tolerance for classifying an orbit as equatorial, in radians.
///
/// Chosen just above the f64 representation floor for the inclination angle.
/// The unit angular-momentum vector `ĥ = (r × v) / ‖r × v‖` is computed from
/// a ratio of similar-magnitude quantities, so `acos(ĥ_z)` retains
/// near-full precision without the cancellation loss that eccentricity
/// suffers.  At 1e-10 rad the node-line direction is still numerically
/// stable; tolerances larger than this would unnecessarily discard RAAN for
/// orbits that the arithmetic resolves.
pub const DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD: f64 = 1.0e-10;
