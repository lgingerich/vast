//! Central body definitions for gravity, atmosphere, and diagnostics.

use super::constants::{J2_EARTH, R_EARTH};
use super::types::{AngularRate, Distance, GravitationalParameter, J2Coefficient, MassDensity};

/// Earth's angular rotation rate in radians per second.
///
/// WGS 84 value, equal to one sidereal day over 2π.
const EARTH_ANGULAR_RATE_RAD_S: f64 = 7.292_115_9e-5;

/// Reference altitude of the exponential atmosphere above the equatorial radius.
const EARTH_ATMOSPHERE_REFERENCE_ALTITUDE_M: f64 = 250_000.0;

/// Reference atmospheric density at the reference altitude, in kg/m³.
///
/// Low solar activity value at 250 km; suitable for loose-fidelity LEO drag.
const EARTH_ATMOSPHERE_REFERENCE_DENSITY_KG_M3: f64 = 1.0e-10;

/// Exponential atmosphere scale height in meters.
const EARTH_ATMOSPHERE_SCALE_HEIGHT_M: f64 = 50_000.0;

/// A central body owning the physical constants used by gravity, atmosphere,
/// and diagnostics.
///
/// `CentralBody` is the single source of truth for a body's gravitational
/// parameter, equatorial radius, rotation rate, J2 coefficient, and optional
/// atmosphere. Force models, orbit construction, termination conditions, and
/// observations read from a `CentralBody` instead of scattering Earth constants
/// through scenarios and call sites.
///
/// Use [`CentralBody::earth`] for Earth scenarios. Non-Earth bodies will be
/// added as named constructors when their scenarios become active.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CentralBody {
    /// Human-readable body name (e.g. `"earth"`).
    name: &'static str,
    /// Standard gravitational parameter in m³/s².
    mu: GravitationalParameter,
    /// Equatorial reference radius in meters.
    equatorial_radius: Distance,
    /// Body angular rotation rate about the inertial z axis in rad/s.
    rotation_rate: AngularRate,
    /// Second zonal harmonic coefficient, when the body is modeled as oblate.
    j2: Option<J2Coefficient>,
    /// Exponential atmosphere parameters, when the body has an atmosphere.
    atmosphere: Option<AtmosphereModel>,
}

impl CentralBody {
    /// Creates a central body from its physical constants.
    ///
    /// This constructor is infallible because callers supply already-validated
    /// domain types. Use named constructors such as [`CentralBody::earth`] for
    /// building bodies from raw physical constants.
    #[inline]
    #[must_use]
    pub const fn new(
        name: &'static str,
        mu: GravitationalParameter,
        equatorial_radius: Distance,
        rotation_rate: AngularRate,
        j2: Option<J2Coefficient>,
        atmosphere: Option<AtmosphereModel>,
    ) -> Self {
        Self {
            name,
            mu,
            equatorial_radius,
            rotation_rate,
            j2,
            atmosphere,
        }
    }

    /// Earth central body using WGS 84 / EGM2008 constants and a loose-fidelity
    /// exponential atmosphere suitable for LEO drag scenarios.
    ///
    /// Returns [`crate::errors::NumericalError`] only if a compile-time Earth constant is
    /// invalid; in practice this cannot happen with the shipped constants.
    pub fn earth() -> Result<Self, crate::errors::NumericalError> {
        Ok(Self::new(
            "earth",
            GravitationalParameter::earth(),
            Distance::new(R_EARTH)?,
            AngularRate::new(EARTH_ANGULAR_RATE_RAD_S)?,
            Some(J2Coefficient::new(J2_EARTH)?),
            Some(AtmosphereModel::new(
                Distance::new(EARTH_ATMOSPHERE_REFERENCE_ALTITUDE_M)?,
                MassDensity::new(EARTH_ATMOSPHERE_REFERENCE_DENSITY_KG_M3)?,
                Distance::new(EARTH_ATMOSPHERE_SCALE_HEIGHT_M)?,
            )),
        ))
    }

    /// Returns the body name.
    #[inline]
    #[must_use]
    pub const fn name(&self) -> &'static str {
        self.name
    }

    /// Returns the standard gravitational parameter.
    #[inline]
    #[must_use]
    pub const fn mu(&self) -> GravitationalParameter {
        self.mu
    }

    /// Returns the equatorial reference radius.
    #[inline]
    #[must_use]
    pub const fn equatorial_radius(&self) -> Distance {
        self.equatorial_radius
    }

    /// Returns the body angular rotation rate about the inertial z axis.
    #[inline]
    #[must_use]
    pub const fn rotation_rate(&self) -> AngularRate {
        self.rotation_rate
    }

    /// Returns the J2 coefficient when the body is modeled as oblate.
    #[inline]
    #[must_use]
    pub const fn j2(&self) -> Option<J2Coefficient> {
        self.j2
    }

    /// Returns the exponential atmosphere parameters when present.
    #[inline]
    #[must_use]
    pub const fn atmosphere(&self) -> Option<AtmosphereModel> {
        self.atmosphere
    }
}

/// Parameters of a spherical exponential atmosphere attached to a central body.
///
/// Density at radius `r` (meters) is
/// `reference_density * exp(-(altitude - reference_altitude) / scale_height)`,
/// where `altitude = r - body_equatorial_radius`. The atmosphere co-rotates
/// with the body at [`CentralBody::rotation_rate`].
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AtmosphereModel {
    /// Reference altitude above the equatorial radius in meters.
    reference_altitude: Distance,
    /// Atmospheric density at the reference altitude in kg/m³.
    reference_density: MassDensity,
    /// Exponential scale height in meters.
    scale_height: Distance,
}

impl AtmosphereModel {
    /// Creates an exponential atmosphere parameter set.
    #[inline]
    #[must_use]
    pub const fn new(
        reference_altitude: Distance,
        reference_density: MassDensity,
        scale_height: Distance,
    ) -> Self {
        Self {
            reference_altitude,
            reference_density,
            scale_height,
        }
    }

    /// Returns the reference altitude above the equatorial radius.
    #[inline]
    #[must_use]
    pub const fn reference_altitude(&self) -> Distance {
        self.reference_altitude
    }

    /// Returns the atmospheric density at the reference altitude.
    #[inline]
    #[must_use]
    pub const fn reference_density(&self) -> MassDensity {
        self.reference_density
    }

    /// Returns the exponential scale height.
    #[inline]
    #[must_use]
    pub const fn scale_height(&self) -> Distance {
        self.scale_height
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn earth_body_carries_expected_constants() {
        let earth = CentralBody::earth().expect("Earth constants valid");
        assert_eq!(earth.name(), "earth");
        assert_eq!(earth.mu(), GravitationalParameter::earth());
        assert_eq!(earth.equatorial_radius().meters(), R_EARTH);
        assert!(earth.j2().is_some());
        assert_eq!(earth.j2().map(J2Coefficient::scalar), Some(J2_EARTH));
        assert!(earth.atmosphere().is_some());
    }

    #[test]
    fn custom_body_without_atmosphere_or_j2() {
        let body = CentralBody::new(
            "point",
            GravitationalParameter::earth(),
            Distance::new(R_EARTH).expect("R_EARTH valid"),
            AngularRate::new(0.0).expect("zero angular rate valid"),
            None,
            None,
        );
        assert_eq!(body.name(), "point");
        assert!(body.j2().is_none());
        assert!(body.atmosphere().is_none());
    }
}
