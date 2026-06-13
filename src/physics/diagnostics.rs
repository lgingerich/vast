use nalgebra::Vector3;

use super::state::{CartesianState, CartesianStates};
use super::types::{Distance, Eccentricity, GravitationalParameter, Speed};
use crate::errors::NumericalError;

/// Specific orbital energy, measured in square meters per square second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpecificEnergy(f64);

impl SpecificEnergy {
    /// Creates a specific energy from square meters per square second.
    ///
    /// Returns an error if the value is not finite.
    pub fn from_square_meters_per_second_squared(value: f64) -> Result<Self, NumericalError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite {
                field: "specific energy",
            })
        }
    }

    /// Returns this specific energy in square meters per square second.
    #[inline]
    pub const fn square_meters_per_square_second(self) -> f64 {
        self.0
    }
}

/// Specific angular momentum, measured in square meters per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpecificAngularMomentum(f64);

impl SpecificAngularMomentum {
    /// Creates a specific angular momentum from square meters per second.
    ///
    /// Returns an error if the value is not finite or negative.
    pub fn from_square_meters_per_second(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "specific angular momentum",
            });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative {
                field: "specific angular momentum",
            });
        }
        Ok(Self(value))
    }

    /// Returns this specific angular momentum in square meters per second.
    #[inline]
    pub const fn square_meters_per_second(self) -> f64 {
        self.0
    }
}

/// Two-body orbital diagnostics for a Cartesian state.
pub struct OrbitalDiagnostics {
    /// Current radius from the central body's center of mass.
    radius: Distance,
    /// Current inertial speed.
    speed: Speed,
    /// Specific orbital energy.
    specific_energy: SpecificEnergy,
    /// Specific angular momentum vector.
    specific_angular_momentum_vector: Vector3<f64>,
    /// Specific angular momentum magnitude.
    specific_angular_momentum: SpecificAngularMomentum,
    /// Eccentricity vector.
    eccentricity_vector: Vector3<f64>,
    /// Eccentricity magnitude.
    eccentricity: Eccentricity,
    /// Semi-major axis for bound elliptical orbits.
    semi_major_axis: Option<Distance>,
    /// Periapsis radius for bound elliptical orbits.
    periapsis_radius: Option<Distance>,
    /// Apoapsis radius for bound elliptical orbits.
    apoapsis_radius: Option<Distance>,
}

impl OrbitalDiagnostics {
    /// Computes two-body diagnostics from an inertial Cartesian state.
    ///
    /// Returns an error if the position is at the origin or a computed
    /// quantity is non-finite.
    pub fn from_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
        Self::from_components(state.position.vector(), state.velocity.vector(), mu)
    }

    /// Computes two-body diagnostics from an indexed inertial Cartesian state view.
    pub fn from_states(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
    ) -> Result<Self, crate::errors::StateError> {
        let components = states.components(index)?;
        Ok(Self::from_components(
            components.position_vector(),
            components.velocity_vector(),
            mu,
        )?)
    }

    pub(crate) fn from_components(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
        let mu_f64 = mu.meters_cubed_per_second_squared();
        let radius = position.norm();
        let speed = velocity.norm();

        if radius == 0.0 {
            return Err(NumericalError::ZeroNorm { field: "position" });
        }

        let specific_energy = 0.5 * speed * speed - mu_f64 / radius;
        let specific_angular_momentum_vector = position.cross(&velocity);
        let specific_angular_momentum = specific_angular_momentum_vector.norm();
        let eccentricity_vector =
            velocity.cross(&specific_angular_momentum_vector) / mu_f64 - position / radius;
        let eccentricity = eccentricity_vector.norm();

        let semi_major_axis = bound_semi_major_axis(mu, specific_energy)?;
        let periapsis_radius = match semi_major_axis {
            Some(axis) => {
                let axis_m = axis.meters();
                Some(Distance::from_meters(axis_m * (1.0 - eccentricity))?)
            }
            None => None,
        };
        let apoapsis_radius = match semi_major_axis {
            Some(axis) => {
                let axis_m = axis.meters();
                Some(Distance::from_meters(axis_m * (1.0 + eccentricity))?)
            }
            None => None,
        };

        Ok(Self {
            radius: Distance::from_meters(radius)?,
            speed: Speed::from_meters_per_second(speed)?,
            specific_energy: SpecificEnergy::from_square_meters_per_second_squared(
                specific_energy,
            )?,
            specific_angular_momentum_vector,
            specific_angular_momentum: SpecificAngularMomentum::from_square_meters_per_second(
                specific_angular_momentum,
            )?,
            eccentricity_vector,
            eccentricity: Eccentricity::from_scalar(eccentricity)?,
            semi_major_axis,
            periapsis_radius,
            apoapsis_radius,
        })
    }

    /// Returns the current radius from the central body's center of mass.
    #[inline]
    pub const fn radius(&self) -> Distance {
        self.radius
    }

    /// Returns the current inertial speed.
    #[inline]
    pub const fn speed(&self) -> Speed {
        self.speed
    }

    /// Returns the specific orbital energy.
    #[inline]
    pub const fn specific_energy(&self) -> SpecificEnergy {
        self.specific_energy
    }

    /// Returns the specific angular momentum vector.
    #[inline]
    pub fn specific_angular_momentum_vector(&self) -> &Vector3<f64> {
        &self.specific_angular_momentum_vector
    }

    /// Returns the specific angular momentum magnitude.
    #[inline]
    pub const fn specific_angular_momentum(&self) -> SpecificAngularMomentum {
        self.specific_angular_momentum
    }

    /// Returns the eccentricity vector.
    #[inline]
    pub fn eccentricity_vector(&self) -> &Vector3<f64> {
        &self.eccentricity_vector
    }

    /// Returns the eccentricity magnitude.
    #[inline]
    pub const fn eccentricity(&self) -> Eccentricity {
        self.eccentricity
    }

    /// Returns the semi-major axis for bound elliptical orbits.
    #[inline]
    pub const fn semi_major_axis(&self) -> Option<Distance> {
        self.semi_major_axis
    }

    /// Returns the periapsis radius for bound elliptical orbits.
    #[inline]
    pub const fn periapsis_radius(&self) -> Option<Distance> {
        self.periapsis_radius
    }

    /// Returns the apoapsis radius for bound elliptical orbits.
    #[inline]
    pub const fn apoapsis_radius(&self) -> Option<Distance> {
        self.apoapsis_radius
    }
}

fn bound_semi_major_axis(
    mu: GravitationalParameter,
    specific_energy: f64,
) -> Result<Option<Distance>, NumericalError> {
    if specific_energy >= 0.0 {
        return Ok(None);
    }

    let mu_f64 = mu.meters_cubed_per_second_squared();
    Ok(Some(Distance::from_meters(
        -mu_f64 / (2.0 * specific_energy),
    )?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Angle, CartesianStateBuffer, Eccentricity, EllipticOrbit, MU_EARTH, OrbitDirection,
        Position, R_EARTH, Velocity,
    };

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const ALTITUDE_M: f64 = 400_000.0;
    const EPS: f64 = 1.0e-6;

    fn circular_orbit() -> Result<EllipticOrbit, Box<dyn std::error::Error>> {
        Ok(EllipticOrbit::new(
            Distance::from_meters(R_EARTH + ALTITUDE_M)?,
            Eccentricity::from_scalar(0.0)?,
            earth_mu()?,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0)?,
            OrbitDirection::Prograde,
        )?)
    }

    fn earth_mu() -> Result<GravitationalParameter, Box<dyn std::error::Error>> {
        Ok(GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?)
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn circular_orbit_diagnostics_match_two_body_invariants() -> TestResult {
        let orbit = circular_orbit()?;
        let state = orbit.to_cartesian_state()?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, orbit.mu())?;

        let radius = orbit.semi_major_axis().meters();
        let speed = orbit.speed()?.meters_per_second();

        assert_close(diagnostics.radius.meters(), radius, EPS);
        assert_close(diagnostics.speed.meters_per_second(), speed, EPS);
        assert_close(
            diagnostics
                .specific_energy
                .square_meters_per_square_second(),
            -MU_EARTH / (2.0 * radius),
            EPS,
        );
        assert_close(
            diagnostics
                .specific_angular_momentum
                .square_meters_per_second(),
            radius * speed,
            EPS,
        );
        assert_close(diagnostics.eccentricity.scalar(), 0.0, 1.0e-12);
        assert_close(
            diagnostics
                .semi_major_axis
                .ok_or("missing semi-major axis")?
                .meters(),
            radius,
            EPS,
        );
        assert_close(
            diagnostics
                .periapsis_radius
                .ok_or("missing periapsis radius")?
                .meters(),
            radius,
            EPS,
        );
        assert_close(
            diagnostics
                .apoapsis_radius
                .ok_or("missing apoapsis radius")?
                .meters(),
            radius,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn diagnostics_reject_zero_radius() -> TestResult {
        let state = CartesianState::new(
            Position::from_vector(Vector3::zeros())?,
            Velocity::from_vector(Vector3::zeros())?,
        );
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        assert!(OrbitalDiagnostics::from_states(&states.view(), 0, earth_mu()?).is_err());
        Ok(())
    }

    #[test]
    fn hyperbolic_state_has_no_bound_orbit_radii() -> TestResult {
        let radius = R_EARTH + ALTITUDE_M;
        let state = CartesianState::new(
            Position::from_vector(Vector3::new(radius, 0.0, 0.0))?,
            Velocity::from_vector(Vector3::new(0.0, 20_000.0, 0.0))?,
        );

        let diagnostics = OrbitalDiagnostics::from_state(&state, earth_mu()?)?;

        assert!(diagnostics.semi_major_axis.is_none());
        assert!(diagnostics.periapsis_radius.is_none());
        assert!(diagnostics.apoapsis_radius.is_none());
        Ok(())
    }

    #[test]
    fn diagnostics_reject_invalid_state_index() -> TestResult {
        let states = CartesianStateBuffer::<1>::with_len(1)?;

        assert!(OrbitalDiagnostics::from_states(&states.view(), 1, earth_mu()?).is_err());
        Ok(())
    }
}
