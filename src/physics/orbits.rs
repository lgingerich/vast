use super::elements::{EllipticElementSet, elliptic_elements_to_cartesian_state};
use super::state::CartesianState;
use super::types::{Angle, Distance, Duration, Eccentricity, GravitationalParameter, Speed};
use crate::errors::NumericalError;

/// Direction of motion around an orbital plane.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OrbitDirection {
    /// Motion follows the positive orientation of the orbital plane.
    Prograde,
    /// Motion follows the negative orientation of the orbital plane.
    Retrograde,
}

/// Keplerian (classical) two-body orbit around a central body.
///
/// An orbit with eccentricity in `[0, 1)`. When `eccentricity = 0` the
/// orbit is circular; set `argument_of_periapsis = 0` in that case (the
/// perifocal-to-inertial rotation is defined by convention and the
/// physics is unaffected).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EllipticOrbit {
    /// Semi-major axis of the elliptical orbit.
    semi_major_axis: Distance,
    /// Orbital eccentricity (0 ≤ e < 1).
    eccentricity: Eccentricity,
    /// Standard gravitational parameter of the central body.
    mu: GravitationalParameter,
    /// Inclination from the inertial x-y plane.
    inclination: Angle,
    /// Right ascension of the ascending node in the inertial x-y plane.
    right_ascension_of_ascending_node: Angle,
    /// Argument of periapsis measured from the ascending node.
    argument_of_periapsis: Angle,
    /// True anomaly at epoch.
    true_anomaly: Angle,
    /// Direction of motion around the orbital plane.
    direction: OrbitDirection,
}

impl EllipticOrbit {
    /// Creates a validated elliptic orbit.
    ///
    /// Eccentricity must be in `[0, 1)`. Parabolic and hyperbolic orbits are
    /// intentionally not represented by this type.
    #[expect(
        clippy::too_many_arguments,
        reason = "orbital element construction is clearer with explicit domain fields"
    )]
    pub fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        mu: GravitationalParameter,
        inclination: Angle,
        right_ascension_of_ascending_node: Angle,
        argument_of_periapsis: Angle,
        true_anomaly: Angle,
        direction: OrbitDirection,
    ) -> Result<Self, NumericalError> {
        if eccentricity.scalar() >= 1.0 {
            return Err(NumericalError::ConstraintViolation {
                field: "eccentricity",
                reason: "must be less than 1 for an elliptic orbit",
            });
        }

        Ok(Self {
            semi_major_axis,
            eccentricity,
            mu,
            inclination,
            right_ascension_of_ascending_node,
            argument_of_periapsis,
            true_anomaly,
            direction,
        })
    }

    /// Returns the semi-major axis.
    #[inline]
    pub const fn semi_major_axis(&self) -> Distance {
        self.semi_major_axis
    }

    /// Returns the orbital eccentricity.
    #[inline]
    pub const fn eccentricity(&self) -> Eccentricity {
        self.eccentricity
    }

    /// Returns the standard gravitational parameter.
    #[inline]
    pub const fn mu(&self) -> GravitationalParameter {
        self.mu
    }

    /// Returns the inclination.
    #[inline]
    pub const fn inclination(&self) -> Angle {
        self.inclination
    }

    /// Returns the right ascension of the ascending node.
    #[inline]
    pub const fn right_ascension_of_ascending_node(&self) -> Angle {
        self.right_ascension_of_ascending_node
    }

    /// Returns the argument of periapsis.
    #[inline]
    pub const fn argument_of_periapsis(&self) -> Angle {
        self.argument_of_periapsis
    }

    /// Returns the true anomaly.
    #[inline]
    pub const fn true_anomaly(&self) -> Angle {
        self.true_anomaly
    }

    /// Returns the direction of motion.
    #[inline]
    pub const fn direction(&self) -> OrbitDirection {
        self.direction
    }

    /// Instantaneous orbital speed at the current true anomaly.
    ///
    /// Returns an error if the computed speed is non-finite or negative.
    pub fn speed(&self) -> Result<Speed, NumericalError> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();
        let eccentricity = self.eccentricity.scalar();
        let true_anomaly = self.true_anomaly.radians();

        let radius = semi_major_axis * (1.0 - eccentricity * eccentricity)
            / (1.0 + eccentricity * true_anomaly.cos());

        Speed::from_meters_per_second((mu * (2.0 / radius - 1.0 / semi_major_axis)).sqrt())
    }

    /// Orbital period from Kepler's third law.
    ///
    /// Returns an error if the computed period is non-finite or non-positive.
    pub fn period(&self) -> Result<Duration, NumericalError> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();

        Duration::from_seconds(2.0 * std::f64::consts::PI * (semi_major_axis.powi(3) / mu).sqrt())
    }

    /// Converts this orbit into an inertial Cartesian state.
    ///
    /// Returns an error if the orbital parameters are physically invalid
    /// (e.g. parabolic or hyperbolic eccentricity) or a computed quantity
    /// is non-finite.
    pub fn to_cartesian_state(&self) -> Result<CartesianState, NumericalError> {
        let velocity_sign = match self.direction {
            OrbitDirection::Prograde => 1.0,
            OrbitDirection::Retrograde => -1.0,
        };

        elliptic_elements_to_cartesian_state(EllipticElementSet {
            semi_major_axis: self.semi_major_axis,
            eccentricity: self.eccentricity,
            mu: self.mu,
            inclination: self.inclination,
            right_ascension_of_ascending_node: self.right_ascension_of_ascending_node,
            argument_of_periapsis: self.argument_of_periapsis,
            true_anomaly: self.true_anomaly,
            velocity_sign,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    // ── EllipticOrbit tests ──────────────────────────────────────

    fn elliptic_orbit(
        eccentricity: Eccentricity,
        true_anomaly: Angle,
        direction: OrbitDirection,
    ) -> Result<EllipticOrbit, Box<dyn std::error::Error>> {
        Ok(EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 1_000_000.0)?,
            eccentricity,
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            Angle::from_radians(0.0)?,
            Angle::from_radians(0.0)?,
            Angle::from_radians(0.0)?,
            true_anomaly,
            direction,
        )?)
    }

    #[test]
    fn elliptic_orbit_speed_at_periapsis_matches_vis_viva() -> TestResult {
        let e = Eccentricity::from_scalar(0.3)?;
        let orbit = elliptic_orbit(e, Angle::from_radians(0.0)?, OrbitDirection::Prograde)?;
        let a = orbit.semi_major_axis().meters();
        let mu = MU_EARTH;
        let r_p = a * (1.0 - e.scalar());
        let expected = (mu * (2.0 / r_p - 1.0 / a)).sqrt();

        assert_close(orbit.speed()?.meters_per_second(), expected, EPS);
        Ok(())
    }

    #[test]
    fn elliptic_orbit_speed_at_apoapsis_matches_vis_viva() -> TestResult {
        let e = Eccentricity::from_scalar(0.3)?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(std::f64::consts::PI)?,
            OrbitDirection::Prograde,
        )?;
        let a = orbit.semi_major_axis().meters();
        let mu = MU_EARTH;
        let r_a = a * (1.0 + e.scalar());
        let expected = (mu * (2.0 / r_a - 1.0 / a)).sqrt();

        assert_close(orbit.speed()?.meters_per_second(), expected, EPS);
        Ok(())
    }

    #[test]
    fn elliptic_orbit_period_matches_kepler_third_law() -> TestResult {
        let e = Eccentricity::from_scalar(0.4)?;
        let orbit = elliptic_orbit(e, Angle::from_radians(1.0)?, OrbitDirection::Prograde)?;
        let expected = 2.0
            * std::f64::consts::PI
            * (orbit.semi_major_axis().meters().powi(3) / MU_EARTH).sqrt();

        assert_close(orbit.period()?.seconds(), expected, EPS);
        Ok(())
    }

    #[test]
    fn zero_eccentricity_orbit_produces_circular_cartesian_state() -> TestResult {
        let radius_m = R_EARTH + 400_000.0;
        let true_anomaly = Angle::from_radians(0.8)?;
        let orbit = EllipticOrbit::new(
            Distance::from_meters(radius_m)?,
            Eccentricity::from_scalar(0.0)?,
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            true_anomaly,
            OrbitDirection::Prograde,
        )?;

        let state = orbit.to_cartesian_state()?;
        let speed = (MU_EARTH / radius_m).sqrt();

        assert_close(state.position.vector().norm(), radius_m, EPS);
        assert_close(state.velocity.vector().norm(), speed, EPS);
        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_periapsis_velocity_is_perpendicular_to_position() -> TestResult {
        let e = Eccentricity::from_scalar(0.5)?;
        let orbit = elliptic_orbit(e, Angle::from_radians(0.0)?, OrbitDirection::Prograde)?;
        let state = orbit.to_cartesian_state()?;

        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_retrograde_reverses_velocity() -> TestResult {
        let e = Eccentricity::from_scalar(0.3)?;
        let prograde = elliptic_orbit(e, Angle::from_radians(0.7)?, OrbitDirection::Prograde)?
            .to_cartesian_state()?;
        let retrograde = elliptic_orbit(e, Angle::from_radians(0.7)?, OrbitDirection::Retrograde)?
            .to_cartesian_state()?;

        assert_close(
            retrograde.velocity.vector().x,
            -prograde.velocity.vector().x,
            EPS,
        );
        assert_close(
            retrograde.velocity.vector().y,
            -prograde.velocity.vector().y,
            EPS,
        );
        assert_close(
            retrograde.velocity.vector().z,
            -prograde.velocity.vector().z,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_rejects_eccentricity_at_or_above_one() -> TestResult {
        let e = Eccentricity::from_scalar(1.0)?;

        assert!(elliptic_orbit(e, Angle::from_radians(0.0)?, OrbitDirection::Prograde).is_err());
        Ok(())
    }

    #[test]
    fn elliptic_orbit_rejects_parabolic_eccentricity() -> TestResult {
        let e = Eccentricity::from_scalar(2.0)?;

        assert!(elliptic_orbit(e, Angle::from_radians(0.0)?, OrbitDirection::Prograde).is_err());
        Ok(())
    }

    #[test]
    fn elliptic_orbit_nonzero_inclination_raan_and_argp() -> TestResult {
        let e = Eccentricity::from_scalar(0.2)?;
        let orbit = EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 1_000_000.0)?,
            e,
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            Angle::from_radians(0.6)?,
            Angle::from_radians(0.9)?,
            Angle::from_radians(1.2)?,
            Angle::from_radians(0.4)?,
            OrbitDirection::Prograde,
        )?;
        let state = orbit.to_cartesian_state()?;

        assert_close(
            state.position.vector().norm(),
            orbit.semi_major_axis().meters() * (1.0 - e.scalar() * e.scalar())
                / (1.0 + e.scalar() * orbit.true_anomaly().radians().cos()),
            EPS,
        );
        assert!(
            state.position.vector().z.abs() > 0.0,
            "expected nonzero z for inclined orbit"
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_to_cartesian_state_has_correct_radius() -> TestResult {
        let e = Eccentricity::from_scalar(0.2)?;
        let elliptic = elliptic_orbit(e, Angle::from_radians(0.7)?, OrbitDirection::Prograde)?;

        let state = elliptic.to_cartesian_state()?;

        let radius = elliptic.semi_major_axis().meters() * (1.0 - e.scalar() * e.scalar())
            / (1.0 + e.scalar() * elliptic.true_anomaly().radians().cos());
        assert_close(state.position.vector().norm(), radius, EPS);
        Ok(())
    }
}
