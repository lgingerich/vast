use nalgebra::Vector3;

use super::elements::perifocal_to_inertial_rotation;
use super::state::CartesianState;
use super::types::{
    Angle, Distance, Duration, Eccentricity, GravitationalParameter, Position, Speed, Velocity,
};

/// Direction of motion around an orbital plane.
#[derive(Clone, Copy)]
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
#[derive(Clone, Copy)]
pub struct EllipticOrbit {
    /// Semi-major axis of the elliptical orbit.
    pub semi_major_axis: Distance,
    /// Orbital eccentricity (0 ≤ e < 1).
    pub eccentricity: Eccentricity,
    /// Standard gravitational parameter of the central body.
    pub mu: GravitationalParameter,
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// Right ascension of the ascending node in the inertial x-y plane.
    pub right_ascension_of_ascending_node: Angle,
    /// Argument of periapsis measured from the ascending node.
    pub argument_of_periapsis: Angle,
    /// True anomaly at epoch.
    pub true_anomaly: Angle,
    /// Direction of motion around the orbital plane.
    pub direction: OrbitDirection,
}

impl EllipticOrbit {
    /// Instantaneous orbital speed at the current true anomaly.
    pub fn speed(&self) -> Option<Speed> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();
        let eccentricity = self.eccentricity.scalar();
        let true_anomaly = self.true_anomaly.radians();

        let radius = semi_major_axis * (1.0 - eccentricity * eccentricity)
            / (1.0 + eccentricity * true_anomaly.cos());

        Speed::from_meters_per_second((mu * (2.0 / radius - 1.0 / semi_major_axis)).sqrt())
    }

    /// Orbital period from Kepler's third law.
    pub fn period(&self) -> Option<Duration> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();

        Duration::from_seconds(2.0 * std::f64::consts::PI * (semi_major_axis.powi(3) / mu).sqrt())
    }

    /// Converts this orbit into an inertial Cartesian state.
    pub fn to_cartesian_state(self) -> Option<CartesianState> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();
        let eccentricity = self.eccentricity.scalar();
        let inclination = self.inclination.radians();
        let raan = self.right_ascension_of_ascending_node.radians();
        let argument_of_periapsis = self.argument_of_periapsis.radians();
        let true_anomaly = self.true_anomaly.radians();

        if !mu.is_finite() || mu <= 0.0 {
            return None;
        }
        if !semi_major_axis.is_finite() || semi_major_axis <= 0.0 {
            return None;
        }
        if !eccentricity.is_finite() || eccentricity >= 1.0 {
            return None;
        }

        let semi_latus_rectum = semi_major_axis * (1.0 - eccentricity * eccentricity);
        if semi_latus_rectum <= 0.0 {
            return None;
        }

        let cos_nu = true_anomaly.cos();
        let sin_nu = true_anomaly.sin();
        let radius = semi_latus_rectum / (1.0 + eccentricity * cos_nu);
        let position_pqw = Vector3::new(radius * cos_nu, radius * sin_nu, 0.0);
        let speed_factor = (mu / semi_latus_rectum).sqrt();
        let velocity_pqw = Vector3::new(
            -speed_factor * sin_nu,
            speed_factor * (eccentricity + cos_nu),
            0.0,
        );

        let rotation = perifocal_to_inertial_rotation(inclination, raan, argument_of_periapsis);
        let position = rotation * position_pqw;
        let velocity = match self.direction {
            OrbitDirection::Prograde => rotation * velocity_pqw,
            OrbitDirection::Retrograde => -(rotation * velocity_pqw),
        };

        Some(CartesianState::new(
            Position::from_vector(position)?,
            Velocity::from_vector(velocity)?,
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};

    type TestResult = Result<(), &'static str>;

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
    ) -> Option<EllipticOrbit> {
        Some(EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 1_000_000.0)?,
            eccentricity,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            inclination: Angle::from_radians(0.0)?,
            right_ascension_of_ascending_node: Angle::from_radians(0.0)?,
            argument_of_periapsis: Angle::from_radians(0.0)?,
            true_anomaly,
            direction,
        })
    }

    #[test]
    fn elliptic_orbit_speed_at_periapsis_matches_vis_viva() -> TestResult {
        let e = Eccentricity::from_scalar(0.3).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(0.0).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let a = orbit.semi_major_axis.meters();
        let mu = MU_EARTH;
        let r_p = a * (1.0 - e.scalar());
        let expected = (mu * (2.0 / r_p - 1.0 / a)).sqrt();

        assert_close(
            orbit.speed().ok_or("invalid speed")?.meters_per_second(),
            expected,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_speed_at_apoapsis_matches_vis_viva() -> TestResult {
        let e = Eccentricity::from_scalar(0.3).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(std::f64::consts::PI).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let a = orbit.semi_major_axis.meters();
        let mu = MU_EARTH;
        let r_a = a * (1.0 + e.scalar());
        let expected = (mu * (2.0 / r_a - 1.0 / a)).sqrt();

        assert_close(
            orbit.speed().ok_or("invalid speed")?.meters_per_second(),
            expected,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_period_matches_kepler_third_law() -> TestResult {
        let e = Eccentricity::from_scalar(0.4).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(1.0).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let expected =
            2.0 * std::f64::consts::PI * (orbit.semi_major_axis.meters().powi(3) / MU_EARTH).sqrt();

        assert_close(
            orbit.period().ok_or("invalid period")?.seconds(),
            expected,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn zero_eccentricity_orbit_produces_circular_cartesian_state() -> TestResult {
        let radius_m = R_EARTH + 400_000.0;
        let true_anomaly = Angle::from_radians(0.8).ok_or("invalid true anomaly")?;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(radius_m).ok_or("invalid semi-major axis")?,
            eccentricity: Eccentricity::from_scalar(0.0).ok_or("invalid eccentricity")?,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid mu")?,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly,
            direction: OrbitDirection::Prograde,
        };

        let state = orbit.to_cartesian_state().ok_or("invalid state")?;
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
        let e = Eccentricity::from_scalar(0.5).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(0.0).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;

        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_retrograde_reverses_velocity() -> TestResult {
        let e = Eccentricity::from_scalar(0.3).ok_or("invalid eccentricity")?;
        let prograde = elliptic_orbit(
            e,
            Angle::from_radians(0.7).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?
        .to_cartesian_state()
        .ok_or("invalid prograde state")?;
        let retrograde = elliptic_orbit(
            e,
            Angle::from_radians(0.7).ok_or("invalid anomaly")?,
            OrbitDirection::Retrograde,
        )
        .ok_or("invalid orbit")?
        .to_cartesian_state()
        .ok_or("invalid retrograde state")?;

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
        let e = Eccentricity::from_scalar(1.0).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(0.0).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;

        assert!(orbit.to_cartesian_state().is_none());
        Ok(())
    }

    #[test]
    fn elliptic_orbit_rejects_parabolic_eccentricity() -> TestResult {
        let e = Eccentricity::from_scalar(2.0).ok_or("invalid eccentricity")?;
        let orbit = elliptic_orbit(
            e,
            Angle::from_radians(0.0).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;

        assert!(orbit.to_cartesian_state().is_none());
        Ok(())
    }

    #[test]
    fn elliptic_orbit_nonzero_inclination_raan_and_argp() -> TestResult {
        let e = Eccentricity::from_scalar(0.2).ok_or("invalid eccentricity")?;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 1_000_000.0)
                .ok_or("invalid semi-major axis")?,
            eccentricity: e,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid mu")?,
            inclination: Angle::from_radians(0.6).ok_or("invalid inclination")?,
            right_ascension_of_ascending_node: Angle::from_radians(0.9).ok_or("invalid raan")?,
            argument_of_periapsis: Angle::from_radians(1.2).ok_or("invalid argp")?,
            true_anomaly: Angle::from_radians(0.4).ok_or("invalid true anomaly")?,
            direction: OrbitDirection::Prograde,
        };
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;

        assert_close(
            state.position.vector().norm(),
            orbit.semi_major_axis.meters() * (1.0 - e.scalar() * e.scalar())
                / (1.0 + e.scalar() * orbit.true_anomaly.radians().cos()),
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
        let e = Eccentricity::from_scalar(0.2).ok_or("invalid eccentricity")?;
        let elliptic = elliptic_orbit(
            e,
            Angle::from_radians(0.7).ok_or("invalid anomaly")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;

        let state = elliptic.to_cartesian_state().ok_or("invalid state")?;

        let radius = elliptic.semi_major_axis.meters() * (1.0 - e.scalar() * e.scalar())
            / (1.0 + e.scalar() * elliptic.true_anomaly.radians().cos());
        assert_close(state.position.vector().norm(), radius, EPS);
        Ok(())
    }

    #[test]
    fn distance_rejects_nonpositive_values() {
        assert!(Distance::from_meters(0.0).is_none());
        assert!(Distance::from_meters(-1.0).is_none());
    }

    #[test]
    fn angle_rejects_nan() {
        assert!(Angle::from_radians(f64::NAN).is_none());
    }
}
