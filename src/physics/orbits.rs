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

/// Orientation of a circular orbit in the inertial Cartesian frame.
///
/// `inclination` tilts the orbital plane away from the x-y plane. The right
/// ascension of the ascending node sets the in-plane reference direction from
/// which `CircularOrbit::phase` is measured.
#[derive(Clone, Copy)]
pub struct CircularOrbitOrientation {
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// Right ascension of the ascending node in the inertial x-y plane.
    pub right_ascension_of_ascending_node: Angle,
}

impl CircularOrbitOrientation {
    /// Creates an orbit orientation from inclination and RAAN.
    #[inline]
    pub const fn new(inclination: Angle, right_ascension_of_ascending_node: Angle) -> Self {
        Self {
            inclination,
            right_ascension_of_ascending_node,
        }
    }

    /// Equatorial orientation with zero inclination and zero RAAN.
    #[inline]
    pub const fn equatorial() -> Self {
        Self::new(Angle::ZERO, Angle::ZERO)
    }
}

/// Elliptic two-body orbit around a central body.
///
/// An orbit with eccentricity in `[0, 1)`. When `eccentricity = 0` the orbit
/// is circular; however callers who want an explicitly circular parameterization
/// should prefer [`CircularOrbit`], which avoids the undefined
/// `argument_of_periapsis` field.
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

/// Circular two-body orbit around a central body.
pub struct CircularOrbit {
    /// Orbital radius from the central body's center of mass.
    pub radius: Distance,
    /// Standard gravitational parameter of the central body.
    pub mu: GravitationalParameter,
    /// Orientation of the orbital plane in the inertial Cartesian frame.
    pub orientation: CircularOrbitOrientation,
    /// Angular position on the orbit measured from the ascending-node direction.
    pub phase: Angle,
    /// Direction of motion around the orbital plane.
    pub direction: OrbitDirection,
}

impl CircularOrbit {
    /// Circular orbital speed.
    pub fn speed(&self) -> Option<Speed> {
        self.as_elliptic()?.speed()
    }

    /// Circular orbital period.
    pub fn period(&self) -> Option<Duration> {
        self.as_elliptic()?.period()
    }

    /// Converts this orbit into an inertial Cartesian state.
    pub fn to_cartesian_state(&self) -> Option<CartesianState> {
        self.as_elliptic()?.to_cartesian_state()
    }

    /// Constructs the equivalent elliptic orbit with zero eccentricity and
    /// argument of periapsis aligned with the ascending node.
    fn as_elliptic(&self) -> Option<EllipticOrbit> {
        Some(EllipticOrbit {
            semi_major_axis: self.radius,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu: self.mu,
            inclination: self.orientation.inclination,
            right_ascension_of_ascending_node: self.orientation.right_ascension_of_ascending_node,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: self.phase,
            direction: self.direction,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    fn circular_orbit_with_orientation(
        phase: Angle,
        direction: OrbitDirection,
        orientation: CircularOrbitOrientation,
    ) -> Option<CircularOrbit> {
        Some(CircularOrbit {
            radius: Distance::from_meters(R_EARTH + 400_000.0)?,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            orientation,
            phase,
            direction,
        })
    }

    fn circular_orbit(phase: Angle, direction: OrbitDirection) -> Option<CircularOrbit> {
        circular_orbit_with_orientation(phase, direction, CircularOrbitOrientation::equatorial())
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn circular_orbit_speed_is_sqrt_mu_over_radius() -> TestResult {
        let orbit = circular_orbit(
            Angle::from_radians(0.0).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;

        assert_close(
            orbit
                .speed()
                .ok_or("invalid orbital speed")?
                .meters_per_second(),
            (orbit.mu.meters_cubed_per_second_squared() / orbit.radius.meters()).sqrt(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn circular_orbit_period_matches_two_body_period() -> TestResult {
        let orbit = circular_orbit(
            Angle::from_radians(0.0).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let expected = 2.0
            * std::f64::consts::PI
            * (orbit.radius.meters().powi(3) / orbit.mu.meters_cubed_per_second_squared()).sqrt();

        assert_close(
            orbit.period().ok_or("invalid orbital period")?.seconds(),
            expected,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn equatorial_orientation_matches_xy_geometry() -> TestResult {
        let orbit = circular_orbit(
            Angle::from_radians(0.0).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;
        let speed = orbit
            .speed()
            .ok_or("invalid orbital speed")?
            .meters_per_second();

        assert_close(state.position.vector().x, orbit.radius.meters(), EPS);
        assert_close(state.position.vector().y, 0.0, EPS);
        assert_close(state.position.vector().z, 0.0, EPS);
        assert_close(state.velocity.vector().x, 0.0, EPS);
        assert_close(state.velocity.vector().y, speed, EPS);
        assert_close(state.velocity.vector().z, 0.0, EPS);
        Ok(())
    }

    #[test]
    fn polar_orientation_with_zero_raan_matches_xz_geometry() -> TestResult {
        let orbit = circular_orbit_with_orientation(
            Angle::from_radians(0.0).ok_or("invalid phase")?,
            OrbitDirection::Prograde,
            CircularOrbitOrientation::new(
                Angle::from_radians(std::f64::consts::FRAC_PI_2).ok_or("invalid inclination")?,
                Angle::from_radians(0.0).ok_or("invalid raan")?,
            ),
        )
        .ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;
        let speed = orbit
            .speed()
            .ok_or("invalid orbital speed")?
            .meters_per_second();

        assert_close(state.position.vector().x, orbit.radius.meters(), EPS);
        assert_close(state.position.vector().y, 0.0, EPS);
        assert_close(state.position.vector().z, 0.0, EPS);
        assert_close(state.velocity.vector().x, 0.0, EPS);
        assert_close(state.velocity.vector().y, 0.0, EPS);
        assert_close(state.velocity.vector().z, speed, EPS);
        Ok(())
    }

    #[test]
    fn circular_orbit_retrograde_reverses_velocity() -> TestResult {
        let prograde = circular_orbit(
            Angle::from_radians(0.0).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
        )
        .ok_or("invalid orbit")?
        .to_cartesian_state()
        .ok_or("invalid prograde state")?;
        let retrograde = circular_orbit(
            Angle::from_radians(0.0).ok_or("invalid angle")?,
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
    fn circular_orbit_cartesian_state_preserves_radius_speed_and_perpendicularity() -> TestResult {
        let orbit = circular_orbit_with_orientation(
            Angle::from_radians(1.25).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
            CircularOrbitOrientation::new(
                Angle::from_radians(0.7).ok_or("invalid inclination")?,
                Angle::from_radians(1.1).ok_or("invalid raan")?,
            ),
        )
        .ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;

        assert_close(state.position.vector().norm(), orbit.radius.meters(), EPS);
        assert_close(
            state.velocity.vector().norm(),
            orbit
                .speed()
                .ok_or("invalid orbital speed")?
                .meters_per_second(),
            EPS,
        );
        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn angular_momentum_direction_matches_orientation() -> TestResult {
        let orientation = CircularOrbitOrientation::new(
            Angle::from_radians(0.8).ok_or("invalid inclination")?,
            Angle::from_radians(1.2).ok_or("invalid raan")?,
        );
        let inclination = orientation.inclination.radians();
        let raan = orientation.right_ascension_of_ascending_node.radians();
        let (sin_incl, cos_incl) = inclination.sin_cos();
        let (sin_raan, cos_raan) = raan.sin_cos();
        let h_hat = Vector3::new(sin_incl * sin_raan, -sin_incl * cos_raan, cos_incl);
        let prograde = circular_orbit_with_orientation(
            Angle::from_radians(0.4).ok_or("invalid phase")?,
            OrbitDirection::Prograde,
            orientation,
        )
        .ok_or("invalid prograde orbit")?
        .to_cartesian_state()
        .ok_or("invalid prograde state")?;
        let retrograde = circular_orbit_with_orientation(
            Angle::from_radians(0.4).ok_or("invalid phase")?,
            OrbitDirection::Retrograde,
            orientation,
        )
        .ok_or("invalid retrograde orbit")?
        .to_cartesian_state()
        .ok_or("invalid retrograde state")?;

        let prograde_h = prograde
            .position
            .vector()
            .cross(&prograde.velocity.vector())
            .normalize();
        let retrograde_h = retrograde
            .position
            .vector()
            .cross(&retrograde.velocity.vector())
            .normalize();

        assert_close(prograde_h.x, h_hat.x, EPS);
        assert_close(prograde_h.y, h_hat.y, EPS);
        assert_close(prograde_h.z, h_hat.z, EPS);
        assert_close(retrograde_h.x, -h_hat.x, EPS);
        assert_close(retrograde_h.y, -h_hat.y, EPS);
        assert_close(retrograde_h.z, -h_hat.z, EPS);
        Ok(())
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
    fn zero_eccentricity_elliptic_matches_circular_orbit() -> TestResult {
        let e = Eccentricity::from_scalar(0.0).ok_or("invalid eccentricity")?;
        let elliptic = EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 400_000.0)
                .ok_or("invalid semi-major axis")?,
            eccentricity: e,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid mu")?,
            inclination: Angle::from_radians(0.0).ok_or("invalid inclination")?,
            right_ascension_of_ascending_node: Angle::from_radians(0.0).ok_or("invalid raan")?,
            argument_of_periapsis: Angle::from_radians(0.0).ok_or("invalid argp")?,
            true_anomaly: Angle::from_radians(0.8).ok_or("invalid true anomaly")?,
            direction: OrbitDirection::Prograde,
        };

        let circular = circular_orbit(elliptic.true_anomaly, OrbitDirection::Prograde)
            .ok_or("invalid circular orbit")?;

        let elliptic_state = elliptic
            .to_cartesian_state()
            .ok_or("invalid elliptic state")?;
        let circular_state = circular
            .to_cartesian_state()
            .ok_or("invalid circular state")?;

        assert_close(
            elliptic_state.position.vector().x,
            circular_state.position.vector().x,
            EPS,
        );
        assert_close(
            elliptic_state.position.vector().y,
            circular_state.position.vector().y,
            EPS,
        );
        assert_close(
            elliptic_state.position.vector().z,
            circular_state.position.vector().z,
            EPS,
        );
        assert_close(
            elliptic_state.velocity.vector().x,
            circular_state.velocity.vector().x,
            EPS,
        );
        assert_close(
            elliptic_state.velocity.vector().y,
            circular_state.velocity.vector().y,
            EPS,
        );
        assert_close(
            elliptic_state.velocity.vector().z,
            circular_state.velocity.vector().z,
            EPS,
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
