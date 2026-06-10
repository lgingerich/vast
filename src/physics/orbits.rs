use nalgebra::Vector3;

use super::state::CartesianState;
use super::types::{Angle, Distance, Duration, GravitationalParameter, Position, Speed, Velocity};

/// Direction of motion around an orbital plane.
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

    fn basis(self) -> (Vector3<f64>, Vector3<f64>, Vector3<f64>) {
        let inclination = self.inclination.radians();
        let raan = self.right_ascension_of_ascending_node.radians();
        let (sin_inclination, cos_inclination) = inclination.sin_cos();
        let (sin_raan, cos_raan) = raan.sin_cos();

        let p_hat = Vector3::new(cos_raan, sin_raan, 0.0);
        let h_hat = Vector3::new(
            sin_inclination * sin_raan,
            -sin_inclination * cos_raan,
            cos_inclination,
        );
        let q_hat = h_hat.cross(&p_hat);

        // The basis vectors are derived through trigonometry and cross
        // products rather than constructed from domain-typed inputs.
        // Verify orthonormality here because no domain type enforces
        // this invariant downstream.
        debug_assert!((p_hat.norm() - 1.0).abs() <= 1.0e-12);
        debug_assert!((q_hat.norm() - 1.0).abs() <= 1.0e-12);
        debug_assert!((h_hat.norm() - 1.0).abs() <= 1.0e-12);

        (p_hat, q_hat, h_hat)
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
        let mu = self.mu.meters_cubed_per_second_squared();
        let radius = self.radius.meters();

        Speed::from_meters_per_second((mu / radius).sqrt())
    }

    /// Circular orbital period.
    pub fn period(&self) -> Option<Duration> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let radius = self.radius.meters();

        Duration::from_seconds(2.0 * std::f64::consts::PI * (radius.powi(3) / mu).sqrt())
    }

    /// Converts this orbit into an inertial Cartesian state.
    pub fn to_cartesian_state(&self) -> Option<CartesianState> {
        let radius = self.radius.meters();
        let speed = self.speed()?.meters_per_second();
        let phase = self.phase.radians();

        let (sin_phase, cos_phase) = phase.sin_cos();

        let (p_hat, q_hat, _h_hat) = self.orientation.basis();
        let position = radius * (cos_phase * p_hat + sin_phase * q_hat);
        let prograde_velocity = speed * (-sin_phase * p_hat + cos_phase * q_hat);

        let velocity = match self.direction {
            OrbitDirection::Prograde => prograde_velocity,
            OrbitDirection::Retrograde => -prograde_velocity,
        };

        Some(CartesianState::new(
            Position::from_vector(position)?,
            Velocity::from_vector(velocity)?,
        ))
    }
}

/// Closed set of orbit representations supported by the simulator.
pub enum Orbit {
    /// Circular two-body orbit.
    Circular(CircularOrbit),
}

impl Orbit {
    /// Converts this orbit into an inertial Cartesian state.
    pub fn to_cartesian_state(&self) -> Option<CartesianState> {
        match self {
            Orbit::Circular(orbit) => orbit.to_cartesian_state(),
        }
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
        let (_p_hat, _q_hat, h_hat) = orientation.basis();
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
