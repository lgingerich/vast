use nalgebra::Vector3;

use crate::state::CartesianState;
use crate::types::{Angle, Distance, Duration, GravitationalParameter, Position, Speed, Velocity};

/// Direction of motion around an orbital plane.
pub enum OrbitDirection {
    /// Motion follows the positive orientation of the selected orbital plane.
    Prograde,
    /// Motion follows the negative orientation of the selected orbital plane.
    Retrograde,
}

/// Two-dimensional orbital plane embedded in the inertial Cartesian frame.
pub enum OrbitPlane {
    /// Orbit lies in the x-y plane.
    Xy,
    /// Orbit lies in the x-z plane.
    Xz,
    /// Orbit lies in the y-z plane.
    Yz,
}

/// Circular two-body orbit around a central body.
pub struct CircularOrbit {
    /// Orbital radius from the central body's center of mass.
    pub radius: Distance,
    /// Standard gravitational parameter of the central body.
    pub mu: GravitationalParameter,
    /// Plane in which the circular orbit lies.
    pub plane: OrbitPlane,
    /// Angular position on the orbit measured in the selected orbital plane.
    pub phase: Angle,
    /// Direction of motion around the orbital plane.
    pub direction: OrbitDirection,
}

impl CircularOrbit {
    /// Circular orbital speed.
    pub fn speed(&self) -> Option<Speed> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let radius = self.radius.meters();

        debug_assert!(mu.is_finite());
        debug_assert!(mu > 0.0);
        debug_assert!(radius.is_finite());
        debug_assert!(radius > 0.0);

        Speed::from_meters_per_second((mu / radius).sqrt())
    }

    /// Circular orbital period.
    pub fn period(&self) -> Option<Duration> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let radius = self.radius.meters();

        debug_assert!(mu.is_finite());
        debug_assert!(mu > 0.0);
        debug_assert!(radius.is_finite());
        debug_assert!(radius > 0.0);

        Duration::from_seconds(2.0 * std::f64::consts::PI * (radius.powi(3) / mu).sqrt())
    }

    /// Converts this orbit into an inertial Cartesian state.
    pub fn to_cartesian_state(&self) -> Option<CartesianState> {
        let radius = self.radius.meters();
        let speed = self.speed()?.meters_per_second();
        let phase = self.phase.radians();

        debug_assert!(radius.is_finite());
        debug_assert!(radius > 0.0);
        debug_assert!(speed.is_finite());
        debug_assert!(speed >= 0.0);
        debug_assert!(phase.is_finite());

        let (sin_phase, cos_phase) = phase.sin_cos();

        let (position, prograde_velocity) = match self.plane {
            OrbitPlane::Xy => (
                Vector3::new(radius * cos_phase, radius * sin_phase, 0.0),
                Vector3::new(-speed * sin_phase, speed * cos_phase, 0.0),
            ),
            OrbitPlane::Xz => (
                Vector3::new(radius * cos_phase, 0.0, radius * sin_phase),
                Vector3::new(-speed * sin_phase, 0.0, speed * cos_phase),
            ),
            OrbitPlane::Yz => (
                Vector3::new(0.0, radius * cos_phase, radius * sin_phase),
                Vector3::new(0.0, -speed * sin_phase, speed * cos_phase),
            ),
        };

        let velocity = match self.direction {
            OrbitDirection::Prograde => prograde_velocity,
            OrbitDirection::Retrograde => -prograde_velocity,
        };

        debug_assert!(position.x.is_finite());
        debug_assert!(position.y.is_finite());
        debug_assert!(position.z.is_finite());
        debug_assert!(velocity.x.is_finite());
        debug_assert!(velocity.y.is_finite());
        debug_assert!(velocity.z.is_finite());

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
    use crate::constants::{MU_EARTH, R_EARTH};

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    fn circular_orbit_in_plane(
        phase: Angle,
        direction: OrbitDirection,
        plane: OrbitPlane,
    ) -> Option<CircularOrbit> {
        Some(CircularOrbit {
            radius: Distance::from_meters(R_EARTH + 400_000.0)?,
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            plane,
            phase,
            direction,
        })
    }

    fn circular_orbit(phase: Angle, direction: OrbitDirection) -> Option<CircularOrbit> {
        circular_orbit_in_plane(phase, direction, OrbitPlane::Xy)
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
    fn circular_orbit_converts_phase_zero_to_each_cartesian_plane() -> TestResult {
        for (plane, position_unit, velocity_unit) in [
            (
                OrbitPlane::Xy,
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 1.0, 0.0),
            ),
            (
                OrbitPlane::Xz,
                Vector3::new(1.0, 0.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
            (
                OrbitPlane::Yz,
                Vector3::new(0.0, 1.0, 0.0),
                Vector3::new(0.0, 0.0, 1.0),
            ),
        ] {
            let orbit = circular_orbit_in_plane(
                Angle::from_radians(0.0).ok_or("invalid angle")?,
                OrbitDirection::Prograde,
                plane,
            )
            .ok_or("invalid orbit")?;
            let state = orbit.to_cartesian_state().ok_or("invalid state")?;
            let speed = orbit
                .speed()
                .ok_or("invalid orbital speed")?
                .meters_per_second();

            assert_close(
                state.position.vector().x,
                orbit.radius.meters() * position_unit.x,
                EPS,
            );
            assert_close(
                state.position.vector().y,
                orbit.radius.meters() * position_unit.y,
                EPS,
            );
            assert_close(
                state.position.vector().z,
                orbit.radius.meters() * position_unit.z,
                EPS,
            );
            assert_close(state.velocity.vector().x, speed * velocity_unit.x, EPS);
            assert_close(state.velocity.vector().y, speed * velocity_unit.y, EPS);
            assert_close(state.velocity.vector().z, speed * velocity_unit.z, EPS);
        }
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
        let orbit = circular_orbit(
            Angle::from_radians(1.25).ok_or("invalid angle")?,
            OrbitDirection::Prograde,
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
    fn distance_rejects_nonpositive_values() {
        assert!(Distance::from_meters(0.0).is_none());
        assert!(Distance::from_meters(-1.0).is_none());
    }

    #[test]
    fn angle_rejects_nan() {
        assert!(Angle::from_radians(f64::NAN).is_none());
    }
}
