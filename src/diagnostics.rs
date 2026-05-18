use nalgebra::Vector3;

use crate::state::{CartesianState, CartesianStates};
use crate::types::{Distance, GravitationalParameter, Speed};

/// Specific orbital energy, measured in square meters per square second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct SpecificEnergy(f64);

impl SpecificEnergy {
    /// Creates a specific energy from square meters per square second.
    pub fn from_square_meters_per_square_second(value: f64) -> Option<Self> {
        value.is_finite().then_some(Self(value))
    }

    /// Returns this specific energy in square meters per square second.
    #[inline]
    pub const fn square_meters_per_square_second(self) -> f64 {
        self.0
    }
}

/// Specific angular momentum, measured in square meters per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct SpecificAngularMomentum(f64);

impl SpecificAngularMomentum {
    /// Creates a specific angular momentum from square meters per second.
    pub fn from_square_meters_per_second(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
    }

    /// Returns this specific angular momentum in square meters per second.
    #[inline]
    pub const fn square_meters_per_second(self) -> f64 {
        self.0
    }
}

/// Orbital eccentricity, dimensionless.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct Eccentricity(f64);

impl Eccentricity {
    /// Creates an eccentricity from a dimensionless scalar.
    pub fn from_scalar(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
    }

    /// Returns this eccentricity as a dimensionless scalar.
    #[inline]
    pub const fn scalar(self) -> f64 {
        self.0
    }
}

/// Two-body orbital diagnostics for a Cartesian state.
pub struct OrbitalDiagnostics {
    /// Current radius from the central body's center of mass.
    pub radius: Distance,
    /// Current inertial speed.
    pub speed: Speed,
    /// Specific orbital energy.
    pub specific_energy: SpecificEnergy,
    /// Specific angular momentum vector.
    pub specific_angular_momentum_vector: Vector3<f64>,
    /// Specific angular momentum magnitude.
    pub specific_angular_momentum: SpecificAngularMomentum,
    /// Eccentricity vector.
    pub eccentricity_vector: Vector3<f64>,
    /// Eccentricity magnitude.
    pub eccentricity: Eccentricity,
    /// Semi-major axis for bound elliptical orbits.
    pub semi_major_axis: Option<Distance>,
    /// Periapsis radius for bound elliptical orbits.
    pub periapsis_radius: Option<Distance>,
    /// Apoapsis radius for bound elliptical orbits.
    pub apoapsis_radius: Option<Distance>,
}

impl OrbitalDiagnostics {
    /// Computes two-body diagnostics from an inertial Cartesian state.
    pub fn from_state(state: &CartesianState, mu: GravitationalParameter) -> Option<Self> {
        Self::from_components(state.position.vector(), state.velocity.vector(), mu)
    }

    /// Computes two-body diagnostics from an indexed inertial Cartesian state view.
    pub fn from_states(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
    ) -> Option<Self> {
        let components = states.components(index).ok()?;
        Self::from_components(
            components.position.vector(),
            components.velocity.vector(),
            mu,
        )
    }

    fn from_components(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        mu: GravitationalParameter,
    ) -> Option<Self> {
        let mu = mu.meters_cubed_per_second_squared();
        let radius = position.norm();
        let speed = velocity.norm();

        if radius == 0.0 {
            return None;
        }

        debug_assert!(position.x.is_finite());
        debug_assert!(position.y.is_finite());
        debug_assert!(position.z.is_finite());
        debug_assert!(velocity.x.is_finite());
        debug_assert!(velocity.y.is_finite());
        debug_assert!(velocity.z.is_finite());
        debug_assert!(mu.is_finite());
        debug_assert!(mu > 0.0);
        debug_assert!(radius.is_finite());
        debug_assert!(radius > 0.0);
        debug_assert!(speed.is_finite());
        debug_assert!(speed >= 0.0);

        let specific_energy = 0.5 * speed * speed - mu / radius;
        let specific_angular_momentum_vector = position.cross(&velocity);
        let specific_angular_momentum = specific_angular_momentum_vector.norm();
        let eccentricity_vector =
            velocity.cross(&specific_angular_momentum_vector) / mu - position / radius;
        let eccentricity = eccentricity_vector.norm();

        let semi_major_axis = bound_semi_major_axis(mu, specific_energy);
        let periapsis_radius = semi_major_axis
            .and_then(|axis| Distance::from_meters(axis.meters() * (1.0 - eccentricity)));
        let apoapsis_radius = semi_major_axis
            .and_then(|axis| Distance::from_meters(axis.meters() * (1.0 + eccentricity)));

        Some(Self {
            radius: Distance::from_meters(radius)?,
            speed: Speed::from_meters_per_second(speed)?,
            specific_energy: SpecificEnergy::from_square_meters_per_square_second(specific_energy)?,
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
}

fn bound_semi_major_axis(mu: f64, specific_energy: f64) -> Option<Distance> {
    if specific_energy >= 0.0 {
        return None;
    }

    Distance::from_meters(-mu / (2.0 * specific_energy))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, R_EARTH};
    use crate::orbits::{CircularOrbit, CircularOrbitOrientation, OrbitDirection};
    use crate::state::CartesianStateBuffer;
    use crate::types::{Angle, Position, Velocity};

    type TestResult = Result<(), &'static str>;

    const ALTITUDE_M: f64 = 400_000.0;
    const EPS: f64 = 1.0e-6;

    fn circular_orbit() -> Option<CircularOrbit> {
        Some(CircularOrbit {
            radius: Distance::from_meters(R_EARTH + ALTITUDE_M)?,
            mu: earth_mu()?,
            orientation: CircularOrbitOrientation::equatorial(),
            phase: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        })
    }

    fn earth_mu() -> Option<GravitationalParameter> {
        GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn circular_orbit_diagnostics_match_two_body_invariants() -> TestResult {
        let orbit = circular_orbit().ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, orbit.mu)
            .ok_or("invalid diagnostics")?;

        let radius = orbit.radius.meters();
        let speed = orbit
            .speed()
            .ok_or("invalid circular speed")?
            .meters_per_second();

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
            Position::from_vector(Vector3::zeros()).ok_or("invalid position")?,
            Velocity::from_vector(Vector3::zeros()).ok_or("invalid velocity")?,
        );
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        assert!(
            OrbitalDiagnostics::from_states(&states.view(), 0, earth_mu().ok_or("invalid mu")?)
                .is_none()
        );
        Ok(())
    }

    #[test]
    fn hyperbolic_state_has_no_bound_orbit_radii() -> TestResult {
        let radius = R_EARTH + ALTITUDE_M;
        let state = CartesianState::new(
            Position::from_vector(Vector3::new(radius, 0.0, 0.0)).ok_or("invalid position")?,
            Velocity::from_vector(Vector3::new(0.0, 20_000.0, 0.0)).ok_or("invalid velocity")?,
        );

        let diagnostics = OrbitalDiagnostics::from_state(&state, earth_mu().ok_or("invalid mu")?)
            .ok_or("invalid diagnostics")?;

        assert!(diagnostics.semi_major_axis.is_none());
        assert!(diagnostics.periapsis_radius.is_none());
        assert!(diagnostics.apoapsis_radius.is_none());
        Ok(())
    }

    #[test]
    fn diagnostics_reject_invalid_state_index() -> TestResult {
        let states = CartesianStateBuffer::<1>::new();

        assert!(
            OrbitalDiagnostics::from_states(&states.view(), 1, earth_mu().ok_or("invalid mu")?)
                .is_none()
        );
        Ok(())
    }
}
