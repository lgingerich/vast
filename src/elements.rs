use nalgebra::Vector3;

use crate::diagnostics::Eccentricity;
use crate::state::{CartesianState, CartesianStates};
use crate::types::{Angle, Distance, GravitationalParameter};

const CIRCULAR_ECCENTRICITY_TOLERANCE: f64 = 1.0e-10;
const EQUATORIAL_NODE_TOLERANCE: f64 = 1.0e-10;
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Orbital elements for bound two-body Cartesian states.
///
/// Classical orbital elements have singularities for circular and equatorial
/// orbits, so this enum uses variants that expose only the angles that are
/// physically defined for the state.
#[derive(Debug, Clone, Copy)]
pub enum OrbitalElements {
    /// Non-circular, non-equatorial bound orbit with all classical angles defined.
    Classical(ClassicalOrbitalElements),
    /// Circular inclined orbit where argument of periapsis is undefined.
    CircularInclined(CircularInclinedElements),
    /// Circular equatorial orbit where RAAN and argument of periapsis are undefined.
    CircularEquatorial(CircularEquatorialElements),
    /// Eccentric equatorial orbit where RAAN is undefined.
    EquatorialElliptic(EquatorialEllipticElements),
}

/// Classical elements for a non-circular, non-equatorial bound orbit.
#[derive(Debug, Clone, Copy)]
pub struct ClassicalOrbitalElements {
    /// Semi-major axis for the bound orbit.
    pub semi_major_axis: Distance,
    /// Orbital eccentricity.
    pub eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// Right ascension of the ascending node.
    pub right_ascension_of_ascending_node: Angle,
    /// Argument of periapsis.
    pub argument_of_periapsis: Angle,
    /// True anomaly.
    pub true_anomaly: Angle,
}

/// Elements for a circular inclined bound orbit.
#[derive(Debug, Clone, Copy)]
pub struct CircularInclinedElements {
    /// Semi-major axis for the bound orbit.
    pub semi_major_axis: Distance,
    /// Orbital eccentricity.
    pub eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// Right ascension of the ascending node.
    pub right_ascension_of_ascending_node: Angle,
    /// Argument of latitude measured from the ascending node.
    pub argument_of_latitude: Angle,
}

/// Elements for a circular equatorial bound orbit.
#[derive(Debug, Clone, Copy)]
pub struct CircularEquatorialElements {
    /// Semi-major axis for the bound orbit.
    pub semi_major_axis: Distance,
    /// Orbital eccentricity.
    pub eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// True longitude measured in the inertial x-y plane.
    pub true_longitude: Angle,
}

/// Elements for an eccentric equatorial bound orbit.
#[derive(Debug, Clone, Copy)]
pub struct EquatorialEllipticElements {
    /// Semi-major axis for the bound orbit.
    pub semi_major_axis: Distance,
    /// Orbital eccentricity.
    pub eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    pub inclination: Angle,
    /// Longitude of periapsis measured in the inertial x-y plane.
    pub longitude_of_periapsis: Angle,
    /// True anomaly measured from periapsis.
    pub true_anomaly: Angle,
}

impl OrbitalElements {
    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    pub fn from_cartesian_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Option<Self> {
        Self::from_components(state.position.vector(), state.velocity.vector(), mu)
    }

    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    #[inline]
    pub fn from_state(state: &CartesianState, mu: GravitationalParameter) -> Option<Self> {
        Self::from_cartesian_state(state, mu)
    }

    /// Computes bound two-body orbital elements from an indexed Cartesian state view.
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
        let speed_sq = velocity.norm_squared();

        if radius == 0.0 {
            return None;
        }

        let h = position.cross(&velocity);
        let h_norm = h.norm();
        if h_norm == 0.0 {
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
        debug_assert!(h_norm.is_finite());
        debug_assert!(h_norm > 0.0);

        let specific_energy = 0.5 * speed_sq - mu / radius;
        if specific_energy >= 0.0 {
            return None;
        }

        let semi_major_axis = Distance::from_meters(-mu / (2.0 * specific_energy))?;
        let h_hat = h / h_norm;
        let node = Vector3::new(-h.y, h.x, 0.0);
        let node_norm = node.norm();
        let eccentricity_vector = velocity.cross(&h) / mu - position / radius;
        let eccentricity_scalar = eccentricity_vector.norm();
        let eccentricity = Eccentricity::from_scalar(eccentricity_scalar)?;
        let inclination = angle_from_radians((h.z / h_norm).clamp(-1.0, 1.0).acos())?;

        let is_circular = eccentricity_scalar <= CIRCULAR_ECCENTRICITY_TOLERANCE;
        let is_equatorial = node_norm <= EQUATORIAL_NODE_TOLERANCE;

        match (is_circular, is_equatorial) {
            (false, false) => Some(Self::Classical(ClassicalOrbitalElements {
                semi_major_axis,
                eccentricity,
                inclination,
                right_ascension_of_ascending_node: angle_from_radians(node.y.atan2(node.x))?,
                argument_of_periapsis: angle_between_in_plane(node, eccentricity_vector, h_hat)?,
                true_anomaly: angle_between_in_plane(eccentricity_vector, position, h_hat)?,
            })),
            (true, false) => Some(Self::CircularInclined(CircularInclinedElements {
                semi_major_axis,
                eccentricity,
                inclination,
                right_ascension_of_ascending_node: angle_from_radians(node.y.atan2(node.x))?,
                argument_of_latitude: angle_between_in_plane(node, position, h_hat)?,
            })),
            (true, true) => Some(Self::CircularEquatorial(CircularEquatorialElements {
                semi_major_axis,
                eccentricity,
                inclination,
                true_longitude: angle_from_radians(position.y.atan2(position.x))?,
            })),
            (false, true) => Some(Self::EquatorialElliptic(EquatorialEllipticElements {
                semi_major_axis,
                eccentricity,
                inclination,
                longitude_of_periapsis: angle_from_radians(
                    eccentricity_vector.y.atan2(eccentricity_vector.x),
                )?,
                true_anomaly: angle_between_in_plane(eccentricity_vector, position, h_hat)?,
            })),
        }
    }
}

fn angle_between_in_plane(
    from: Vector3<f64>,
    to: Vector3<f64>,
    plane_normal: Vector3<f64>,
) -> Option<Angle> {
    let x = from.dot(&to);
    let y = from.cross(&to).dot(&plane_normal);
    angle_from_radians(y.atan2(x))
}

fn angle_from_radians(radians: f64) -> Option<Angle> {
    Angle::from_radians(normalize_angle(radians))
}

fn normalize_angle(radians: f64) -> f64 {
    let normalized = radians.rem_euclid(TWO_PI);
    if normalized == TWO_PI {
        0.0
    } else {
        normalized
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, R_EARTH};
    use crate::orbits::{CircularOrbit, CircularOrbitOrientation, OrbitDirection};
    use crate::types::{Position, Velocity};

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    fn earth_mu() -> Option<GravitationalParameter> {
        GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
    }

    fn circular_orbit(orientation: CircularOrbitOrientation) -> Option<CircularOrbit> {
        Some(CircularOrbit {
            radius: Distance::from_meters(R_EARTH + 600_000.0)?,
            mu: earth_mu()?,
            orientation,
            phase: Angle::from_radians(0.4)?,
            direction: OrbitDirection::Prograde,
        })
    }

    fn state_at(position: Vector3<f64>, velocity: Vector3<f64>) -> Option<CartesianState> {
        Some(CartesianState::new(
            Position::from_vector(position)?,
            Velocity::from_vector(velocity)?,
        ))
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn inclined_circular_state_maps_to_circular_inclined_elements() -> TestResult {
        let inclination = Angle::from_radians(0.9).ok_or("invalid inclination")?;
        let raan = Angle::from_radians(1.1).ok_or("invalid raan")?;
        let orbit = circular_orbit(CircularOrbitOrientation::new(inclination, raan))
            .ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;

        let elements =
            OrbitalElements::from_cartesian_state(&state, orbit.mu).ok_or("invalid elements")?;

        let OrbitalElements::CircularInclined(elements) = elements else {
            return Err("expected circular inclined elements");
        };
        assert_close(elements.inclination.radians(), inclination.radians(), EPS);
        assert_close(
            elements.right_ascension_of_ascending_node.radians(),
            raan.radians(),
            EPS,
        );
        assert_close(
            elements.argument_of_latitude.radians(),
            orbit.phase.radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn equatorial_circular_state_maps_to_circular_equatorial_elements() -> TestResult {
        let orbit =
            circular_orbit(CircularOrbitOrientation::equatorial()).ok_or("invalid orbit")?;
        let state = orbit.to_cartesian_state().ok_or("invalid state")?;

        let elements =
            OrbitalElements::from_cartesian_state(&state, orbit.mu).ok_or("invalid elements")?;

        let OrbitalElements::CircularEquatorial(elements) = elements else {
            return Err("expected circular equatorial elements");
        };
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(
            elements.true_longitude.radians(),
            orbit.phase.radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn inclined_eccentric_state_maps_to_classical_elements() -> TestResult {
        let state = state_at(
            Vector3::new(7_000_000.0, 1_000_000.0, 1_300_000.0),
            Vector3::new(-1_000.0, 7_200.0, 1_500.0),
        )
        .ok_or("invalid state")?;

        let elements =
            OrbitalElements::from_cartesian_state(&state, earth_mu().ok_or("invalid mu")?)
                .ok_or("invalid elements")?;

        let OrbitalElements::Classical(elements) = elements else {
            return Err("expected classical elements");
        };
        assert!(elements.eccentricity.scalar() > CIRCULAR_ECCENTRICITY_TOLERANCE);
        assert!(elements.inclination.radians() > EQUATORIAL_NODE_TOLERANCE);
        assert!(elements.semi_major_axis.meters().is_finite());
        Ok(())
    }

    #[test]
    fn equatorial_eccentric_state_maps_to_equatorial_elliptic_elements() -> TestResult {
        let state = state_at(
            Vector3::new(7_000_000.0, 0.0, 0.0),
            Vector3::new(0.0, 8_000.0, 0.0),
        )
        .ok_or("invalid state")?;

        let elements =
            OrbitalElements::from_cartesian_state(&state, earth_mu().ok_or("invalid mu")?)
                .ok_or("invalid elements")?;

        let OrbitalElements::EquatorialElliptic(elements) = elements else {
            return Err("expected equatorial elliptic elements");
        };
        assert!(elements.eccentricity.scalar() > CIRCULAR_ECCENTRICITY_TOLERANCE);
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(elements.longitude_of_periapsis.radians(), 0.0, EPS);
        assert_close(elements.true_anomaly.radians(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn zero_radius_state_has_no_elements() -> TestResult {
        let state = state_at(Vector3::zeros(), Vector3::zeros()).ok_or("invalid state")?;

        assert!(
            OrbitalElements::from_cartesian_state(&state, earth_mu().ok_or("invalid mu")?)
                .is_none()
        );
        Ok(())
    }

    #[test]
    fn angles_are_normalized_to_positive_turn() -> TestResult {
        let angle = angle_from_radians(-0.25).ok_or("invalid angle")?;

        assert_close(angle.radians(), TWO_PI - 0.25, EPS);
        Ok(())
    }
}
