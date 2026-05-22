use nalgebra::Vector3;

use crate::diagnostics::Eccentricity;
use crate::state::{CartesianState, CartesianStates};
use crate::types::{Angle, AngularRate, Distance, GravitationalParameter, Position, Velocity};

const DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE: f64 = 1.0e-6;
const DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD: f64 = 1.0e-10;
const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Policy for classifying singular orbital-element cases.
///
/// Classical orbital elements become ambiguous near circular and equatorial
/// orbits. This policy controls when a state is reported as one of the
/// singular-safe variants instead of forcing noisy classical angles.
#[derive(Debug, Clone, Copy)]
pub struct OrbitalElementClassification {
    /// Eccentricities at or below this value are treated as circular.
    pub circular_eccentricity_tolerance: f64,
    /// Inclinations within this angle of 0 or pi are treated as equatorial.
    pub equatorial_inclination_tolerance: Angle,
}

impl OrbitalElementClassification {
    /// Creates a classification policy.
    pub fn new(
        circular_eccentricity_tolerance: f64,
        equatorial_inclination_tolerance: Angle,
    ) -> Option<Self> {
        let policy = Self {
            circular_eccentricity_tolerance,
            equatorial_inclination_tolerance,
        };
        policy.is_valid().then_some(policy)
    }

    fn is_valid(self) -> bool {
        self.circular_eccentricity_tolerance.is_finite()
            && self.circular_eccentricity_tolerance >= 0.0
            && self.equatorial_inclination_tolerance.radians() >= 0.0
    }
}

impl Default for OrbitalElementClassification {
    fn default() -> Self {
        let Some(equatorial_inclination_tolerance) =
            Angle::from_radians(DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD)
        else {
            unreachable!("default equatorial inclination tolerance is finite");
        };

        Self {
            circular_eccentricity_tolerance: DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE,
            equatorial_inclination_tolerance,
        }
    }
}

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

/// First-order J2 secular rates for near-circular orbits.
#[derive(Debug, Clone, Copy)]
pub struct J2SecularRates {
    /// Right ascension of the ascending node rate.
    pub raan_rate: AngularRate,
}

/// Computes first-order J2 RAAN drift for a circular orbit.
///
/// This is a secular validation helper, not a state propagator.
pub fn j2_circular_raan_rate(
    semi_major_axis: Distance,
    inclination: Angle,
    mu: GravitationalParameter,
    equatorial_radius: Distance,
    j2: f64,
) -> Option<AngularRate> {
    let semi_major_axis = semi_major_axis.meters();
    let inclination = inclination.radians();
    let mu = mu.meters_cubed_per_second_squared();
    let equatorial_radius = equatorial_radius.meters();

    if !j2.is_finite() {
        return None;
    }

    debug_assert!(semi_major_axis.is_finite());
    debug_assert!(semi_major_axis > 0.0);
    debug_assert!(inclination.is_finite());
    debug_assert!(mu.is_finite());
    debug_assert!(mu > 0.0);
    debug_assert!(equatorial_radius.is_finite());
    debug_assert!(equatorial_radius > 0.0);

    let mean_motion = (mu / semi_major_axis.powi(3)).sqrt();
    let radius_ratio = equatorial_radius / semi_major_axis;
    AngularRate::from_radians_per_second(
        -1.5 * j2 * mean_motion * radius_ratio * radius_ratio * inclination.cos(),
    )
}

/// Computes first-order J2 secular rates for a circular orbit.
pub fn j2_circular_secular_rates(
    semi_major_axis: Distance,
    inclination: Angle,
    mu: GravitationalParameter,
    equatorial_radius: Distance,
    j2: f64,
) -> Option<J2SecularRates> {
    Some(J2SecularRates {
        raan_rate: j2_circular_raan_rate(semi_major_axis, inclination, mu, equatorial_radius, j2)?,
    })
}

impl OrbitalElements {
    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    pub fn from_cartesian_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Option<Self> {
        Self::from_cartesian_state_with_classification(
            state,
            mu,
            OrbitalElementClassification::default(),
        )
    }

    /// Computes bound two-body orbital elements using an explicit classification policy.
    pub fn from_cartesian_state_with_classification(
        state: &CartesianState,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Option<Self> {
        Self::from_components(
            state.position.vector(),
            state.velocity.vector(),
            mu,
            classification,
        )
    }

    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    #[inline]
    pub fn from_state(state: &CartesianState, mu: GravitationalParameter) -> Option<Self> {
        Self::from_cartesian_state(state, mu)
    }

    /// Computes bound two-body orbital elements using an explicit classification policy.
    #[inline]
    pub fn from_state_with_classification(
        state: &CartesianState,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Option<Self> {
        Self::from_cartesian_state_with_classification(state, mu, classification)
    }

    /// Computes bound two-body orbital elements from an indexed Cartesian state view.
    pub fn from_states(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
    ) -> Option<Self> {
        Self::from_states_with_classification(
            states,
            index,
            mu,
            OrbitalElementClassification::default(),
        )
    }

    /// Computes bound two-body orbital elements from a state view using an explicit classification policy.
    pub fn from_states_with_classification(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Option<Self> {
        let components = states.components(index).ok()?;
        Self::from_components(
            components.position.vector(),
            components.velocity.vector(),
            mu,
            classification,
        )
    }

    fn from_components(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Option<Self> {
        if !classification.is_valid() {
            return None;
        }

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
        let eccentricity_vector = velocity.cross(&h) / mu - position / radius;
        let eccentricity_scalar = eccentricity_vector.norm();
        let eccentricity = Eccentricity::from_scalar(eccentricity_scalar)?;
        let inclination = angle_from_radians((h.z / h_norm).clamp(-1.0, 1.0).acos())?;

        let is_circular = eccentricity_scalar <= classification.circular_eccentricity_tolerance;
        let is_equatorial =
            is_equatorial_inclination(inclination, classification.equatorial_inclination_tolerance);

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

    /// Returns the orbital phase angle used for apoapsis and periapsis detection.
    #[must_use]
    pub fn orbital_phase_angle(&self) -> Angle {
        match self {
            Self::Classical(elements) => elements.true_anomaly,
            Self::CircularInclined(elements) => elements.argument_of_latitude,
            Self::CircularEquatorial(elements) => elements.true_longitude,
            Self::EquatorialElliptic(elements) => elements.true_anomaly,
        }
    }
}

impl ClassicalOrbitalElements {
    /// Builds an inertial Cartesian state from classical Keplerian elements.
    pub fn to_cartesian_state(self, mu: GravitationalParameter) -> Option<CartesianState> {
        let mu = mu.meters_cubed_per_second_squared();
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
        if !eccentricity.is_finite() || !(0.0..1.0).contains(&eccentricity) {
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
        let velocity = rotation * velocity_pqw;

        Some(CartesianState {
            position: Position::from_vector(position)?,
            velocity: Velocity::from_vector(velocity)?,
        })
    }
}

fn perifocal_to_inertial_rotation(
    inclination: f64,
    raan: f64,
    argument_of_periapsis: f64,
) -> nalgebra::Matrix3<f64> {
    let (sin_i, cos_i) = inclination.sin_cos();
    let (sin_raan, cos_raan) = raan.sin_cos();
    let (sin_w, cos_w) = argument_of_periapsis.sin_cos();

    nalgebra::Matrix3::new(
        cos_raan * cos_w - sin_raan * sin_w * cos_i,
        -cos_raan * sin_w - sin_raan * cos_w * cos_i,
        sin_raan * sin_i,
        sin_raan * cos_w + cos_raan * sin_w * cos_i,
        -sin_raan * sin_w + cos_raan * cos_w * cos_i,
        -cos_raan * sin_i,
        sin_w * sin_i,
        cos_w * sin_i,
        cos_i,
    )
}

fn is_equatorial_inclination(inclination: Angle, tolerance: Angle) -> bool {
    let inclination = inclination.radians();
    let tolerance = tolerance.radians();
    inclination <= tolerance || (std::f64::consts::PI - inclination).abs() <= tolerance
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
    use crate::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::dynamics::{OblateBodyGravity, TwoBodyGravity};
    use crate::integrators::{Integrator, VelocityVerletIntegrator};
    use crate::orbits::{CircularOrbit, CircularOrbitOrientation, OrbitDirection};
    use crate::state::CartesianStateBuffer;
    use crate::types::{Duration, Epoch, Position, TimeStep, Velocity};

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

    fn angle_delta_degrees(final_degrees: f64, initial_degrees: f64) -> f64 {
        let delta = (final_degrees - initial_degrees).rem_euclid(360.0);
        if delta > 180.0 { delta - 360.0 } else { delta }
    }

    fn raan_degrees(elements: &OrbitalElements) -> Option<f64> {
        match elements {
            OrbitalElements::Classical(elements) => Some(
                elements
                    .right_ascension_of_ascending_node
                    .radians()
                    .to_degrees(),
            ),
            OrbitalElements::CircularInclined(elements) => Some(
                elements
                    .right_ascension_of_ascending_node
                    .radians()
                    .to_degrees(),
            ),
            OrbitalElements::CircularEquatorial(_) | OrbitalElements::EquatorialElliptic(_) => None,
        }
    }

    #[test]
    fn j2_circular_raan_rate_is_negative_for_inclined_leo() -> TestResult {
        let rate = j2_circular_raan_rate(
            Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid semi-major axis")?,
            Angle::from_radians(51.6_f64.to_radians()).ok_or("invalid inclination")?,
            earth_mu().ok_or("invalid mu")?,
            Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            J2_EARTH,
        )
        .ok_or("invalid rate")?;

        assert!(rate.radians_per_second() < 0.0);
        assert_close(rate.degrees_per_day(), -4.518243696, 1.0e-9);
        Ok(())
    }

    #[test]
    fn j2_circular_raan_rate_is_zero_when_j2_is_zero() -> TestResult {
        let rate = j2_circular_raan_rate(
            Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid semi-major axis")?,
            Angle::from_radians(51.6_f64.to_radians()).ok_or("invalid inclination")?,
            earth_mu().ok_or("invalid mu")?,
            Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            0.0,
        )
        .ok_or("invalid rate")?;

        assert_close(rate.radians_per_second(), 0.0, EPS);
        assert_close(rate.degrees_per_day(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn j2_circular_raan_rate_is_zero_for_polar_orbit() -> TestResult {
        let rate = j2_circular_raan_rate(
            Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid semi-major axis")?,
            Angle::from_radians(std::f64::consts::FRAC_PI_2).ok_or("invalid inclination")?,
            earth_mu().ok_or("invalid mu")?,
            Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            J2_EARTH,
        )
        .ok_or("invalid rate")?;

        assert_close(rate.degrees_per_day(), 0.0, 1.0e-12);
        Ok(())
    }

    #[test]
    fn j2_circular_raan_rate_rejects_nonfinite_j2() -> TestResult {
        assert!(
            j2_circular_raan_rate(
                Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid semi-major axis")?,
                Angle::from_radians(51.6_f64.to_radians()).ok_or("invalid inclination")?,
                earth_mu().ok_or("invalid mu")?,
                Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
                f64::NAN,
            )
            .is_none()
        );
        Ok(())
    }

    #[test]
    fn j2_circular_secular_rates_wraps_raan_rate() -> TestResult {
        let rates = j2_circular_secular_rates(
            Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid semi-major axis")?,
            Angle::from_radians(51.6_f64.to_radians()).ok_or("invalid inclination")?,
            earth_mu().ok_or("invalid mu")?,
            Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            J2_EARTH,
        )
        .ok_or("invalid rates")?;

        assert!(rates.raan_rate.radians_per_second() < 0.0);
        Ok(())
    }

    #[test]
    fn simulated_j2_raan_drift_matches_analytical_rate() -> TestResult {
        let mu = earth_mu().ok_or("invalid mu")?;
        let orbit = CircularOrbit {
            radius: Distance::from_meters(R_EARTH + 600_000.0).ok_or("invalid radius")?,
            mu,
            orientation: CircularOrbitOrientation::new(
                Angle::from_radians(51.6_f64.to_radians()).ok_or("invalid inclination")?,
                Angle::from_radians(32.0_f64.to_radians()).ok_or("invalid raan")?,
            ),
            phase: Angle::from_radians(0.0).ok_or("invalid phase")?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;
        let gravity = OblateBodyGravity {
            mu,
            equatorial_radius: Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            j2: J2_EARTH,
        };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(30.0).ok_or("invalid time step")?;
        let duration = Duration::from_seconds(3.0 * 86_400.0).ok_or("invalid duration")?;
        let initial_raan = raan_degrees(
            &OrbitalElements::from_states(&states.view(), 0, mu)
                .ok_or("invalid initial elements")?,
        )
        .ok_or("initial RAAN undefined")?;
        let mut epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        while epoch.seconds() < duration.seconds() {
            let remaining_seconds = duration.seconds() - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))
                .ok_or("invalid step")?;
            integrator
                .step(&gravity, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).ok_or("invalid next epoch")?;
        }

        let final_raan = raan_degrees(
            &OrbitalElements::from_states(&states.view(), 0, mu).ok_or("invalid final elements")?,
        )
        .ok_or("final RAAN undefined")?;
        let simulated_drift_degrees_per_day =
            angle_delta_degrees(final_raan, initial_raan) / (duration.seconds() / 86_400.0);
        let analytical_drift_degrees_per_day = j2_circular_raan_rate(
            orbit.radius,
            orbit.orientation.inclination,
            mu,
            Distance::from_meters(R_EARTH).ok_or("invalid equatorial radius")?,
            J2_EARTH,
        )
        .ok_or("invalid analytical rate")?
        .degrees_per_day();

        assert_close(
            simulated_drift_degrees_per_day,
            analytical_drift_degrees_per_day,
            0.1,
        );
        Ok(())
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
    fn propagated_inclined_circular_state_stays_circular_in_elements() -> TestResult {
        let inclination = Angle::from_radians(0.9).ok_or("invalid inclination")?;
        let raan = Angle::from_radians(1.1).ok_or("invalid raan")?;
        let orbit = circular_orbit(CircularOrbitOrientation::new(inclination, raan))
            .ok_or("invalid orbit")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;
        let gravity = TwoBodyGravity { mu: orbit.mu };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(10.0).ok_or("invalid time step")?;
        let target_seconds = orbit.period().ok_or("invalid period")?.seconds() * 3.0;
        let mut epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))
                .ok_or("invalid step")?;
            integrator
                .step(&gravity, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).ok_or("invalid next epoch")?;
        }

        let elements =
            OrbitalElements::from_states(&states.view(), 0, orbit.mu).ok_or("invalid elements")?;

        let OrbitalElements::CircularInclined(elements) = elements else {
            return Err("expected circular inclined elements");
        };
        assert!(
            elements.eccentricity.scalar()
                <= OrbitalElementClassification::default().circular_eccentricity_tolerance
        );
        assert_close(
            elements.inclination.radians(),
            inclination.radians(),
            1.0e-9,
        );
        assert_close(
            elements.right_ascension_of_ascending_node.radians(),
            raan.radians(),
            1.0e-9,
        );
        Ok(())
    }

    #[test]
    fn strict_classification_can_keep_near_circular_state_classical() -> TestResult {
        let inclination = Angle::from_radians(0.9).ok_or("invalid inclination")?;
        let raan = Angle::from_radians(1.1).ok_or("invalid raan")?;
        let orbit = circular_orbit(CircularOrbitOrientation::new(inclination, raan))
            .ok_or("invalid orbit")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;
        let gravity = TwoBodyGravity { mu: orbit.mu };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(10.0).ok_or("invalid time step")?;
        let target_seconds = orbit.period().ok_or("invalid period")?.seconds() * 3.0;
        let mut epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))
                .ok_or("invalid step")?;
            integrator
                .step(&gravity, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).ok_or("invalid next epoch")?;
        }

        let strict_classification = OrbitalElementClassification::new(
            1.0e-10,
            Angle::from_radians(DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD)
                .ok_or("invalid tolerance")?,
        )
        .ok_or("invalid classification")?;
        let elements = OrbitalElements::from_states_with_classification(
            &states.view(),
            0,
            orbit.mu,
            strict_classification,
        )
        .ok_or("invalid elements")?;

        let OrbitalElements::Classical(elements) = elements else {
            return Err("expected classical elements");
        };
        assert!(
            elements.eccentricity.scalar() > strict_classification.circular_eccentricity_tolerance
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
        assert!(
            elements.eccentricity.scalar()
                > OrbitalElementClassification::default().circular_eccentricity_tolerance
        );
        assert!(
            elements.inclination.radians()
                > OrbitalElementClassification::default()
                    .equatorial_inclination_tolerance
                    .radians()
        );
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
        assert!(
            elements.eccentricity.scalar()
                > OrbitalElementClassification::default().circular_eccentricity_tolerance
        );
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(elements.longitude_of_periapsis.radians(), 0.0, EPS);
        assert_close(elements.true_anomaly.radians(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn classification_policy_rejects_invalid_tolerances() -> TestResult {
        let zero = Angle::from_radians(0.0).ok_or("invalid zero tolerance")?;
        let negative = Angle::from_radians(-1.0).ok_or("invalid negative tolerance")?;

        assert!(OrbitalElementClassification::new(f64::NAN, zero).is_none());
        assert!(OrbitalElementClassification::new(-1.0, zero).is_none());
        assert!(OrbitalElementClassification::new(1.0e-6, negative).is_none());
        Ok(())
    }

    #[test]
    fn classical_elements_round_trip_cartesian_state() -> TestResult {
        let mu = earth_mu().ok_or("invalid mu")?;
        let elements = ClassicalOrbitalElements {
            semi_major_axis: Distance::from_meters(8_000_000.0).ok_or("invalid sma")?,
            eccentricity: Eccentricity::from_scalar(0.2).ok_or("invalid eccentricity")?,
            inclination: Angle::from_radians(0.4).ok_or("invalid inclination")?,
            right_ascension_of_ascending_node: Angle::from_radians(1.0).ok_or("invalid raan")?,
            argument_of_periapsis: Angle::from_radians(0.7).ok_or("invalid argp")?,
            true_anomaly: Angle::from_radians(1.2).ok_or("invalid true anomaly")?,
        };

        let state = elements
            .to_cartesian_state(mu)
            .ok_or("failed to build cartesian state")?;
        let recovered = OrbitalElements::from_cartesian_state(&state, mu)
            .ok_or("invalid recovered elements")?;
        let OrbitalElements::Classical(recovered) = recovered else {
            return Err("expected classical elements after round trip");
        };

        assert_close(
            recovered.semi_major_axis.meters(),
            elements.semi_major_axis.meters(),
            1.0e-3,
        );
        assert_close(
            recovered.eccentricity.scalar(),
            elements.eccentricity.scalar(),
            1.0e-6,
        );
        assert_close(
            recovered.inclination.radians(),
            elements.inclination.radians(),
            1.0e-6,
        );
        assert_close(
            recovered.true_anomaly.radians(),
            elements.true_anomaly.radians(),
            1.0e-6,
        );
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
