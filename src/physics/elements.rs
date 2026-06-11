use nalgebra::Vector3;

use super::constants::{
    DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE, DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD,
};
use super::state::{CartesianState, CartesianStates};
use super::types::{Angle, Distance, Eccentricity, GravitationalParameter, Position, Velocity};
use crate::errors::NumericalError;

/// Policy for classifying singular orbital-element cases.
///
/// Classical orbital elements become ambiguous near circular and equatorial
/// orbits. This policy controls when a state is reported as one of the
/// singular-safe variants instead of forcing noisy classical angles.
#[derive(Debug, Clone, Copy)]
pub struct OrbitalElementClassification {
    /// Eccentricities at or below this value are treated as circular.
    pub circular_eccentricity_tolerance: Eccentricity,
    /// Inclinations within this angle of 0 or pi are treated as equatorial.
    pub equatorial_inclination_tolerance: Angle,
}

impl OrbitalElementClassification {
    /// Creates a classification policy.
    pub fn new(
        circular_eccentricity_tolerance: Eccentricity,
        equatorial_inclination_tolerance: Angle,
    ) -> Self {
        Self {
            circular_eccentricity_tolerance,
            equatorial_inclination_tolerance,
        }
    }
}

impl Default for OrbitalElementClassification {
    fn default() -> Self {
        // Both constants are compile-time values known to be finite and
        // non-negative — verified by the unreachable! guards below.
        let Ok(circular_eccentricity_tolerance) =
            Eccentricity::from_scalar(DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE)
        else {
            unreachable!("default circular eccentricity tolerance is finite and non-negative");
        };
        let Ok(equatorial_inclination_tolerance) =
            Angle::from_radians(DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD)
        else {
            unreachable!("default equatorial inclination tolerance is finite");
        };
        Self {
            circular_eccentricity_tolerance,
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

impl OrbitalElements {
    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    ///
    /// Returns an error if the state is unbound, has zero angular momentum,
    /// or a computed quantity is non-finite.
    pub fn from_cartesian_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
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
    ) -> Result<Self, NumericalError> {
        Self::from_components(
            state.position.vector(),
            state.velocity.vector(),
            mu,
            classification,
        )
    }

    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    #[inline]
    pub fn from_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
        Self::from_cartesian_state(state, mu)
    }

    /// Computes bound two-body orbital elements using an explicit classification policy.
    #[inline]
    pub fn from_state_with_classification(
        state: &CartesianState,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Result<Self, NumericalError> {
        Self::from_cartesian_state_with_classification(state, mu, classification)
    }

    /// Computes bound two-body orbital elements from an indexed Cartesian state view.
    pub fn from_states(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
    ) -> Result<Self, crate::errors::StateError> {
        Self::from_states_with_classification(
            states,
            index,
            mu,
            OrbitalElementClassification::default(),
        )
    }

    /// Computes bound two-body orbital elements from a state view using an explicit
    /// classification policy.
    pub fn from_states_with_classification(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Result<Self, crate::errors::StateError> {
        let components = states.components(index)?;
        Ok(Self::from_components(
            components.position.vector(),
            components.velocity.vector(),
            mu,
            classification,
        )?)
    }

    fn from_components(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Result<Self, NumericalError> {
        let diagnostics =
            super::diagnostics::OrbitalDiagnostics::from_components(position, velocity, mu)?;

        if diagnostics
            .specific_energy
            .square_meters_per_square_second()
            >= 0.0
        {
            return Err(NumericalError::ConstraintViolation {
                field: "specific energy",
                reason: "must be negative for a bound orbit",
            });
        }

        let semi_major_axis =
            diagnostics
                .semi_major_axis
                .ok_or(NumericalError::ConstraintViolation {
                    field: "semi-major axis",
                    reason: "not defined for unbound orbits",
                })?;
        let eccentricity = diagnostics.eccentricity;
        let eccentricity_scalar = eccentricity.scalar();
        let eccentricity_vector = diagnostics.eccentricity_vector;
        let h = diagnostics.specific_angular_momentum_vector;
        let h_norm = diagnostics
            .specific_angular_momentum
            .square_meters_per_second();
        if h_norm == 0.0 {
            return Err(NumericalError::ZeroNorm {
                field: "angular momentum",
            });
        }

        let h_hat = h / h_norm;
        let node = Vector3::new(-h.y, h.x, 0.0);
        let inclination = angle_from_radians((h.z / h_norm).clamp(-1.0, 1.0).acos())?;

        let is_circular =
            eccentricity_scalar <= classification.circular_eccentricity_tolerance.scalar();
        let is_equatorial =
            is_equatorial_inclination(inclination, classification.equatorial_inclination_tolerance);

        match (is_circular, is_equatorial) {
            (false, false) => Ok(Self::Classical(ClassicalOrbitalElements {
                semi_major_axis,
                eccentricity,
                inclination,
                right_ascension_of_ascending_node: angle_from_radians(node.y.atan2(node.x))?,
                argument_of_periapsis: angle_between_in_plane(node, eccentricity_vector, h_hat)?,
                true_anomaly: angle_between_in_plane(eccentricity_vector, position, h_hat)?,
            })),
            (true, false) => Ok(Self::CircularInclined(CircularInclinedElements {
                semi_major_axis,
                eccentricity,
                inclination,
                right_ascension_of_ascending_node: angle_from_radians(node.y.atan2(node.x))?,
                argument_of_latitude: angle_between_in_plane(node, position, h_hat)?,
            })),
            (true, true) => Ok(Self::CircularEquatorial(CircularEquatorialElements {
                semi_major_axis,
                eccentricity,
                inclination,
                true_longitude: angle_from_radians(position.y.atan2(position.x))?,
            })),
            (false, true) => Ok(Self::EquatorialElliptic(EquatorialEllipticElements {
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
    ///
    /// Returns an error if the orbital parameters are physically invalid or
    /// a computed quantity is non-finite.
    pub fn to_cartesian_state(
        self,
        mu: GravitationalParameter,
    ) -> Result<CartesianState, NumericalError> {
        let mu = mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();
        let eccentricity = self.eccentricity.scalar();
        let inclination = self.inclination.radians();
        let raan = self.right_ascension_of_ascending_node.radians();
        let argument_of_periapsis = self.argument_of_periapsis.radians();
        let true_anomaly = self.true_anomaly.radians();

        if !mu.is_finite() || mu <= 0.0 {
            return Err(NumericalError::NonPositive { field: "mu" });
        }
        if !semi_major_axis.is_finite() || semi_major_axis <= 0.0 {
            return Err(NumericalError::NonPositive {
                field: "semi-major axis",
            });
        }
        if !eccentricity.is_finite() || !(0.0..1.0).contains(&eccentricity) {
            return Err(NumericalError::ConstraintViolation {
                field: "eccentricity",
                reason: "must be finite and in [0, 1)",
            });
        }

        let semi_latus_rectum = semi_major_axis * (1.0 - eccentricity * eccentricity);
        if semi_latus_rectum <= 0.0 {
            return Err(NumericalError::NonPositive {
                field: "semi-latus rectum",
            });
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

        Ok(CartesianState {
            position: Position::from_vector(position)?,
            velocity: Velocity::from_vector(velocity)?,
        })
    }
}

pub(crate) fn perifocal_to_inertial_rotation(
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
) -> Result<Angle, NumericalError> {
    let x = from.dot(&to);
    let y = from.cross(&to).dot(&plane_normal);
    angle_from_radians(y.atan2(x))
}

fn angle_from_radians(radians: f64) -> Result<Angle, NumericalError> {
    Angle::from_radians(normalize_angle(radians))
}

fn normalize_angle(radians: f64) -> f64 {
    let two_pi = 2.0 * std::f64::consts::PI;
    let normalized = radians.rem_euclid(two_pi);
    if normalized == two_pi {
        0.0
    } else {
        normalized
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::physics::dynamics::{OblateBodyGravity, TwoBodyGravity};
    use crate::physics::integrators::{Integrator, VelocityVerletIntegrator};
    use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
    use crate::physics::state::CartesianStateBuffer;
    use crate::physics::types::{
        Angle, Distance, Duration, Eccentricity, Epoch, GravitationalParameter, Position, TimeStep,
        Velocity,
    };

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    fn earth_mu() -> Result<GravitationalParameter, Box<dyn std::error::Error>> {
        Ok(GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?)
    }

    fn circular_orbit(
        inclination: Angle,
        raan: Angle,
    ) -> Result<EllipticOrbit, Box<dyn std::error::Error>> {
        Ok(EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 600_000.0)?,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu: earth_mu()?,
            inclination,
            right_ascension_of_ascending_node: raan,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.4)?,
            direction: OrbitDirection::Prograde,
        })
    }

    fn state_at(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> Result<CartesianState, Box<dyn std::error::Error>> {
        Ok(CartesianState::new(
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
    fn simulated_j2_raan_drift_matches_analytical_rate() -> TestResult {
        let mu = earth_mu()?;
        let inclination = 51.6_f64.to_radians();
        let j2 = J2_EARTH;
        let equatorial_radius = R_EARTH;
        let radius_m = R_EARTH + 600_000.0;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(radius_m)?,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu,
            inclination: Angle::from_radians(inclination)?,
            right_ascension_of_ascending_node: Angle::from_radians(32.0_f64.to_radians())?,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let gravity = OblateBodyGravity {
            mu,
            equatorial_radius: Distance::from_meters(equatorial_radius)?,
            j2,
        };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(30.0)?;
        let duration = Duration::from_seconds(3.0 * 86_400.0)?;
        let initial_raan = raan_degrees(&OrbitalElements::from_states(&states.view(), 0, mu)?)
            .ok_or("initial RAAN undefined")?;
        let mut epoch = Epoch::from_seconds(0.0)?;

        while epoch.seconds() < duration.seconds() {
            let remaining_seconds = duration.seconds() - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))?;
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step)?;
        }

        let final_raan = raan_degrees(&OrbitalElements::from_states(&states.view(), 0, mu)?)
            .ok_or("final RAAN undefined")?;
        let simulated_drift_degrees_per_day =
            angle_delta_degrees(final_raan, initial_raan) / (duration.seconds() / 86_400.0);

        // First-order J2 secular RAAN rate: Ω̇ = −3/2 · J₂ · n · (Rₑ/a)² · cos i
        let mean_motion = (mu.meters_cubed_per_second_squared() / radius_m.powi(3)).sqrt();
        let radius_ratio = equatorial_radius / radius_m;
        let analytic_rad_per_sec =
            -1.5 * j2 * mean_motion * radius_ratio * radius_ratio * inclination.cos();
        let analytical_drift_degrees_per_day = analytic_rad_per_sec.to_degrees() * 86_400.0;

        assert_close(
            simulated_drift_degrees_per_day,
            analytical_drift_degrees_per_day,
            0.1,
        );
        Ok(())
    }

    #[test]
    fn inclined_circular_state_maps_to_circular_inclined_elements() -> TestResult {
        let inclination = Angle::from_radians(0.9)?;
        let raan = Angle::from_radians(1.1)?;
        let orbit = circular_orbit(inclination, raan)?;
        let state = orbit.to_cartesian_state()?;

        let elements = OrbitalElements::from_cartesian_state(&state, orbit.mu)?;

        let OrbitalElements::CircularInclined(elements) = elements else {
            return Err("expected circular inclined elements".into());
        };
        assert_close(elements.inclination.radians(), inclination.radians(), EPS);
        assert_close(
            elements.right_ascension_of_ascending_node.radians(),
            raan.radians(),
            EPS,
        );
        assert_close(
            elements.argument_of_latitude.radians(),
            orbit.true_anomaly.radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn equatorial_circular_state_maps_to_circular_equatorial_elements() -> TestResult {
        let orbit = circular_orbit(Angle::ZERO, Angle::ZERO)?;
        let state = orbit.to_cartesian_state()?;

        let elements = OrbitalElements::from_cartesian_state(&state, orbit.mu)?;

        let OrbitalElements::CircularEquatorial(elements) = elements else {
            return Err("expected circular equatorial elements".into());
        };
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(
            elements.true_longitude.radians(),
            orbit.true_anomaly.radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn propagated_inclined_circular_state_stays_circular_in_elements() -> TestResult {
        let inclination = Angle::from_radians(0.9)?;
        let raan = Angle::from_radians(1.1)?;
        let orbit = circular_orbit(inclination, raan)?;
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let gravity = TwoBodyGravity { mu: orbit.mu };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let target_seconds = orbit.period()?.seconds() * 3.0;
        let mut epoch = Epoch::from_seconds(0.0)?;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))?;
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step)?;
        }

        let elements = OrbitalElements::from_states(&states.view(), 0, orbit.mu)?;

        let OrbitalElements::CircularInclined(elements) = elements else {
            return Err("expected circular inclined elements".into());
        };
        assert!(
            elements.eccentricity.scalar()
                <= OrbitalElementClassification::default()
                    .circular_eccentricity_tolerance
                    .scalar()
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
        let inclination = Angle::from_radians(0.9)?;
        let raan = Angle::from_radians(1.1)?;
        let orbit = circular_orbit(inclination, raan)?;
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let gravity = TwoBodyGravity { mu: orbit.mu };
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let target_seconds = orbit.period()?.seconds() * 3.0;
        let mut epoch = Epoch::from_seconds(0.0)?;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))?;
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step)?;
        }

        let strict_classification = OrbitalElementClassification::new(
            Eccentricity::from_scalar(1.0e-10)?,
            Angle::from_radians(DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD)?,
        );
        let elements = OrbitalElements::from_states_with_classification(
            &states.view(),
            0,
            orbit.mu,
            strict_classification,
        )?;

        let OrbitalElements::Classical(elements) = elements else {
            return Err("expected classical elements".into());
        };
        assert!(
            elements.eccentricity.scalar()
                > strict_classification
                    .circular_eccentricity_tolerance
                    .scalar()
        );
        Ok(())
    }

    #[test]
    fn inclined_eccentric_state_maps_to_classical_elements() -> TestResult {
        let state = state_at(
            Vector3::new(7_000_000.0, 1_000_000.0, 1_300_000.0),
            Vector3::new(-1_000.0, 7_200.0, 1_500.0),
        )?;

        let elements = OrbitalElements::from_cartesian_state(&state, earth_mu()?)?;

        let OrbitalElements::Classical(elements) = elements else {
            return Err("expected classical elements".into());
        };
        assert!(
            elements.eccentricity.scalar()
                > OrbitalElementClassification::default()
                    .circular_eccentricity_tolerance
                    .scalar()
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
        )?;

        let elements = OrbitalElements::from_cartesian_state(&state, earth_mu()?)?;

        let OrbitalElements::EquatorialElliptic(elements) = elements else {
            return Err("expected equatorial elliptic elements".into());
        };
        assert!(
            elements.eccentricity.scalar()
                > OrbitalElementClassification::default()
                    .circular_eccentricity_tolerance
                    .scalar()
        );
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(elements.longitude_of_periapsis.radians(), 0.0, EPS);
        assert_close(elements.true_anomaly.radians(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn classification_policy_rejects_invalid_tolerances() {
        assert!(Eccentricity::from_scalar(f64::NAN).is_err());
        assert!(Eccentricity::from_scalar(-1.0).is_err());
        assert!(Angle::from_radians(f64::NAN).is_err());
    }

    #[test]
    fn classical_elements_round_trip_cartesian_state() -> TestResult {
        let mu = earth_mu()?;
        let elements = ClassicalOrbitalElements {
            semi_major_axis: Distance::from_meters(8_000_000.0)?,
            eccentricity: Eccentricity::from_scalar(0.2)?,
            inclination: Angle::from_radians(0.4)?,
            right_ascension_of_ascending_node: Angle::from_radians(1.0)?,
            argument_of_periapsis: Angle::from_radians(0.7)?,
            true_anomaly: Angle::from_radians(1.2)?,
        };

        let state = elements.to_cartesian_state(mu)?;
        let recovered = OrbitalElements::from_cartesian_state(&state, mu)?;
        let OrbitalElements::Classical(recovered) = recovered else {
            return Err("expected classical elements after round trip".into());
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
        let state = state_at(Vector3::zeros(), Vector3::zeros())?;

        assert!(OrbitalElements::from_cartesian_state(&state, earth_mu()?).is_err());
        Ok(())
    }

    #[test]
    fn angles_are_normalized_to_positive_turn() -> TestResult {
        let angle = angle_from_radians(-0.25)?;

        assert_close(angle.radians(), 2.0 * std::f64::consts::PI - 0.25, EPS);
        Ok(())
    }
}
