use nalgebra::Vector3;

use super::frame::Frame;
use super::state::CartesianState;
use super::types::{Angle, Distance, Eccentricity, GravitationalParameter, Position, Velocity};
use crate::errors::{NumericalError, StateError};

/// Policy for classifying singular orbital-element cases.
///
/// Classical orbital elements become ambiguous near circular and equatorial
/// orbits. This policy controls when a state is reported as one of the
/// singular-safe variants instead of forcing noisy classical angles.
#[derive(Debug, Clone, Copy)]
pub struct OrbitalElementClassification {
    /// Eccentricities at or below this value are treated as circular.
    circular_eccentricity_tolerance: Eccentricity,
    /// Inclinations within this angle of 0 or pi are treated as equatorial.
    equatorial_inclination_tolerance: Angle,
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

    /// Eccentricities at or below this value are treated as circular.
    #[inline]
    #[must_use]
    pub const fn circular_eccentricity_tolerance(self) -> Eccentricity {
        self.circular_eccentricity_tolerance
    }

    /// Inclinations within this angle of 0 or pi are treated as equatorial.
    #[inline]
    #[must_use]
    pub const fn equatorial_inclination_tolerance(self) -> Angle {
        self.equatorial_inclination_tolerance
    }
}

impl Default for OrbitalElementClassification {
    fn default() -> Self {
        Self {
            circular_eccentricity_tolerance:
                Eccentricity::DEFAULT_CIRCULAR_CLASSIFICATION_TOLERANCE,
            equatorial_inclination_tolerance: Angle::DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE,
        }
    }
}

/// Orbital elements for bound two-body Cartesian states.
///
/// Classical orbital elements have singularities for circular and equatorial
/// orbits, so this enum uses variants that expose only the angles that are
/// physically defined for the state.
#[derive(Debug, Clone, Copy, PartialEq)]
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ClassicalOrbitalElements {
    /// Semi-major axis for the bound orbit.
    semi_major_axis: Distance,
    /// Orbital eccentricity.
    eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    inclination: Angle,
    /// Right ascension of the ascending node.
    right_ascension_of_ascending_node: Angle,
    /// Argument of periapsis.
    argument_of_periapsis: Angle,
    /// True anomaly.
    true_anomaly: Angle,
}

/// Elements for a circular inclined bound orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularInclinedElements {
    /// Semi-major axis for the bound orbit.
    semi_major_axis: Distance,
    /// Orbital eccentricity.
    eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    inclination: Angle,
    /// Right ascension of the ascending node.
    right_ascension_of_ascending_node: Angle,
    /// Argument of latitude measured from the ascending node.
    argument_of_latitude: Angle,
}

/// Elements for a circular equatorial bound orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CircularEquatorialElements {
    /// Semi-major axis for the bound orbit.
    semi_major_axis: Distance,
    /// Orbital eccentricity.
    eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    inclination: Angle,
    /// True longitude measured in the inertial x-y plane.
    true_longitude: Angle,
}

/// Elements for an eccentric equatorial bound orbit.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EquatorialEllipticElements {
    /// Semi-major axis for the bound orbit.
    semi_major_axis: Distance,
    /// Orbital eccentricity.
    eccentricity: Eccentricity,
    /// Inclination from the inertial x-y plane.
    inclination: Angle,
    /// Longitude of periapsis measured in the inertial x-y plane.
    longitude_of_periapsis: Angle,
    /// True anomaly measured from periapsis.
    true_anomaly: Angle,
}

impl OrbitalElements {
    // ---------------- Construction ----------------

    /// Computes bound two-body orbital elements from an inertial Cartesian state.
    ///
    /// `classification` controls when singular-safe variants are used instead
    /// of classical elements. See [`OrbitalElementClassification`] for details.
    ///
    /// Returns an error if the state is unbound, has zero angular momentum,
    /// or a computed quantity is non-finite.
    pub fn from_state(
        state: &CartesianState,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Result<Self, NumericalError> {
        Self::from_components(state.position, state.velocity, mu, classification)
    }

    fn from_components(
        position: Position,
        velocity: Velocity,
        mu: GravitationalParameter,
        classification: OrbitalElementClassification,
    ) -> Result<Self, NumericalError> {
        let position_v = position.vector();
        let velocity_v = velocity.vector();
        let mu_f64 = mu.meters_cubed_per_second_squared();
        let radius = position_v.norm();
        let speed = velocity_v.norm();

        if radius == 0.0 {
            return Err(NumericalError::ZeroNorm { field: "position" });
        }

        let specific_energy = 0.5 * speed * speed - mu_f64 / radius;
        if specific_energy >= 0.0 {
            return Err(NumericalError::ConstraintViolation {
                field: "specific energy",
                reason: "must be negative for a bound orbit",
            });
        }

        let h = position_v.cross(&velocity_v);
        let h_norm = h.norm();
        if h_norm == 0.0 {
            return Err(NumericalError::ZeroNorm {
                field: "angular momentum",
            });
        }

        let eccentricity_vector = velocity_v.cross(&h) / mu_f64 - position_v / radius;
        let eccentricity = Eccentricity::new(eccentricity_vector.norm());
        let eccentricity_scalar = eccentricity.scalar();
        let semi_major_axis = Distance::new(-mu_f64 / (2.0 * specific_energy));

        let h_hat = h / h_norm;
        let node = Vector3::new(-h.y, h.x, 0.0);
        let inclination = Angle::new((h.z / h_norm).clamp(-1.0, 1.0).acos());

        let is_circular =
            eccentricity_scalar <= classification.circular_eccentricity_tolerance.scalar();
        let is_equatorial =
            is_equatorial_inclination(inclination, classification.equatorial_inclination_tolerance);

        match (is_circular, is_equatorial) {
            (false, false) => Ok(Self::Classical(ClassicalOrbitalElements::new(
                semi_major_axis,
                eccentricity,
                inclination,
                Angle::new(node.y.atan2(node.x)),
                angle_between_in_plane(node, eccentricity_vector, h_hat),
                angle_between_in_plane(eccentricity_vector, position_v, h_hat),
            ))),
            (true, false) => Ok(Self::CircularInclined(CircularInclinedElements::new(
                semi_major_axis,
                eccentricity,
                inclination,
                Angle::new(node.y.atan2(node.x)),
                angle_between_in_plane(node, position_v, h_hat),
            ))),
            (true, true) => Ok(Self::CircularEquatorial(CircularEquatorialElements::new(
                semi_major_axis,
                eccentricity,
                inclination,
                Angle::new(position_v.y.atan2(position_v.x)),
            ))),
            (false, true) => Ok(Self::EquatorialElliptic(EquatorialEllipticElements::new(
                semi_major_axis,
                eccentricity,
                inclination,
                Angle::new(eccentricity_vector.y.atan2(eccentricity_vector.x)),
                angle_between_in_plane(eccentricity_vector, position_v, h_hat),
            ))),
        }
    }

    // ---------------- Queries ----------------

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

    /// Builds an inertial Cartesian state from any bound orbital-element variant.
    ///
    /// Each variant maps onto the canonical `(RAAN, argument of periapsis, true
    /// anomaly)` triple used by the perifocal-to-inertial transform: angles that
    /// are undefined for a singular variant are set to zero, and the variant's
    /// defined in-plane angle (argument of latitude, true longitude, or longitude
    /// of periapsis) takes the corresponding slot.
    pub fn to_cartesian_state(self, frame: &Frame) -> Result<CartesianState, StateError> {
        frame.require_inertial_axes()?;
        let (raan, argument_of_periapsis, true_anomaly) = self.canonical_angles();
        EllipticElementSet::new(
            self.semi_major_axis(),
            self.eccentricity(),
            frame.mu()?,
            self.inclination(),
            raan,
            argument_of_periapsis,
            true_anomaly,
        )?
        .to_cartesian_state()
    }

    /// Maps a variant onto the `(RAAN, argument of periapsis, true anomaly)` triple
    /// expected by [`EllipticElementSet`].
    ///
    /// Undefined angles are zeroed by convention; the variant's defined in-plane
    /// angle fills the true-anomaly slot (or, for `EquatorialElliptic`, the
    /// argument-of-periapsis slot with true anomaly kept separate).
    #[inline]
    const fn canonical_angles(self) -> (Angle, Angle, Angle) {
        match self {
            Self::Classical(e) => (
                e.right_ascension_of_ascending_node,
                e.argument_of_periapsis,
                e.true_anomaly,
            ),
            Self::CircularInclined(e) => (
                e.right_ascension_of_ascending_node,
                Angle::ZERO,
                e.argument_of_latitude,
            ),
            Self::CircularEquatorial(e) => (Angle::ZERO, Angle::ZERO, e.true_longitude),
            Self::EquatorialElliptic(e) => (Angle::ZERO, e.longitude_of_periapsis, e.true_anomaly),
        }
    }

    // ---------------- Common Accessors ----------------

    /// Semi-major axis (common to all bound-element variants).
    #[inline]
    pub const fn semi_major_axis(&self) -> Distance {
        match self {
            Self::Classical(e) => e.semi_major_axis(),
            Self::CircularInclined(e) => e.semi_major_axis(),
            Self::CircularEquatorial(e) => e.semi_major_axis(),
            Self::EquatorialElliptic(e) => e.semi_major_axis(),
        }
    }

    /// Orbital eccentricity (common to all bound-element variants).
    #[inline]
    pub const fn eccentricity(&self) -> Eccentricity {
        match self {
            Self::Classical(e) => e.eccentricity(),
            Self::CircularInclined(e) => e.eccentricity(),
            Self::CircularEquatorial(e) => e.eccentricity(),
            Self::EquatorialElliptic(e) => e.eccentricity(),
        }
    }

    /// Inclination from the inertial x-y plane (common to all variants).
    #[inline]
    pub const fn inclination(&self) -> Angle {
        match self {
            Self::Classical(e) => e.inclination(),
            Self::CircularInclined(e) => e.inclination(),
            Self::CircularEquatorial(e) => e.inclination(),
            Self::EquatorialElliptic(e) => e.inclination(),
        }
    }

    /// Right ascension of the ascending node, when defined.
    ///
    /// Returns `None` for equatorial variants where RAAN is undefined.
    #[inline]
    pub const fn right_ascension_of_ascending_node(&self) -> Option<Angle> {
        match self {
            Self::Classical(e) => Some(e.right_ascension_of_ascending_node()),
            Self::CircularInclined(e) => Some(e.right_ascension_of_ascending_node()),
            Self::CircularEquatorial(_) | Self::EquatorialElliptic(_) => None,
        }
    }
}

impl ClassicalOrbitalElements {
    /// Creates a classical orbital-element set from domain quantities.
    #[inline]
    pub(crate) const fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        inclination: Angle,
        right_ascension_of_ascending_node: Angle,
        argument_of_periapsis: Angle,
        true_anomaly: Angle,
    ) -> Self {
        Self {
            semi_major_axis,
            eccentricity,
            inclination,
            right_ascension_of_ascending_node,
            argument_of_periapsis,
            true_anomaly,
        }
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
}

impl CircularInclinedElements {
    /// Creates a circular inclined orbital-element set from domain quantities.
    #[inline]
    pub(crate) const fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        inclination: Angle,
        right_ascension_of_ascending_node: Angle,
        argument_of_latitude: Angle,
    ) -> Self {
        Self {
            semi_major_axis,
            eccentricity,
            inclination,
            right_ascension_of_ascending_node,
            argument_of_latitude,
        }
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

    /// Returns the argument of latitude.
    #[inline]
    pub const fn argument_of_latitude(&self) -> Angle {
        self.argument_of_latitude
    }
}

impl CircularEquatorialElements {
    /// Creates a circular equatorial orbital-element set from domain quantities.
    #[inline]
    pub(crate) const fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        inclination: Angle,
        true_longitude: Angle,
    ) -> Self {
        Self {
            semi_major_axis,
            eccentricity,
            inclination,
            true_longitude,
        }
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

    /// Returns the inclination.
    #[inline]
    pub const fn inclination(&self) -> Angle {
        self.inclination
    }

    /// Returns the true longitude.
    #[inline]
    pub const fn true_longitude(&self) -> Angle {
        self.true_longitude
    }
}

impl EquatorialEllipticElements {
    /// Creates an eccentric equatorial orbital-element set from domain quantities.
    #[inline]
    pub(crate) const fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        inclination: Angle,
        longitude_of_periapsis: Angle,
        true_anomaly: Angle,
    ) -> Self {
        Self {
            semi_major_axis,
            eccentricity,
            inclination,
            longitude_of_periapsis,
            true_anomaly,
        }
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

    /// Returns the inclination.
    #[inline]
    pub const fn inclination(&self) -> Angle {
        self.inclination
    }

    /// Returns the longitude of periapsis.
    #[inline]
    pub const fn longitude_of_periapsis(&self) -> Angle {
        self.longitude_of_periapsis
    }

    /// Returns the true anomaly.
    #[inline]
    pub const fn true_anomaly(&self) -> Angle {
        self.true_anomaly
    }
}

pub(super) struct EllipticElementSet {
    semi_major_axis: Distance,
    eccentricity: Eccentricity,
    mu: GravitationalParameter,
    inclination: Angle,
    right_ascension_of_ascending_node: Angle,
    argument_of_periapsis: Angle,
    true_anomaly: Angle,
}

impl EllipticElementSet {
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn new(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        mu: GravitationalParameter,
        inclination: Angle,
        right_ascension_of_ascending_node: Angle,
        argument_of_periapsis: Angle,
        true_anomaly: Angle,
    ) -> Result<Self, NumericalError> {
        if eccentricity.scalar() >= 1.0 {
            return Err(NumericalError::ConstraintViolation {
                field: "eccentricity",
                reason: "must be less than 1 for elliptic orbits",
            });
        }
        if semi_major_axis.meters() <= 0.0 {
            return Err(NumericalError::NonPositive {
                field: "semi-major axis",
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
        })
    }

    pub(crate) fn to_cartesian_state(&self) -> Result<CartesianState, StateError> {
        let mu = self.mu.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis.meters();
        let eccentricity = self.eccentricity.scalar();
        let true_anomaly = self.true_anomaly.radians();

        // Invariants guaranteed by EllipticElementSet::new:
        // eccentricity ∈ [0, 1), semi_major_axis > 0.
        // Together these imply semi_latus_rectum > 0 and radius > 0.

        let semi_latus_rectum = semi_major_axis * (1.0 - eccentricity * eccentricity);

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

        let rotation = perifocal_to_inertial_rotation(
            self.inclination,
            self.right_ascension_of_ascending_node,
            self.argument_of_periapsis,
        );
        let position = rotation * position_pqw;
        let velocity = rotation * velocity_pqw;

        Ok(CartesianState::new(
            Position::from_vector(position),
            Velocity::from_vector(velocity),
        ))
    }
}

fn perifocal_to_inertial_rotation(
    inclination: Angle,
    raan: Angle,
    argument_of_periapsis: Angle,
) -> nalgebra::Matrix3<f64> {
    let inclination = inclination.radians();
    let raan = raan.radians();
    let argument_of_periapsis = argument_of_periapsis.radians();
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
) -> Angle {
    let x = from.dot(&to);
    let y = from.cross(&to).dot(&plane_normal);
    // `Angle::new` wraps the signed `atan2` result into `[0, 2π)`.
    Angle::new(y.atan2(x))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{
        DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD, J2_EARTH, MU_EARTH, R_EARTH,
    };
    use crate::physics::dynamics::{J2Gravity, TwoBodyGravity};
    use crate::physics::frame::Frame;
    use crate::physics::integrators::{Integrator, VelocityVerletIntegrator};
    use crate::physics::orbits::Orbit;
    use crate::physics::state::CartesianStateBuffer;
    use crate::physics::types::{
        Angle, Distance, Duration, Eccentricity, Epoch, GravitationalParameter, J2Coefficient,
        Position, TimeStep, Velocity,
    };

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    fn earth_mu() -> Result<GravitationalParameter, Box<dyn std::error::Error>> {
        Ok(GravitationalParameter::new(MU_EARTH))
    }

    fn frame() -> Frame {
        Frame::earth_inertial()
    }

    fn circular_orbit(
        inclination: Angle,
        raan: Angle,
    ) -> Result<Orbit, Box<dyn std::error::Error>> {
        Ok(Orbit::elliptic(
            Distance::new(R_EARTH + 600_000.0),
            Eccentricity::new(0.0),
            inclination,
            raan,
            Angle::ZERO,
            Angle::new(0.4),
        ))
    }

    fn state_at(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> Result<CartesianState, Box<dyn std::error::Error>> {
        Ok(CartesianState::new(
            Position::from_vector(position),
            Velocity::from_vector(velocity),
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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let inclination = 51.6_f64.to_radians();
        let j2 = J2_EARTH;
        let equatorial_radius = Distance::new(frame.equatorial_radius()?.meters());
        let altitude_m = 600_000.0;
        let orbit = Orbit::circular_inclined(
            Distance::new(altitude_m),
            Angle::new(inclination),
            Angle::new(32.0_f64.to_radians()),
        );
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;
        let gravity = J2Gravity::new(mu, equatorial_radius, J2Coefficient::new(j2));
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::new(30.0);
        let duration = Duration::new(3.0 * 86_400.0);
        let initial_state = states.view().read_state(0)?;
        let initial_raan = raan_degrees(&OrbitalElements::from_state(
            &initial_state,
            mu,
            OrbitalElementClassification::default(),
        )?)
        .ok_or("initial RAAN undefined")?;
        let mut epoch = Epoch::ZERO;

        while epoch.seconds() < duration.seconds() {
            let remaining_seconds = duration.seconds() - epoch.seconds();
            let step = TimeStep::new(dt.seconds().min(remaining_seconds));
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        let final_state = states.view().read_state(0)?;
        let final_raan = raan_degrees(&OrbitalElements::from_state(
            &final_state,
            mu,
            OrbitalElementClassification::default(),
        )?)
        .ok_or("final RAAN undefined")?;
        let simulated_drift_degrees_per_day =
            angle_delta_degrees(final_raan, initial_raan) / (duration.seconds() / 86_400.0);

        // First-order J2 secular RAAN rate: Ω̇ = −3/2 · J₂ · n · (Rₑ/a)² · cos i
        let semi_major_axis_m = orbit.semi_major_axis(&frame)?.meters();
        let mean_motion = (mu.meters_cubed_per_second_squared() / semi_major_axis_m.powi(3)).sqrt();
        let radius_ratio = equatorial_radius.meters() / semi_major_axis_m;
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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let inclination = Angle::new(0.9);
        let raan = Angle::new(1.1);
        let orbit = circular_orbit(inclination, raan)?;
        let state = orbit.to_cartesian_state(&frame)?;

        let elements =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;

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
            orbit.true_anomaly().radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn equatorial_circular_state_maps_to_circular_equatorial_elements() -> TestResult {
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let orbit = circular_orbit(Angle::ZERO, Angle::ZERO)?;
        let state = orbit.to_cartesian_state(&frame)?;

        let elements =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;

        let OrbitalElements::CircularEquatorial(elements) = elements else {
            return Err("expected circular equatorial elements".into());
        };
        assert_close(elements.inclination.radians(), 0.0, EPS);
        assert_close(
            elements.true_longitude.radians(),
            orbit.true_anomaly().radians(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn propagated_inclined_circular_state_stays_circular_in_elements() -> TestResult {
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let inclination = Angle::new(0.9);
        let raan = Angle::new(1.1);
        let orbit = circular_orbit(inclination, raan)?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::new(10.0);
        let target_seconds = orbit.period(&frame)?.seconds() * 3.0;
        let mut epoch = Epoch::ZERO;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::new(dt.seconds().min(remaining_seconds));
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        let state = states.view().read_state(0)?;
        let elements =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;

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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let inclination = Angle::new(0.9);
        let raan = Angle::new(1.1);
        let orbit = circular_orbit(inclination, raan)?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let integrator = VelocityVerletIntegrator;
        let dt = TimeStep::new(10.0);
        let target_seconds = orbit.period(&frame)?.seconds() * 3.0;
        let mut epoch = Epoch::ZERO;

        while epoch.seconds() < target_seconds {
            let remaining_seconds = target_seconds - epoch.seconds();
            let step = TimeStep::new(dt.seconds().min(remaining_seconds));
            integrator.step(&gravity, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        let strict_classification = OrbitalElementClassification::new(
            Eccentricity::new(1.0e-10),
            Angle::new(DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD),
        );
        let state = states.view().read_state(0)?;
        let elements = OrbitalElements::from_state(&state, mu, strict_classification)?;

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

        let elements = OrbitalElements::from_state(
            &state,
            earth_mu()?,
            OrbitalElementClassification::default(),
        )?;

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

        let elements = OrbitalElements::from_state(
            &state,
            earth_mu()?,
            OrbitalElementClassification::default(),
        )?;

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
    fn classical_elements_round_trip_cartesian_state() -> TestResult {
        let mu = earth_mu()?;
        let elements = ClassicalOrbitalElements::new(
            Distance::new(8_000_000.0),
            Eccentricity::new(0.2),
            Angle::new(0.4),
            Angle::new(1.0),
            Angle::new(0.7),
            Angle::new(1.2),
        );

        let state = OrbitalElements::Classical(elements).to_cartesian_state(&frame())?;
        let recovered =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;
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
    fn circular_inclined_elements_convert_to_cartesian_state() -> TestResult {
        let mu = earth_mu()?;
        let elements = CircularInclinedElements::new(
            Distance::new(8_000_000.0),
            Eccentricity::new(0.0),
            Angle::new(0.4),
            Angle::new(1.0),
            Angle::new(1.2),
        );

        let state = OrbitalElements::CircularInclined(elements).to_cartesian_state(&frame())?;
        let recovered =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;
        let OrbitalElements::CircularInclined(recovered) = recovered else {
            return Err("expected circular inclined elements after round trip".into());
        };

        assert_close(
            recovered.semi_major_axis.meters(),
            elements.semi_major_axis.meters(),
            1.0e-3,
        );
        assert_close(
            recovered.argument_of_latitude.radians(),
            elements.argument_of_latitude.radians(),
            1.0e-6,
        );
        Ok(())
    }

    #[test]
    fn circular_equatorial_elements_convert_to_cartesian_state() -> TestResult {
        let mu = earth_mu()?;
        let elements = CircularEquatorialElements::new(
            Distance::new(8_000_000.0),
            Eccentricity::new(0.0),
            Angle::ZERO,
            Angle::new(1.2),
        );

        let state = OrbitalElements::CircularEquatorial(elements).to_cartesian_state(&frame())?;
        let recovered =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;
        let OrbitalElements::CircularEquatorial(recovered) = recovered else {
            return Err("expected circular equatorial elements after round trip".into());
        };

        assert_close(
            recovered.semi_major_axis.meters(),
            elements.semi_major_axis.meters(),
            1.0e-3,
        );
        assert_close(
            recovered.true_longitude.radians(),
            elements.true_longitude.radians(),
            1.0e-6,
        );
        Ok(())
    }

    #[test]
    fn equatorial_elliptic_elements_convert_to_cartesian_state() -> TestResult {
        let mu = earth_mu()?;
        let elements = EquatorialEllipticElements::new(
            Distance::new(8_000_000.0),
            Eccentricity::new(0.2),
            Angle::ZERO,
            Angle::new(0.7),
            Angle::new(1.2),
        );

        let state = OrbitalElements::EquatorialElliptic(elements).to_cartesian_state(&frame())?;
        let recovered =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;
        let OrbitalElements::EquatorialElliptic(recovered) = recovered else {
            return Err("expected equatorial elliptic elements after round trip".into());
        };

        assert_close(
            recovered.eccentricity.scalar(),
            elements.eccentricity.scalar(),
            1.0e-6,
        );
        assert_close(
            recovered.longitude_of_periapsis.radians(),
            elements.longitude_of_periapsis.radians(),
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

        assert!(
            OrbitalElements::from_state(
                &state,
                earth_mu()?,
                OrbitalElementClassification::default()
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn unbound_state_rejected() -> TestResult {
        // Escape velocity at 7,000 km from Earth: √(2 * 3.986e14 / 7e6) ≈ 10,672 m/s.
        // Use 11 km/s to ensure specific energy is positive (parabolic/hyperbolic).
        let state = state_at(
            Vector3::new(7_000_000.0, 0.0, 0.0),
            Vector3::new(0.0, 11_000.0, 0.0),
        )?;

        let result = OrbitalElements::from_state(
            &state,
            earth_mu()?,
            OrbitalElementClassification::default(),
        );
        assert!(result.is_err());
        Ok(())
    }

    #[test]
    fn angles_are_normalized_to_positive_turn() -> TestResult {
        let angle = Angle::new(-0.25);

        assert_close(angle.radians(), 2.0 * std::f64::consts::PI - 0.25, EPS);
        Ok(())
    }

    #[test]
    fn orbital_phase_angle_returns_correct_angle_per_variant() -> TestResult {
        let sma = Distance::new(8_000_000.0);
        let ecc = Eccentricity::new(0.1);
        let inc = Angle::new(0.4);
        let raan = Angle::new(1.0);
        let aop = Angle::new(0.7);

        let test_cases: [(&str, OrbitalElements, f64); 4] = [
            (
                "Classical → true anomaly",
                OrbitalElements::Classical(ClassicalOrbitalElements::new(
                    sma,
                    ecc,
                    inc,
                    raan,
                    aop,
                    Angle::new(1.2),
                )),
                1.2,
            ),
            (
                "CircularInclined → argument of latitude",
                OrbitalElements::CircularInclined(CircularInclinedElements::new(
                    sma,
                    Eccentricity::new(0.0),
                    inc,
                    raan,
                    Angle::new(2.3),
                )),
                2.3,
            ),
            (
                "CircularEquatorial → true longitude",
                OrbitalElements::CircularEquatorial(CircularEquatorialElements::new(
                    sma,
                    Eccentricity::new(0.0),
                    Angle::ZERO,
                    Angle::new(3.4),
                )),
                3.4,
            ),
            (
                "EquatorialElliptic → true anomaly",
                OrbitalElements::EquatorialElliptic(EquatorialEllipticElements::new(
                    sma,
                    ecc,
                    Angle::ZERO,
                    aop,
                    Angle::new(4.5),
                )),
                4.5,
            ),
        ];

        for (label, elements, expected_rad) in &test_cases {
            let phase = elements.orbital_phase_angle();
            assert_close(phase.radians(), *expected_rad, EPS);
            // Verify the label uniquely identifies the variant under test.
            eprintln!("{label}: {expected_rad} rad → {:.6} rad", phase.radians());
        }
        Ok(())
    }
}
