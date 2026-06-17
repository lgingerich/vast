use std::ops::{Add, Sub};

use nalgebra::Vector3;

/// Angle in radians, wrapped to the half-open turn `[0, 2π)`.
///
/// Every constructor folds its input into `[0, 2π)`, so arithmetic wraps too and
/// negative inputs become their positive-turn equivalent. Inclination's physical
/// range `[0, π]` is already inside this interval, so wrapping never alters it.
/// Code needing a signed difference in `(-π, π]` must derive it from the radian
/// value; `Angle` itself never reads back negative.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
    /// Zero radians.
    pub const ZERO: Self = Self(0.0);
    pub(crate) const DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE: Self =
        Self(crate::physics::constants::DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD);

    /// Creates an angle from radians, wrapped into `[0, 2π)`.
    ///
    /// Panics in debug builds if the input is not finite.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "angle must be finite");
        Self::wrapped(value)
    }

    /// Folds a finite radian value into `[0, 2π)` without re-validating finiteness.
    ///
    /// This is the shared wrapping primitive behind `from_radians` and `Angle`
    /// arithmetic. Callers must pass a finite value.
    #[inline]
    fn wrapped(value: f64) -> Self {
        Self(value.rem_euclid(std::f64::consts::TAU))
    }

    /// Creates an angle from degrees, wrapped into `[0, 2π)`.
    ///
    /// Wrapping happens in the degree domain before conversion, so round inputs
    /// like `360`, `450`, or `-90` fold exactly (to `0°`, `90°`, `270°`) and
    /// avoid the double-rounding that wrapping after `to_radians` would cause.
    ///
    /// Panics in debug builds if the input is not finite.
    pub fn from_degrees(degrees: f64) -> Self {
        debug_assert!(degrees.is_finite(), "angle must be finite");
        // Wrap in the degree domain first to avoid double rounding. The result
        // already lies in `[0, 2π)`; `wrapped`'s own fold is a finite-input
        // backstop and a no-op for values produced here.
        Self::wrapped(degrees.rem_euclid(360.0).to_radians())
    }

    /// Returns this angle in radians, in `[0, 2π)`.
    #[inline]
    pub const fn radians(self) -> f64 {
        self.0
    }

    /// Returns this angle in degrees, in `[0, 360)`.
    #[inline]
    pub const fn degrees(self) -> f64 {
        self.0.to_degrees()
    }
}

impl Add for Angle {
    type Output = Angle;

    /// Adds two angles, wrapping the result back into `[0, 2π)`.
    ///
    /// Total: both operands lie in `[0, 2π)`, so the sum is finite and `wrapped`
    /// always yields a valid angle.
    #[inline]
    fn add(self, rhs: Angle) -> Angle {
        Angle::wrapped(self.0 + rhs.0)
    }
}

impl Sub for Angle {
    type Output = Angle;

    /// Subtracts two angles, wrapping the result back into `[0, 2π)`.
    ///
    /// Total: both operands lie in `[0, 2π)`, so the difference is finite and
    /// `wrapped` always yields a valid angle.
    #[inline]
    fn sub(self, rhs: Angle) -> Angle {
        Angle::wrapped(self.0 - rhs.0)
    }
}

/// Angular rate, measured in radians per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct AngularRate(f64);

impl AngularRate {
    /// Creates an angular rate from radians per second.
    ///
    /// Panics in debug builds if the input is not finite.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "angular rate must be finite");
        Self(value)
    }

    /// Returns this angular rate in radians per second.
    #[inline]
    pub const fn radians_per_second(self) -> f64 {
        self.0
    }

    /// Returns this angular rate in degrees per day.
    #[inline]
    pub const fn degrees_per_day(self) -> f64 {
        self.0.to_degrees() * 86_400.0
    }
}

/// Instant in simulation time, measured in seconds from the simulation epoch.
///
/// `Epoch` represents an absolute point on the simulation timeline.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Epoch(f64);

impl Epoch {
    /// Epoch zero — the start of the simulation timeline.
    pub const ZERO: Self = Self(0.0);

    /// Creates an epoch from seconds since the simulation epoch.
    ///
    /// Panics in debug builds if the input is not finite.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "epoch must be finite");
        Self(value)
    }

    /// Advances this epoch by a positive time step.
    ///
    /// Infallible: both operands are finite, so the only way the sum could be
    /// non-finite is `f64` overflow (~1e300 s past the epoch), which is a
    /// programming error rather than a recoverable runtime condition. A debug
    /// assertion guards that boundary without burdening the step loop.
    #[inline]
    #[must_use]
    pub fn advance_by(self, dt: TimeStep) -> Self {
        let seconds = self.seconds() + dt.seconds();
        debug_assert!(seconds.is_finite());
        Self(seconds)
    }

    /// Returns the elapsed `Duration` from an earlier epoch.
    ///
    /// Callers must ensure `earlier <= self` — forward-only propagation
    /// guarantees this, and debug builds will catch violations.
    #[inline]
    #[must_use]
    pub fn duration_since(self, earlier: Epoch) -> Duration {
        debug_assert!(self >= earlier);
        let seconds = self.seconds() - earlier.seconds();
        // `self >= earlier` already implies a non-negative difference; the only
        // way this construction can break `Duration`'s invariant is `f64`
        // overflow when the epochs span the full representable range.
        debug_assert!(seconds.is_finite());
        Duration(seconds)
    }

    /// Returns seconds since the simulation epoch.
    #[inline]
    pub const fn seconds(self) -> f64 {
        self.0
    }
}

/// Positive integration step size, measured in seconds.
///
/// Use `TimeStep` when advancing a simulation state forward in time.
///
/// Only `Add` is provided: subtracting two steps can be non-positive, which
/// would break the positivity invariant, so there is no `Sub`.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct TimeStep(f64);

impl TimeStep {
    /// Creates a positive time step from seconds.
    ///
    /// Panics in debug builds if the input is not finite or not positive.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "time step must be finite");
        debug_assert!(value > 0.0, "time step must be positive");
        Self(value)
    }

    /// Returns this time step in seconds.
    #[inline]
    pub const fn seconds(self) -> f64 {
        self.0
    }
}

impl Add for TimeStep {
    type Output = TimeStep;

    /// Adds two time steps.
    ///
    /// Infallible: the sum of two positive values is positive, so the only
    /// possible failure is `f64` overflow to `+inf`, which a debug assertion
    /// guards as a programming error.
    #[inline]
    fn add(self, rhs: TimeStep) -> TimeStep {
        let seconds = self.0 + rhs.0;
        debug_assert!(seconds.is_finite());
        Self(seconds)
    }
}

/// Non-negative elapsed simulation time, measured in seconds.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Duration(f64);

impl Duration {
    /// Creates a non-negative duration from seconds.
    ///
    /// Panics in debug builds if the input is not finite or negative.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "duration must be finite");
        debug_assert!(value >= 0.0, "duration must be non-negative");
        Self(value)
    }

    /// Returns this duration in seconds.
    #[inline]
    pub const fn seconds(self) -> f64 {
        self.0
    }

    /// Subtracts `rhs`, returning the remaining duration.
    ///
    /// Fallible because subtracting a larger duration is a legitimate,
    /// input-dependent outcome the caller may need to handle; it returns
    /// an error in that case. The difference of two finite non-negative
    /// values cannot overflow, so finiteness is a debug-only backstop.
    #[inline]
    pub fn checked_sub(self, rhs: Duration) -> Result<Duration, crate::errors::NumericalError> {
        let seconds = self.0 - rhs.0;
        debug_assert!(seconds.is_finite());
        if seconds < 0.0 {
            return Err(crate::errors::NumericalError::Negative { field: "duration" });
        }
        Ok(Self(seconds))
    }
}

impl From<TimeStep> for Duration {
    #[inline]
    fn from(value: TimeStep) -> Self {
        Self(value.seconds())
    }
}

impl Add for Duration {
    type Output = Duration;

    /// Adds two durations.
    ///
    /// Infallible: the sum of two non-negative values is non-negative, so the
    /// only possible failure is `f64` overflow to `+inf`, which a debug
    /// assertion guards as a programming error.
    #[inline]
    fn add(self, rhs: Duration) -> Duration {
        let seconds = self.0 + rhs.0;
        debug_assert!(seconds.is_finite());
        Self(seconds)
    }
}

/// Scalar distance, measured in meters.
///
/// Only `Add` is provided: subtracting two distances can be non-positive, which
/// would break the positivity invariant, so there is no `Sub`.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Distance(f64);

impl Distance {
    /// Creates a positive distance from meters.
    ///
    /// Panics in debug builds if the input is not finite or not positive.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "distance must be finite");
        debug_assert!(value > 0.0, "distance must be positive");
        Self(value)
    }

    /// Returns this distance in meters.
    #[inline]
    pub const fn meters(self) -> f64 {
        self.0
    }
}

impl Add for Distance {
    type Output = Distance;

    /// Adds two distances.
    ///
    /// Infallible: the sum of two positive values is positive, so the only
    /// possible failure is `f64` overflow to `+inf`, which a debug assertion
    /// guards as a programming error.
    #[inline]
    fn add(self, rhs: Distance) -> Distance {
        let meters = self.0 + rhs.0;
        debug_assert!(meters.is_finite());
        Self(meters)
    }
}

/// Mass density, measured in kilograms per cubic meter.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct MassDensity(f64);

impl MassDensity {
    /// Creates a non-negative mass density from kilograms per cubic meter.
    ///
    /// Panics in debug builds if the input is not finite or negative.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "mass density must be finite");
        debug_assert!(value >= 0.0, "mass density must be non-negative");
        Self(value)
    }

    /// Returns this mass density in kilograms per cubic meter.
    #[inline]
    pub const fn kilograms_per_cubic_meter(self) -> f64 {
        self.0
    }
}

/// Area per mass, measured in square meters per kilogram.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct AreaPerMass(f64);

impl AreaPerMass {
    /// Creates a non-negative area per mass from square meters per kilogram.
    ///
    /// Panics in debug builds if the input is not finite or negative.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "area per mass must be finite");
        debug_assert!(value >= 0.0, "area per mass must be non-negative");
        Self(value)
    }

    /// Returns this area per mass in square meters per kilogram.
    #[inline]
    pub const fn square_meters_per_kilogram(self) -> f64 {
        self.0
    }
}

/// Scalar speed, measured in meters per second.
///
/// Only `Add` is provided: subtracting two speeds can be negative, which would
/// break the non-negativity invariant, so there is no `Sub`.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Speed(f64);

impl Speed {
    /// Creates a non-negative speed from meters per second.
    ///
    /// Panics in debug builds if the input is not finite or negative.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "speed must be finite");
        debug_assert!(value >= 0.0, "speed must be non-negative");
        Self(value)
    }

    /// Returns this speed in meters per second.
    #[inline]
    pub const fn meters_per_second(self) -> f64 {
        self.0
    }
}

impl Add for Speed {
    type Output = Speed;

    /// Adds two speeds.
    ///
    /// Infallible: the sum of two non-negative values is non-negative, so the
    /// only possible failure is `f64` overflow to `+inf`, which a debug
    /// assertion guards as a programming error.
    #[inline]
    fn add(self, rhs: Speed) -> Speed {
        let meters_per_second = self.0 + rhs.0;
        debug_assert!(meters_per_second.is_finite());
        Self(meters_per_second)
    }
}

/// Standard gravitational parameter, measured in cubic meters per second squared.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct GravitationalParameter(f64);

impl GravitationalParameter {
    /// Earth's standard gravitational parameter (3.986004418e14 m³/s²).
    ///
    /// Convenience constructor. Equivalent to
    /// `GravitationalParameter::new(MU_EARTH)` but infallible because the
    /// constant is trivially valid.
    #[inline]
    pub const fn earth() -> Self {
        Self(crate::physics::constants::MU_EARTH)
    }

    /// Creates a positive gravitational parameter from cubic meters per second squared.
    ///
    /// Panics in debug builds if the input is not finite or not positive.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "gravitational parameter must be finite");
        debug_assert!(value > 0.0, "gravitational parameter must be positive");
        Self(value)
    }

    /// Returns this gravitational parameter in cubic meters per second squared.
    #[inline]
    pub const fn meters_cubed_per_second_squared(self) -> f64 {
        self.0
    }
}

/// Dimensionless second zonal harmonic coefficient, commonly called J2.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct J2Coefficient(f64);

impl J2Coefficient {
    /// Creates a J2 coefficient from a dimensionless scalar.
    ///
    /// Panics in debug builds if the input is not finite.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "J2 coefficient must be finite");
        Self(value)
    }

    /// Returns this J2 coefficient as a dimensionless scalar.
    #[inline]
    pub const fn scalar(self) -> f64 {
        self.0
    }
}

/// Orbital eccentricity, dimensionless.
///
/// Values `>= 1` are permitted and represent open (parabolic or hyperbolic)
/// trajectories; only the lower bound `>= 0` is enforced.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Eccentricity(f64);

impl Eccentricity {
    /// Eccentricity of a perfectly circular orbit.
    pub const CIRCULAR: Self = Self(0.0);

    pub(crate) const DEFAULT_CIRCULAR_CLASSIFICATION_TOLERANCE: Self =
        Self(crate::physics::constants::DEFAULT_CIRCULAR_ECCENTRICITY_TOLERANCE);

    /// Creates an eccentricity from a dimensionless scalar.
    ///
    /// Panics in debug builds if the input is not finite or negative.
    pub fn new(value: f64) -> Self {
        debug_assert!(value.is_finite(), "eccentricity must be finite");
        debug_assert!(value >= 0.0, "eccentricity must be non-negative");
        Self(value)
    }

    /// Returns this eccentricity as a dimensionless scalar.
    #[inline]
    pub const fn scalar(self) -> f64 {
        self.0
    }
}

/// Position vector in a Cartesian frame, measured in meters.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position(Vector3<f64>);

impl Position {
    /// Creates a position from a finite Cartesian vector.
    ///
    /// Panics in debug builds if any component is not finite.
    pub fn from_vector(value: Vector3<f64>) -> Self {
        debug_assert!(
            value.x.is_finite() && value.y.is_finite() && value.z.is_finite(),
            "position components must be finite"
        );
        Self(value)
    }

    /// Returns this position as a Cartesian vector.
    #[inline]
    pub const fn vector(self) -> Vector3<f64> {
        self.0
    }
}

/// Velocity vector in a Cartesian frame, measured in meters per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Velocity(Vector3<f64>);

impl Velocity {
    /// Creates a velocity from a finite Cartesian vector.
    ///
    /// Panics in debug builds if any component is not finite.
    pub fn from_vector(value: Vector3<f64>) -> Self {
        debug_assert!(
            value.x.is_finite() && value.y.is_finite() && value.z.is_finite(),
            "velocity components must be finite"
        );
        Self(value)
    }

    /// Returns this velocity as a Cartesian vector.
    #[inline]
    pub const fn vector(self) -> Vector3<f64> {
        self.0
    }
}

/// Acceleration vector in a Cartesian frame, measured in meters per second squared.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Acceleration(Vector3<f64>);

impl Acceleration {
    /// Creates an acceleration from a finite Cartesian vector.
    ///
    /// Panics in debug builds if any component is not finite.
    pub fn from_vector(value: Vector3<f64>) -> Self {
        debug_assert!(
            value.x.is_finite() && value.y.is_finite() && value.z.is_finite(),
            "acceleration components must be finite"
        );
        Self(value)
    }

    /// Returns this acceleration as a Cartesian vector.
    #[inline]
    pub const fn vector(self) -> Vector3<f64> {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1.0e-12;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn epoch_advances_by_time_step() {
        let epoch = Epoch::new(10.0);
        let dt = TimeStep::new(2.5);

        assert_close(epoch.advance_by(dt).seconds(), 12.5, EPS);
    }

    #[test]
    fn epoch_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            let _ = Epoch::new(value);
        }
    }

    #[test]
    fn epoch_duration_since_returns_elapsed_time() {
        let start = Epoch::new(5.0);
        let end = Epoch::new(12.5);
        assert_close(end.duration_since(start).seconds(), 7.5, EPS);
    }

    #[test]
    fn time_step_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            let _ = TimeStep::new(value);
        }
    }

    #[test]
    fn duration_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            let _ = Duration::new(value);
        }
    }

    #[test]
    fn duration_can_be_created_from_time_step() {
        let dt = TimeStep::new(2.5);
        let duration = Duration::from(dt);

        assert_close(duration.seconds(), 2.5, EPS);
    }

    #[test]
    fn distance_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            let _ = Distance::new(value);
        }
    }

    #[test]
    fn mass_density_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            let _ = MassDensity::new(value);
        }
    }

    #[test]
    fn area_per_mass_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            let _ = AreaPerMass::new(value);
        }
    }

    #[test]
    fn speed_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            let _ = Speed::new(value);
        }
    }

    #[test]
    fn gravitational_parameter_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            let _ = GravitationalParameter::new(value);
        }
    }

    #[test]
    fn angle_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            let _ = Angle::new(value);
        }
    }

    #[test]
    fn angle_from_degrees_folds_round_inputs_exactly() {
        // Wrapping in the degree domain avoids double rounding, so full turns
        // land on exactly 0 rather than a TAU-epsilon residue.
        assert_eq!(Angle::from_degrees(360.0).radians(), 0.0);
        assert_eq!(Angle::from_degrees(720.0).radians(), 0.0);
        assert_eq!(Angle::from_degrees(450.0).degrees(), 90.0);
        assert_eq!(Angle::from_degrees(-90.0).degrees(), 270.0);
    }

    #[test]
    fn angular_rate_converts_to_degrees_per_day() {
        let rate = AngularRate::new(std::f64::consts::PI / 86_400.0);

        assert_close(
            rate.radians_per_second(),
            std::f64::consts::PI / 86_400.0,
            EPS,
        );
        assert_close(rate.degrees_per_day(), 180.0, EPS);
    }

    #[test]
    fn angular_rate_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            let _ = AngularRate::new(value);
        }
    }

    #[test]
    fn position_velocity_and_acceleration_round_trip_finite_vectors() {
        let vector = Vector3::new(1.0, -2.0, 3.0);

        assert_eq!(Position::from_vector(vector).vector(), vector);
        assert_eq!(Velocity::from_vector(vector).vector(), vector);
        assert_eq!(Acceleration::from_vector(vector).vector(), vector);
    }

    #[test]
    fn eccentricity_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            let _ = Eccentricity::new(value);
        }
    }

    #[test]
    fn j2_coefficient_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            let _ = J2Coefficient::new(value);
        }
    }

    // ── operator tests ──────────────────────────────────────────

    #[test]
    fn angle_add_and_subtract() {
        let a = Angle::new(1.0);
        let b = Angle::new(0.5);
        assert_close((a + b).radians(), 1.5, EPS);
        assert_close((a - b).radians(), 0.5, EPS);
    }

    #[test]
    fn angle_construction_wraps_into_positive_turn() {
        let tau = std::f64::consts::TAU;

        assert_close(Angle::new(-0.25).radians(), tau - 0.25, EPS);
        assert_close(Angle::new(tau + 0.25).radians(), 0.25, EPS);
        assert_close(Angle::from_degrees(-90.0).degrees(), 270.0, EPS);
    }

    #[test]
    fn angle_arithmetic_wraps_into_positive_turn() {
        let tau = std::f64::consts::TAU;

        // Crossing a full turn folds back to the start of the interval.
        let near_turn = Angle::new(tau - 0.5);
        let step = Angle::new(1.0);
        assert_close((near_turn + step).radians(), 0.5, EPS);

        // A negative raw difference is reported as its positive-turn equivalent.
        let small = Angle::new(0.25);
        let large = Angle::new(1.0);
        assert_close((small - large).radians(), tau - 0.75, EPS);
    }

    #[test]
    fn time_step_add_and_ratio() {
        let a = TimeStep::new(10.0);
        let b = TimeStep::new(5.0);
        assert_close((a + b).seconds(), 15.0, EPS);
        assert_close(a.seconds() / b.seconds(), 2.0, EPS);
    }

    #[test]
    fn duration_add_and_subtract() {
        let a = Duration::new(10.0);
        let b = Duration::new(3.0);
        assert_close((a + b).seconds(), 13.0, EPS);
        assert_close(a.checked_sub(b).unwrap().seconds(), 7.0, EPS);
    }

    #[test]
    fn duration_subtract_beyond_zero_fails() {
        let a = Duration::new(3.0);
        let b = Duration::new(5.0);
        assert!(a.checked_sub(b).is_err());
    }

    #[test]
    fn distance_add_and_ratio() {
        let a = Distance::new(100.0);
        let b = Distance::new(50.0);
        assert_close((a + b).meters(), 150.0, EPS);
        assert_close(a.meters() / b.meters(), 2.0, EPS);
    }

    #[test]
    fn speed_add_and_ratio() {
        let a = Speed::new(100.0);
        let b = Speed::new(50.0);
        assert_close((a + b).meters_per_second(), 150.0, EPS);
        assert_close(a.meters_per_second() / b.meters_per_second(), 2.0, EPS);
    }

    #[test]
    fn gravitational_parameter_ratio() {
        let mu1 = GravitationalParameter::new(1.0e14);
        let mu2 = GravitationalParameter::new(2.0e14);
        assert_close(
            mu1.meters_cubed_per_second_squared() / mu2.meters_cubed_per_second_squared(),
            0.5,
            EPS,
        );
    }
}
