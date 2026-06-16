use std::ops::{Add, Sub};

use crate::errors::NumericalError;
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
    /// Non-finite inputs are rejected before wrapping, since `rem_euclid` would
    /// otherwise yield `NaN`.
    pub fn from_radians(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "angle" });
        }
        Ok(Self::wrapped(value))
    }

    /// Folds a finite radian value into `[0, 2π)` without re-validating finiteness.
    ///
    /// This is the shared wrapping primitive behind `from_radians` and `Angle`
    /// arithmetic. Callers must pass a finite value: `from_radians` and
    /// `from_degrees` validate before calling, and `Angle` arithmetic operates on
    /// operands already in `[0, 2π)`, whose sum and difference are provably finite.
    #[inline]
    fn wrapped(value: f64) -> Self {
        Self(value.rem_euclid(std::f64::consts::TAU))
    }

    /// Creates an angle from degrees, wrapped into `[0, 2π)`.
    ///
    /// Wrapping happens in the degree domain before conversion, so round inputs
    /// like `360`, `450`, or `-90` fold exactly (to `0°`, `90°`, `270°`) and
    /// avoid the double-rounding that wrapping after `to_radians` would cause.
    pub fn from_degrees(degrees: f64) -> Result<Self, NumericalError> {
        if !degrees.is_finite() {
            return Err(NumericalError::NotFinite { field: "angle" });
        }
        // Wrap in the degree domain first to avoid double rounding. The result
        // already lies in `[0, 2π)`; `wrapped`'s own fold is a finite-input
        // backstop and a no-op for values produced here.
        Ok(Self::wrapped(degrees.rem_euclid(360.0).to_radians()))
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
    pub fn from_radians_per_second(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "angular rate",
            });
        }
        Ok(Self(value))
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
    pub fn from_seconds(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "epoch" });
        }
        Ok(Self(value))
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
    pub fn from_seconds(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "time step" });
        }
        if value <= 0.0 {
            return Err(NumericalError::NonPositive { field: "time step" });
        }
        Ok(Self(value))
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
    pub fn from_seconds(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "duration" });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative { field: "duration" });
        }
        Ok(Self(value))
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
    /// `NumericalError::Negative`. The difference of two finite non-negative
    /// values cannot overflow, so finiteness is a debug-only backstop.
    #[inline]
    pub fn checked_sub(self, rhs: Duration) -> Result<Duration, NumericalError> {
        let seconds = self.0 - rhs.0;
        debug_assert!(seconds.is_finite());
        if seconds < 0.0 {
            return Err(NumericalError::Negative { field: "duration" });
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
    pub fn from_meters(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "distance" });
        }
        if value <= 0.0 {
            return Err(NumericalError::NonPositive { field: "distance" });
        }
        Ok(Self(value))
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
    pub fn from_kilograms_per_cubic_meter(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "mass density",
            });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative {
                field: "mass density",
            });
        }
        Ok(Self(value))
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
    pub fn from_square_meters_per_kilogram(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "area per mass",
            });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative {
                field: "area per mass",
            });
        }
        Ok(Self(value))
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
    pub fn from_meters_per_second(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite { field: "speed" });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative { field: "speed" });
        }
        Ok(Self(value))
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
    /// Earth's standard gravitational parameter (3.986004418e14 m³/s²).
    ///
    /// Convenience constructor.  Equivalent to
    /// `GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)`
    /// but infallible because the constant is trivially valid.
    #[inline]
    pub const fn earth() -> Self {
        Self(crate::physics::constants::MU_EARTH)
    }

    /// Creates a positive gravitational parameter from cubic meters per second squared.
    pub fn from_meters_cubed_per_second_squared(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "gravitational parameter",
            });
        }
        if value <= 0.0 {
            return Err(NumericalError::NonPositive {
                field: "gravitational parameter",
            });
        }
        Ok(Self(value))
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
    pub fn from_scalar(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "j2 coefficient",
            });
        }
        Ok(Self(value))
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
    pub fn from_scalar(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "eccentricity",
            });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative {
                field: "eccentricity",
            });
        }
        Ok(Self(value))
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
    pub fn from_vector(value: Vector3<f64>) -> Result<Self, NumericalError> {
        if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite { field: "position" })
        }
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
    pub fn from_vector(value: Vector3<f64>) -> Result<Self, NumericalError> {
        if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite { field: "velocity" })
        }
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
    pub fn from_vector(value: Vector3<f64>) -> Result<Self, NumericalError> {
        if value.x.is_finite() && value.y.is_finite() && value.z.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite {
                field: "acceleration",
            })
        }
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

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-12;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn epoch_advances_by_time_step() -> TestResult {
        let epoch = Epoch::from_seconds(10.0)?;
        let dt = TimeStep::from_seconds(2.5)?;

        assert_close(epoch.advance_by(dt).seconds(), 12.5, EPS);
        Ok(())
    }

    #[test]
    fn epoch_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(Epoch::from_seconds(value).is_ok());
        }
    }

    #[test]
    fn epoch_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Epoch::from_seconds(value).is_err());
        }
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "is_finite")]
    fn epoch_advance_panics_on_nonfinite_result() {
        let epoch = Epoch::from_seconds(f64::MAX).expect("finite epoch");
        let dt = TimeStep::from_seconds(f64::MAX).expect("positive finite step");

        let _ = epoch.advance_by(dt);
    }

    #[test]
    fn epoch_duration_since_returns_elapsed_time() -> TestResult {
        let start = Epoch::from_seconds(5.0)?;
        let end = Epoch::from_seconds(12.5)?;
        assert_close(end.duration_since(start).seconds(), 7.5, EPS);
        Ok(())
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "self >= earlier")]
    fn epoch_duration_since_panics_when_earlier_is_later() {
        let start = Epoch::from_seconds(5.0).expect("finite epoch");
        let end = Epoch::from_seconds(12.5).expect("finite epoch");

        let _ = start.duration_since(end);
    }

    #[test]
    fn time_step_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(TimeStep::from_seconds(value).is_ok());
        }
    }

    #[test]
    fn time_step_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(TimeStep::from_seconds(value).is_err());
        }
    }

    #[test]
    fn duration_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            assert!(Duration::from_seconds(value).is_ok());
        }
    }

    #[test]
    fn duration_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Duration::from_seconds(value).is_err());
        }
    }

    #[test]
    fn duration_can_be_created_from_time_step() -> TestResult {
        let dt = TimeStep::from_seconds(2.5)?;
        let duration = Duration::from(dt);

        assert_close(duration.seconds(), 2.5, EPS);
        Ok(())
    }

    #[test]
    fn distance_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(Distance::from_meters(value).is_ok());
        }
    }

    #[test]
    fn distance_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Distance::from_meters(value).is_err());
        }
    }

    #[test]
    fn mass_density_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            assert!(MassDensity::from_kilograms_per_cubic_meter(value).is_ok());
        }
    }

    #[test]
    fn mass_density_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(MassDensity::from_kilograms_per_cubic_meter(value).is_err());
        }
    }

    #[test]
    fn area_per_mass_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            assert!(AreaPerMass::from_square_meters_per_kilogram(value).is_ok());
        }
    }

    #[test]
    fn area_per_mass_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(AreaPerMass::from_square_meters_per_kilogram(value).is_err());
        }
    }

    #[test]
    fn speed_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            assert!(Speed::from_meters_per_second(value).is_ok());
        }
    }

    #[test]
    fn speed_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Speed::from_meters_per_second(value).is_err());
        }
    }

    #[test]
    fn gravitational_parameter_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(GravitationalParameter::from_meters_cubed_per_second_squared(value).is_ok());
        }
    }

    #[test]
    fn gravitational_parameter_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(GravitationalParameter::from_meters_cubed_per_second_squared(value).is_err());
        }
    }

    #[test]
    fn angle_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(Angle::from_radians(value).is_ok());
        }
    }

    #[test]
    fn angle_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Angle::from_radians(value).is_err());
        }
    }

    #[test]
    fn angle_from_degrees_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Angle::from_degrees(value).is_err());
        }
    }

    #[test]
    fn angle_from_degrees_folds_round_inputs_exactly() -> TestResult {
        // Wrapping in the degree domain avoids double rounding, so full turns
        // land on exactly 0 rather than a TAU-epsilon residue.
        assert_eq!(Angle::from_degrees(360.0)?.radians(), 0.0);
        assert_eq!(Angle::from_degrees(720.0)?.radians(), 0.0);
        assert_eq!(Angle::from_degrees(450.0)?.degrees(), 90.0);
        assert_eq!(Angle::from_degrees(-90.0)?.degrees(), 270.0);
        Ok(())
    }

    #[test]
    fn angular_rate_converts_to_degrees_per_day() -> TestResult {
        let rate = AngularRate::from_radians_per_second(std::f64::consts::PI / 86_400.0)?;

        assert_close(
            rate.radians_per_second(),
            std::f64::consts::PI / 86_400.0,
            EPS,
        );
        assert_close(rate.degrees_per_day(), 180.0, EPS);
        Ok(())
    }

    #[test]
    fn angular_rate_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(AngularRate::from_radians_per_second(value).is_err());
        }
    }

    #[test]
    fn angular_rate_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(AngularRate::from_radians_per_second(value).is_ok());
        }
    }

    #[test]
    fn position_velocity_and_acceleration_round_trip_finite_vectors() -> TestResult {
        let vector = Vector3::new(1.0, -2.0, 3.0);

        assert_eq!(Position::from_vector(vector)?.vector(), vector);
        assert_eq!(Velocity::from_vector(vector)?.vector(), vector);
        assert_eq!(Acceleration::from_vector(vector)?.vector(), vector);
        Ok(())
    }

    #[test]
    fn position_velocity_and_acceleration_reject_nonfinite_components() {
        for vector in [
            Vector3::new(f64::NAN, 0.0, 0.0),
            Vector3::new(0.0, f64::INFINITY, 0.0),
            Vector3::new(0.0, 0.0, f64::NEG_INFINITY),
        ] {
            assert!(Position::from_vector(vector).is_err());
            assert!(Velocity::from_vector(vector).is_err());
            assert!(Acceleration::from_vector(vector).is_err());
        }
    }

    #[test]
    fn eccentricity_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            assert!(Eccentricity::from_scalar(value).is_ok());
        }
    }

    #[test]
    fn eccentricity_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Eccentricity::from_scalar(value).is_err());
        }
    }

    #[test]
    fn j2_coefficient_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(J2Coefficient::from_scalar(value).is_ok());
        }
    }

    #[test]
    fn j2_coefficient_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(J2Coefficient::from_scalar(value).is_err());
        }
    }

    // ── operator tests ──────────────────────────────────────────

    #[test]
    fn angle_add_and_subtract() -> TestResult {
        let a = Angle::from_radians(1.0)?;
        let b = Angle::from_radians(0.5)?;
        assert_close((a + b).radians(), 1.5, EPS);
        assert_close((a - b).radians(), 0.5, EPS);
        Ok(())
    }

    #[test]
    fn angle_construction_wraps_into_positive_turn() -> TestResult {
        let tau = std::f64::consts::TAU;

        assert_close(Angle::from_radians(-0.25)?.radians(), tau - 0.25, EPS);
        assert_close(Angle::from_radians(tau + 0.25)?.radians(), 0.25, EPS);
        assert_close(Angle::from_degrees(-90.0)?.degrees(), 270.0, EPS);
        Ok(())
    }

    #[test]
    fn angle_arithmetic_wraps_into_positive_turn() -> TestResult {
        let tau = std::f64::consts::TAU;

        // Crossing a full turn folds back to the start of the interval.
        let near_turn = Angle::from_radians(tau - 0.5)?;
        let step = Angle::from_radians(1.0)?;
        assert_close((near_turn + step).radians(), 0.5, EPS);

        // A negative raw difference is reported as its positive-turn equivalent.
        let small = Angle::from_radians(0.25)?;
        let large = Angle::from_radians(1.0)?;
        assert_close((small - large).radians(), tau - 0.75, EPS);
        Ok(())
    }

    #[test]
    fn time_step_add_and_ratio() -> TestResult {
        let a = TimeStep::from_seconds(10.0)?;
        let b = TimeStep::from_seconds(5.0)?;
        assert_close((a + b).seconds(), 15.0, EPS);
        assert_close(a.seconds() / b.seconds(), 2.0, EPS);
        Ok(())
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "is_finite")]
    fn time_step_add_panics_on_nonfinite_result() {
        let a = TimeStep::from_seconds(f64::MAX).expect("positive finite step");
        let b = TimeStep::from_seconds(f64::MAX).expect("positive finite step");

        let _ = a + b;
    }

    #[test]
    fn duration_add_and_subtract() -> TestResult {
        let a = Duration::from_seconds(10.0)?;
        let b = Duration::from_seconds(3.0)?;
        assert_close((a + b).seconds(), 13.0, EPS);
        assert_close(a.checked_sub(b)?.seconds(), 7.0, EPS);
        Ok(())
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "is_finite")]
    fn duration_add_panics_on_nonfinite_result() {
        let a = Duration::from_seconds(f64::MAX).expect("non-negative finite duration");
        let b = Duration::from_seconds(f64::MAX).expect("non-negative finite duration");

        let _ = a + b;
    }

    #[test]
    fn duration_subtract_beyond_zero_fails() -> TestResult {
        let a = Duration::from_seconds(3.0)?;
        let b = Duration::from_seconds(5.0)?;
        assert!(a.checked_sub(b).is_err());
        Ok(())
    }

    #[test]
    fn distance_add_and_ratio() -> TestResult {
        let a = Distance::from_meters(100.0)?;
        let b = Distance::from_meters(50.0)?;
        assert_close((a + b).meters(), 150.0, EPS);
        assert_close(a.meters() / b.meters(), 2.0, EPS);
        Ok(())
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "is_finite")]
    fn distance_add_panics_on_nonfinite_result() {
        let a = Distance::from_meters(f64::MAX).expect("positive finite distance");
        let b = Distance::from_meters(f64::MAX).expect("positive finite distance");

        let _ = a + b;
    }

    #[test]
    fn speed_add_and_ratio() -> TestResult {
        let a = Speed::from_meters_per_second(100.0)?;
        let b = Speed::from_meters_per_second(50.0)?;
        assert_close((a + b).meters_per_second(), 150.0, EPS);
        assert_close(a.meters_per_second() / b.meters_per_second(), 2.0, EPS);
        Ok(())
    }

    #[cfg(debug_assertions)]
    #[test]
    #[should_panic(expected = "is_finite")]
    fn speed_add_panics_on_nonfinite_result() {
        let a = Speed::from_meters_per_second(f64::MAX).expect("non-negative finite speed");
        let b = Speed::from_meters_per_second(f64::MAX).expect("non-negative finite speed");

        let _ = a + b;
    }

    #[test]
    fn gravitational_parameter_ratio() -> TestResult {
        let mu1 = GravitationalParameter::from_meters_cubed_per_second_squared(1.0e14)?;
        let mu2 = GravitationalParameter::from_meters_cubed_per_second_squared(2.0e14)?;
        assert_close(
            mu1.meters_cubed_per_second_squared() / mu2.meters_cubed_per_second_squared(),
            0.5,
            EPS,
        );
        Ok(())
    }
}
