use std::ops::{Add, Sub};

use crate::errors::NumericalError;
use nalgebra::Vector3;

/// Angle, measured in radians.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Angle(f64);

impl Angle {
    /// Zero radians.
    pub const ZERO: Self = Self(0.0);
    pub(crate) const DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE: Self =
        Self(crate::physics::constants::DEFAULT_EQUATORIAL_INCLINATION_TOLERANCE_RAD);

    /// Creates an angle from radians.
    pub fn from_radians(value: f64) -> Result<Self, NumericalError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite { field: "angle" })
        }
    }

    /// Creates an angle from degrees.
    pub fn from_degrees(degrees: f64) -> Result<Self, NumericalError> {
        Self::from_radians(degrees.to_radians())
    }

    /// Returns this angle in radians.
    #[inline]
    pub const fn radians(self) -> f64 {
        self.0
    }
}

impl Add for Angle {
    type Output = Result<Angle, NumericalError>;

    fn add(self, rhs: Angle) -> Result<Angle, NumericalError> {
        Angle::from_radians(self.0 + rhs.0)
    }
}

impl Sub for Angle {
    type Output = Result<Angle, NumericalError>;

    fn sub(self, rhs: Angle) -> Result<Angle, NumericalError> {
        Angle::from_radians(self.0 - rhs.0)
    }
}

/// Angular rate, measured in radians per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct AngularRate(f64);

impl AngularRate {
    /// Creates an angular rate from radians per second.
    pub fn from_radians_per_second(value: f64) -> Result<Self, NumericalError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite {
                field: "angular rate",
            })
        }
    }

    /// Returns this angular rate in radians per second.
    #[inline]
    pub const fn radians_per_second(self) -> f64 {
        self.0
    }

    /// Returns this angular rate in degrees per day.
    #[inline]
    pub fn degrees_per_day(self) -> f64 {
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
    /// Creates an epoch from seconds since the simulation epoch.
    pub fn from_seconds(value: f64) -> Result<Self, NumericalError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite { field: "epoch" })
        }
    }

    /// Advances this epoch by a positive time step.
    pub fn advance_by(self, dt: TimeStep) -> Result<Self, NumericalError> {
        Self::from_seconds(self.seconds() + dt.seconds())
    }

    /// Returns the elapsed `Duration` from an earlier epoch.
    ///
    /// Callers must ensure `earlier <= self` — forward-only propagation
    /// guarantees this, and debug builds will catch violations.
    #[must_use]
    pub fn duration_since(self, earlier: Epoch) -> Duration {
        debug_assert!(self >= earlier);
        let seconds = self.seconds() - earlier.seconds();
        debug_assert!(seconds.is_finite());
        debug_assert!(seconds >= 0.0);
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
    type Output = Result<TimeStep, NumericalError>;

    fn add(self, rhs: TimeStep) -> Result<TimeStep, NumericalError> {
        TimeStep::from_seconds(self.0 + rhs.0)
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
}

impl From<TimeStep> for Duration {
    fn from(value: TimeStep) -> Self {
        Self(value.seconds())
    }
}

impl Add for Duration {
    type Output = Result<Duration, NumericalError>;

    fn add(self, rhs: Duration) -> Result<Duration, NumericalError> {
        Duration::from_seconds(self.0 + rhs.0)
    }
}

impl Sub for Duration {
    type Output = Result<Duration, NumericalError>;

    fn sub(self, rhs: Duration) -> Result<Duration, NumericalError> {
        Duration::from_seconds(self.0 - rhs.0)
    }
}

/// Scalar distance, measured in meters.
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
    type Output = Result<Distance, NumericalError>;

    fn add(self, rhs: Distance) -> Result<Distance, NumericalError> {
        Distance::from_meters(self.0 + rhs.0)
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
    type Output = Result<Speed, NumericalError>;

    fn add(self, rhs: Speed) -> Result<Speed, NumericalError> {
        Speed::from_meters_per_second(self.0 + rhs.0)
    }
}

/// Standard gravitational parameter, measured in cubic meters per second squared.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct GravitationalParameter(f64);

impl GravitationalParameter {
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
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite { field: "j2" })
        }
    }

    /// Returns this J2 coefficient as a dimensionless scalar.
    #[inline]
    pub const fn scalar(self) -> f64 {
        self.0
    }
}

/// Orbital eccentricity, dimensionless.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, PartialOrd)]
pub struct Eccentricity(f64);

impl Eccentricity {
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
        if vector_is_finite(value) {
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
        if vector_is_finite(value) {
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
        if vector_is_finite(value) {
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

fn vector_is_finite(value: Vector3<f64>) -> bool {
    value.x.is_finite() && value.y.is_finite() && value.z.is_finite()
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

        assert_close(epoch.advance_by(dt)?.seconds(), 12.5, EPS);
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

    #[test]
    fn epoch_advance_rejects_nonfinite_result() -> TestResult {
        let epoch = Epoch::from_seconds(f64::MAX)?;
        let dt = TimeStep::from_seconds(f64::MAX)?;

        assert!(epoch.advance_by(dt).is_err());
        Ok(())
    }

    #[test]
    fn epoch_duration_since_returns_elapsed_time() -> TestResult {
        let start = Epoch::from_seconds(5.0)?;
        let end = Epoch::from_seconds(12.5)?;
        assert_close(end.duration_since(start).seconds(), 7.5, EPS);
        Ok(())
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
        assert_close((a + b)?.radians(), 1.5, EPS);
        assert_close((a - b)?.radians(), 0.5, EPS);
        Ok(())
    }

    #[test]
    fn angle_arithmetic_rejects_nonfinite_results() -> TestResult {
        let a = Angle::from_radians(f64::MAX)?;
        let b = Angle::from_radians(f64::MAX)?;

        assert!((a + b).is_err());
        assert!((Angle::from_radians(-f64::MAX)? - b).is_err());
        Ok(())
    }

    #[test]
    fn time_step_add_and_ratio() -> TestResult {
        let a = TimeStep::from_seconds(10.0)?;
        let b = TimeStep::from_seconds(5.0)?;
        assert_close((a + b)?.seconds(), 15.0, EPS);
        assert_close(a.seconds() / b.seconds(), 2.0, EPS);
        Ok(())
    }

    #[test]
    fn time_step_add_rejects_nonfinite_result() -> TestResult {
        let a = TimeStep::from_seconds(f64::MAX)?;
        let b = TimeStep::from_seconds(f64::MAX)?;

        assert!((a + b).is_err());
        Ok(())
    }

    #[test]
    fn duration_add_and_subtract() -> TestResult {
        let a = Duration::from_seconds(10.0)?;
        let b = Duration::from_seconds(3.0)?;
        assert_close((a + b)?.seconds(), 13.0, EPS);
        assert_close((a - b).unwrap().seconds(), 7.0, EPS);
        Ok(())
    }

    #[test]
    fn duration_add_rejects_nonfinite_result() -> TestResult {
        let a = Duration::from_seconds(f64::MAX)?;
        let b = Duration::from_seconds(f64::MAX)?;

        assert!((a + b).is_err());
        Ok(())
    }

    #[test]
    fn duration_subtract_beyond_zero_fails() -> TestResult {
        let a = Duration::from_seconds(3.0)?;
        let b = Duration::from_seconds(5.0)?;
        assert!((a - b).is_err());
        Ok(())
    }

    #[test]
    fn distance_add_and_ratio() -> TestResult {
        let a = Distance::from_meters(100.0)?;
        let b = Distance::from_meters(50.0)?;
        assert_close((a + b)?.meters(), 150.0, EPS);
        assert_close(a.meters() / b.meters(), 2.0, EPS);
        Ok(())
    }

    #[test]
    fn distance_add_rejects_nonfinite_result() -> TestResult {
        let a = Distance::from_meters(f64::MAX)?;
        let b = Distance::from_meters(f64::MAX)?;

        assert!((a + b).is_err());
        Ok(())
    }

    #[test]
    fn speed_add_and_ratio() -> TestResult {
        let a = Speed::from_meters_per_second(100.0)?;
        let b = Speed::from_meters_per_second(50.0)?;
        assert_close((a + b)?.meters_per_second(), 150.0, EPS);
        assert_close(a.meters_per_second() / b.meters_per_second(), 2.0, EPS);
        Ok(())
    }

    #[test]
    fn speed_add_rejects_nonfinite_result() -> TestResult {
        let a = Speed::from_meters_per_second(f64::MAX)?;
        let b = Speed::from_meters_per_second(f64::MAX)?;

        assert!((a + b).is_err());
        Ok(())
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
