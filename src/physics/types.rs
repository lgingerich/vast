use nalgebra::Vector3;

/// Angle, measured in radians.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct Angle(f64);

impl Angle {
    /// Zero radians.
    pub const ZERO: Self = Self(0.0);

    /// Creates an angle from radians.
    pub fn from_radians(value: f64) -> Option<Self> {
        value.is_finite().then_some(Self(value))
    }

    /// Returns this angle in radians.
    #[inline]
    pub const fn radians(self) -> f64 {
        self.0
    }
}

/// Angular rate, measured in radians per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct AngularRate(f64);

impl AngularRate {
    /// Creates an angular rate from radians per second.
    pub fn from_radians_per_second(value: f64) -> Option<Self> {
        value.is_finite().then_some(Self(value))
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
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Epoch(f64);

impl Epoch {
    /// Creates an epoch from seconds since the simulation epoch.
    pub fn from_seconds(value: f64) -> Option<Self> {
        value.is_finite().then_some(Self(value))
    }

    /// Advances this epoch by a positive time step.
    pub fn advance_by(self, dt: TimeStep) -> Option<Self> {
        Self::from_seconds(self.seconds() + dt.seconds())
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
#[derive(Debug, Clone, Copy)]
pub struct TimeStep(f64);

impl TimeStep {
    /// Creates a positive time step from seconds.
    pub fn from_seconds(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0).then_some(Self(value))
    }

    /// Returns this time step in seconds.
    #[inline]
    pub const fn seconds(self) -> f64 {
        self.0
    }
}

/// Non-negative elapsed simulation time, measured in seconds.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Duration(f64);

impl Duration {
    /// Creates a non-negative duration from seconds.
    pub fn from_seconds(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
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

/// Scalar distance, measured in meters.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct Distance(f64);

impl Distance {
    /// Creates a positive distance from meters.
    pub fn from_meters(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0).then_some(Self(value))
    }

    /// Returns this distance in meters.
    #[inline]
    pub const fn meters(self) -> f64 {
        self.0
    }
}

/// Mass density, measured in kilograms per cubic meter.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct MassDensity(f64);

impl MassDensity {
    /// Creates a non-negative mass density from kilograms per cubic meter.
    pub fn from_kilograms_per_cubic_meter(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
    }

    /// Returns this mass density in kilograms per cubic meter.
    #[inline]
    pub const fn kilograms_per_cubic_meter(self) -> f64 {
        self.0
    }
}

/// Area per mass, measured in square meters per kilogram.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct AreaPerMass(f64);

impl AreaPerMass {
    /// Creates a non-negative area per mass from square meters per kilogram.
    pub fn from_square_meters_per_kilogram(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
    }

    /// Returns this area per mass in square meters per kilogram.
    #[inline]
    pub const fn square_meters_per_kilogram(self) -> f64 {
        self.0
    }
}

/// Scalar speed, measured in meters per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct Speed(f64);

impl Speed {
    /// Creates a non-negative speed from meters per second.
    pub fn from_meters_per_second(value: f64) -> Option<Self> {
        (value.is_finite() && value >= 0.0).then_some(Self(value))
    }

    /// Returns this speed in meters per second.
    #[inline]
    pub const fn meters_per_second(self) -> f64 {
        self.0
    }
}

/// Standard gravitational parameter, measured in cubic meters per second squared.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravitationalParameter(f64);

impl GravitationalParameter {
    /// Creates a positive gravitational parameter from cubic meters per second squared.
    pub fn from_meters_cubed_per_second_squared(value: f64) -> Option<Self> {
        (value.is_finite() && value > 0.0).then_some(Self(value))
    }

    /// Returns this gravitational parameter in cubic meters per second squared.
    #[inline]
    pub const fn meters_cubed_per_second_squared(self) -> f64 {
        self.0
    }
}

/// Position vector in a Cartesian frame, measured in meters.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position(Vector3<f64>);

impl Position {
    /// Creates a position from a finite Cartesian vector.
    pub fn from_vector(value: Vector3<f64>) -> Option<Self> {
        vector_is_finite(value).then_some(Self(value))
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
    pub fn from_vector(value: Vector3<f64>) -> Option<Self> {
        vector_is_finite(value).then_some(Self(value))
    }

    /// Returns this velocity as a Cartesian vector.
    #[inline]
    pub const fn vector(self) -> Vector3<f64> {
        self.0
    }
}

/// Acceleration vector in a Cartesian frame, measured in meters per second squared.
#[repr(transparent)]
#[derive(Debug, Clone, Copy)]
pub struct Acceleration(Vector3<f64>);

impl Acceleration {
    /// Creates an acceleration from a finite Cartesian vector.
    pub fn from_vector(value: Vector3<f64>) -> Option<Self> {
        vector_is_finite(value).then_some(Self(value))
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

    const EPS: f64 = 1.0e-12;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn epoch_advances_by_time_step() -> Result<(), &'static str> {
        let epoch = Epoch::from_seconds(10.0).ok_or("invalid epoch")?;
        let dt = TimeStep::from_seconds(2.5).ok_or("invalid time step")?;

        assert_close(
            epoch.advance_by(dt).ok_or("invalid next epoch")?.seconds(),
            12.5,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn epoch_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(Epoch::from_seconds(value).is_some());
        }
    }

    #[test]
    fn epoch_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Epoch::from_seconds(value).is_none());
        }
    }

    #[test]
    fn epoch_advance_rejects_nonfinite_result() -> Result<(), &'static str> {
        let epoch = Epoch::from_seconds(f64::MAX).ok_or("invalid epoch")?;
        let dt = TimeStep::from_seconds(f64::MAX).ok_or("invalid time step")?;

        assert!(epoch.advance_by(dt).is_none());
        Ok(())
    }

    #[test]
    fn time_step_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(TimeStep::from_seconds(value).is_some());
        }
    }

    #[test]
    fn time_step_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(TimeStep::from_seconds(value).is_none());
        }
    }

    #[test]
    fn duration_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            assert!(Duration::from_seconds(value).is_some());
        }
    }

    #[test]
    fn duration_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Duration::from_seconds(value).is_none());
        }
    }

    #[test]
    fn duration_can_be_created_from_time_step() -> Result<(), &'static str> {
        let dt = TimeStep::from_seconds(2.5).ok_or("invalid time step")?;
        let duration = Duration::from(dt);

        assert_close(duration.seconds(), 2.5, EPS);
        Ok(())
    }

    #[test]
    fn distance_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(Distance::from_meters(value).is_some());
        }
    }

    #[test]
    fn distance_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Distance::from_meters(value).is_none());
        }
    }

    #[test]
    fn mass_density_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            assert!(MassDensity::from_kilograms_per_cubic_meter(value).is_some());
        }
    }

    #[test]
    fn mass_density_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(MassDensity::from_kilograms_per_cubic_meter(value).is_none());
        }
    }

    #[test]
    fn area_per_mass_accepts_nonnegative_finite_values() {
        for value in [0.0, f64::MIN_POSITIVE, 1.0] {
            assert!(AreaPerMass::from_square_meters_per_kilogram(value).is_some());
        }
    }

    #[test]
    fn area_per_mass_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(AreaPerMass::from_square_meters_per_kilogram(value).is_none());
        }
    }

    #[test]
    fn speed_accepts_nonnegative_finite_values() {
        for value in [0.0, 1.0] {
            assert!(Speed::from_meters_per_second(value).is_some());
        }
    }

    #[test]
    fn speed_rejects_negative_and_nonfinite_values() {
        for value in [-1.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Speed::from_meters_per_second(value).is_none());
        }
    }

    #[test]
    fn gravitational_parameter_accepts_positive_finite_values() {
        for value in [f64::MIN_POSITIVE, 1.0] {
            assert!(GravitationalParameter::from_meters_cubed_per_second_squared(value).is_some());
        }
    }

    #[test]
    fn gravitational_parameter_rejects_nonpositive_and_nonfinite_values() {
        for value in [-1.0, 0.0, f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(GravitationalParameter::from_meters_cubed_per_second_squared(value).is_none());
        }
    }

    #[test]
    fn angle_accepts_finite_values() {
        for value in [-1.0, 0.0, 1.0] {
            assert!(Angle::from_radians(value).is_some());
        }
    }

    #[test]
    fn angle_rejects_nonfinite_values() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(Angle::from_radians(value).is_none());
        }
    }

    #[test]
    fn angular_rate_converts_to_degrees_per_day() -> Result<(), &'static str> {
        let rate = AngularRate::from_radians_per_second(std::f64::consts::PI / 86_400.0)
            .ok_or("invalid angular rate")?;

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
            assert!(AngularRate::from_radians_per_second(value).is_none());
        }
    }

    #[test]
    fn position_velocity_and_acceleration_reject_nonfinite_components() {
        for vector in [
            Vector3::new(f64::NAN, 0.0, 0.0),
            Vector3::new(0.0, f64::INFINITY, 0.0),
            Vector3::new(0.0, 0.0, f64::NEG_INFINITY),
        ] {
            assert!(Position::from_vector(vector).is_none());
            assert!(Velocity::from_vector(vector).is_none());
            assert!(Acceleration::from_vector(vector).is_none());
        }
    }
}
