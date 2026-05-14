use crate::errors::DynamicsError;
pub use crate::state::CartesianState;
pub use crate::types::{Acceleration, Epoch, GravitationalParameter};

/// Point-mass gravity model for a single central body.
pub struct TwoBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    pub mu: GravitationalParameter,
}

/// Computes acceleration for a state at a given simulation time.
pub trait Dynamics {
    /// Returns the acceleration applied to `state` at epoch `t`.
    fn acceleration(&self, state: &CartesianState, t: Epoch)
    -> Result<Acceleration, DynamicsError>;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration(
        &self,
        state: &CartesianState,
        _t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let r = state.position.vector();
        let radius_m = r.norm();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        Acceleration::from_vector(-self.mu.meters_cubed_per_second_squared() * r / radius_m.powi(3))
            .ok_or(DynamicsError::InvalidAcceleration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, R_EARTH};
    use crate::types::{Position, Velocity};
    use nalgebra::Vector3;

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

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
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let state =
            state_at(Vector3::new(radius, 0.0, 0.0), Vector3::zeros()).ok_or("invalid state")?;

        let acceleration = gravity
            .acceleration(&state, Epoch::from_seconds(0.0).ok_or("invalid epoch")?)
            .map_err(|_| "invalid acceleration")?
            .vector();

        assert!(acceleration.x < 0.0);
        assert_close(acceleration.y, 0.0, EPS);
        assert_close(acceleration.z, 0.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_has_inverse_square_magnitude() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let state =
            state_at(Vector3::new(radius, 0.0, 0.0), Vector3::zeros()).ok_or("invalid state")?;

        let acceleration = gravity
            .acceleration(&state, Epoch::from_seconds(0.0).ok_or("invalid epoch")?)
            .map_err(|_| "invalid acceleration")?
            .vector();

        assert_close(acceleration.norm(), MU_EARTH / radius.powi(2), EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_falls_off_with_inverse_square_distance() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let near_state =
            state_at(Vector3::new(radius, 0.0, 0.0), Vector3::zeros()).ok_or("invalid state")?;
        let far_state = state_at(Vector3::new(2.0 * radius, 0.0, 0.0), Vector3::zeros())
            .ok_or("invalid state")?;

        let epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;
        let near_acceleration = gravity
            .acceleration(&near_state, epoch)
            .map_err(|_| "invalid near acceleration")?
            .vector()
            .norm();
        let far_acceleration = gravity
            .acceleration(&far_state, epoch)
            .map_err(|_| "invalid far acceleration")?
            .vector()
            .norm();

        assert_close(far_acceleration, near_acceleration / 4.0, EPS);
        Ok(())
    }

    #[test]
    fn position_rejects_nan_components() {
        assert!(Position::from_vector(Vector3::new(f64::NAN, 0.0, 0.0)).is_none());
    }

    #[test]
    fn gravitational_parameter_rejects_zero() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(0.0).is_none());
    }

    #[test]
    fn gravitational_parameter_rejects_nan() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(f64::NAN).is_none());
    }

    #[test]
    fn two_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        let state = state_at(Vector3::zeros(), Vector3::zeros()).ok_or("invalid state")?;

        assert!(matches!(
            gravity.acceleration(&state, Epoch::from_seconds(0.0).ok_or("invalid epoch")?),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }
}
