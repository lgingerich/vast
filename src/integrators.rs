use crate::dynamics::Dynamics;
use crate::errors::IntegrationError;
use crate::state::CartesianState;
use crate::types::{Epoch, Position, TimeStep, Velocity};

/// Semi-implicit Euler time integrator.
pub struct EulerIntegrator;

/// Advances a simulation state by one time step.
pub trait Integrator {
    /// Steps `state` forward by `dt` using the provided dynamics model.
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        state: &CartesianState,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<CartesianState, IntegrationError>;
}

impl Integrator for EulerIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        state: &CartesianState,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<CartesianState, IntegrationError> {
        let acceleration = dynamics.acceleration(state, t)?;
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let acceleration = acceleration.vector();

        debug_assert!(dt_s.is_finite());
        debug_assert!(dt_s > 0.0);
        debug_assert!(position.x.is_finite());
        debug_assert!(position.y.is_finite());
        debug_assert!(position.z.is_finite());
        debug_assert!(velocity.x.is_finite());
        debug_assert!(velocity.y.is_finite());
        debug_assert!(velocity.z.is_finite());
        debug_assert!(acceleration.x.is_finite());
        debug_assert!(acceleration.y.is_finite());
        debug_assert!(acceleration.z.is_finite());

        let new_vel = velocity + acceleration * dt_s;
        let new_pos = position + new_vel * dt_s;

        let velocity = Velocity::from_vector(new_vel).ok_or(IntegrationError::InvalidVelocity)?;
        let position = Position::from_vector(new_pos).ok_or(IntegrationError::InvalidPosition)?;

        Ok(CartesianState::new(position, velocity))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::DynamicsError;
    use crate::types::Acceleration;
    use nalgebra::Vector3;

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    struct ConstantAcceleration {
        acceleration: Acceleration,
    }

    impl Dynamics for ConstantAcceleration {
        fn acceleration(
            &self,
            _state: &CartesianState,
            _t: Epoch,
        ) -> Result<Acceleration, DynamicsError> {
            Ok(self.acceleration)
        }
    }

    struct FailingDynamics {
        error: DynamicsError,
    }

    impl Dynamics for FailingDynamics {
        fn acceleration(
            &self,
            _state: &CartesianState,
            _t: Epoch,
        ) -> Result<Acceleration, DynamicsError> {
            Err(self.error)
        }
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
    fn semi_implicit_euler_updates_velocity_before_position() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0))
                .ok_or("invalid acceleration")?,
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))
            .ok_or("invalid state")?;

        let next = integrator
            .step(
                &dynamics,
                &state,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
                TimeStep::from_seconds(5.0).ok_or("invalid time step")?,
            )
            .map_err(|_| "integration failed")?;

        assert_close(next.velocity.vector().x, 1.0, EPS);
        assert_close(next.velocity.vector().y, 12.0, EPS);
        assert_close(next.velocity.vector().z, 3.0, EPS);
        assert_close(next.position.vector().x, 15.0, EPS);
        assert_close(next.position.vector().y, 80.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    #[test]
    fn semi_implicit_euler_propagates_dynamics_errors() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = FailingDynamics {
            error: DynamicsError::PositionAtGravityCenter,
        };
        let state =
            state_at(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros()).ok_or("invalid state")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &state,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
                TimeStep::from_seconds(1.0).ok_or("invalid time step")?,
            ),
            Err(IntegrationError::Dynamics(
                DynamicsError::PositionAtGravityCenter
            )),
        ));
        Ok(())
    }

    #[test]
    fn semi_implicit_euler_rejects_nonfinite_velocity_update() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(f64::MAX, 0.0, 0.0))
                .ok_or("invalid acceleration")?,
        };
        let state = state_at(Vector3::zeros(), Vector3::zeros()).ok_or("invalid state")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &state,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
                TimeStep::from_seconds(2.0).ok_or("invalid time step")?,
            ),
            Err(IntegrationError::InvalidVelocity),
        ));
        Ok(())
    }

    #[test]
    fn semi_implicit_euler_rejects_nonfinite_position_update() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::zeros())
                .ok_or("invalid acceleration")?,
        };
        let state = state_at(
            Vector3::new(f64::MAX, 0.0, 0.0),
            Vector3::new(f64::MAX, 0.0, 0.0),
        )
        .ok_or("invalid state")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &state,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
                TimeStep::from_seconds(2.0).ok_or("invalid time step")?,
            ),
            Err(IntegrationError::InvalidPosition),
        ));
        Ok(())
    }

    #[test]
    fn time_step_rejects_zero() {
        assert!(TimeStep::from_seconds(0.0).is_none());
    }

    #[test]
    fn time_step_rejects_nan() {
        assert!(TimeStep::from_seconds(f64::NAN).is_none());
    }
}
