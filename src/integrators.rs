use crate::dynamics::Dynamics;
use crate::errors::IntegrationError;
use crate::state::CartesianState;
use crate::types::{Epoch, Position, TimeStep, Velocity};
use nalgebra::Vector3;

/// Semi-implicit Euler time integrator.
pub struct EulerIntegrator;

/// Fourth-order Runge-Kutta time integrator.
pub struct RungeKutta4Integrator;

struct StateDerivative {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
}

impl StateDerivative {
    fn at<D: Dynamics>(
        dynamics: &D,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        t: Epoch,
    ) -> Result<Self, IntegrationError> {
        let state = CartesianState::new(
            Position::from_vector(position).ok_or(IntegrationError::InvalidPosition)?,
            Velocity::from_vector(velocity).ok_or(IntegrationError::InvalidVelocity)?,
        );
        let acceleration = dynamics.acceleration(&state, t)?;

        Ok(Self {
            position: velocity,
            velocity: acceleration.vector(),
        })
    }
}

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

impl Integrator for RungeKutta4Integrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        state: &CartesianState,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<CartesianState, IntegrationError> {
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();

        debug_assert!(dt_s.is_finite());
        debug_assert!(dt_s > 0.0);
        debug_assert!(position.x.is_finite());
        debug_assert!(position.y.is_finite());
        debug_assert!(position.z.is_finite());
        debug_assert!(velocity.x.is_finite());
        debug_assert!(velocity.y.is_finite());
        debug_assert!(velocity.z.is_finite());

        let k1 = StateDerivative::at(dynamics, position, velocity, t)?;
        let k2 = StateDerivative::at(
            dynamics,
            position + k1.position * (0.5 * dt_s),
            velocity + k1.velocity * (0.5 * dt_s),
            advance_epoch(t, 0.5 * dt_s)?,
        )?;
        let k3 = StateDerivative::at(
            dynamics,
            position + k2.position * (0.5 * dt_s),
            velocity + k2.velocity * (0.5 * dt_s),
            advance_epoch(t, 0.5 * dt_s)?,
        )?;
        let k4 = StateDerivative::at(
            dynamics,
            position + k3.position * dt_s,
            velocity + k3.velocity * dt_s,
            advance_epoch(t, dt_s)?,
        )?;

        let new_pos = position
            + (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position) * (dt_s / 6.0);
        let new_vel = velocity
            + (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) * (dt_s / 6.0);

        let velocity = Velocity::from_vector(new_vel).ok_or(IntegrationError::InvalidVelocity)?;
        let position = Position::from_vector(new_pos).ok_or(IntegrationError::InvalidPosition)?;

        Ok(CartesianState::new(position, velocity))
    }
}

fn advance_epoch(t: Epoch, seconds: f64) -> Result<Epoch, IntegrationError> {
    Epoch::from_seconds(t.seconds() + seconds).ok_or(IntegrationError::InvalidEpoch)
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
    fn runge_kutta4_matches_constant_acceleration_solution() -> TestResult {
        let integrator = RungeKutta4Integrator;
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
        assert_close(next.position.vector().y, 55.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    #[test]
    fn runge_kutta4_propagates_dynamics_errors() -> TestResult {
        let integrator = RungeKutta4Integrator;
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
    fn time_step_rejects_zero() {
        assert!(TimeStep::from_seconds(0.0).is_none());
    }

    #[test]
    fn time_step_rejects_nan() {
        assert!(TimeStep::from_seconds(f64::NAN).is_none());
    }
}
