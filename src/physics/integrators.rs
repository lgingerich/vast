use super::dynamics::Dynamics;
use super::state::{
    CartesianPositionComponents, CartesianStateComponents, CartesianStatesMut,
    CartesianVelocityComponents,
};
use super::types::{Epoch, TimeStep};
use crate::errors::IntegrationError;
use nalgebra::Vector3;

/// Semi-implicit Euler time integrator.
pub struct EulerIntegrator;

/// Velocity Verlet time integrator.
///
/// This symplectic integrator is a good default for long-rollout orbital
/// propagation with position-dependent accelerations.
pub struct VelocityVerletIntegrator;

/// Leapfrog time integrator using kick-drift-kick ordering.
///
/// For position-dependent dynamics this is mathematically equivalent to
/// velocity Verlet, but the explicit kick-drift-kick form is useful for future
/// splitting methods and control insertion points.
pub struct LeapfrogIntegrator;

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
        let state = components_from_vectors(position, velocity)?;
        let acceleration = dynamics.acceleration_at(state, t)?;

        Ok(Self {
            position: velocity,
            velocity: acceleration.vector(),
        })
    }
}

/// Advances a simulation state by one time step.
pub trait Integrator {
    /// Steps one indexed body forward by `dt` using the provided dynamics model.
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), IntegrationError>;
}

impl Integrator for EulerIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), IntegrationError> {
        let state = states.components(index)?;
        let acceleration = dynamics.acceleration_at(state, t)?;
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let acceleration = acceleration.vector();

        let new_vel = velocity + acceleration * dt_s;
        let new_pos = position + new_vel * dt_s;

        let velocity =
            CartesianVelocityComponents::from_meters_per_second(new_vel.x, new_vel.y, new_vel.z)?;
        let position = CartesianPositionComponents::from_meters(new_pos.x, new_pos.y, new_pos.z)?;

        states.write_components(index, CartesianStateComponents { position, velocity })?;
        Ok(())
    }
}

impl Integrator for VelocityVerletIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), IntegrationError> {
        let state = states.components(index)?;
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let acceleration = dynamics.acceleration_at(state, t)?.vector();
        let next_t = t.advance_by(dt)?;

        let new_pos = position + velocity * dt_s + acceleration * (0.5 * dt_s * dt_s);
        let new_position =
            CartesianPositionComponents::from_meters(new_pos.x, new_pos.y, new_pos.z)?;
        let next_state = CartesianStateComponents {
            position: new_position,
            velocity: CartesianVelocityComponents::from_meters_per_second(
                (velocity + acceleration * dt_s).x,
                (velocity + acceleration * dt_s).y,
                (velocity + acceleration * dt_s).z,
            )?,
        };
        let next_acceleration = dynamics.acceleration_at(next_state, next_t)?.vector();
        let new_vel = velocity + (acceleration + next_acceleration) * (0.5 * dt_s);
        let new_velocity =
            CartesianVelocityComponents::from_meters_per_second(new_vel.x, new_vel.y, new_vel.z)?;

        states.write_components(
            index,
            CartesianStateComponents {
                position: new_position,
                velocity: new_velocity,
            },
        )?;
        Ok(())
    }
}

impl Integrator for LeapfrogIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), IntegrationError> {
        let state = states.components(index)?;
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let acceleration = dynamics.acceleration_at(state, t)?.vector();
        let next_t = t.advance_by(dt)?;

        let half_velocity = velocity + acceleration * (0.5 * dt_s);
        let new_pos = position + half_velocity * dt_s;
        let new_position =
            CartesianPositionComponents::from_meters(new_pos.x, new_pos.y, new_pos.z)?;
        let next_state = CartesianStateComponents {
            position: new_position,
            velocity: CartesianVelocityComponents::from_meters_per_second(
                half_velocity.x,
                half_velocity.y,
                half_velocity.z,
            )?,
        };
        let next_acceleration = dynamics.acceleration_at(next_state, next_t)?.vector();
        let new_vel = half_velocity + next_acceleration * (0.5 * dt_s);
        let new_velocity =
            CartesianVelocityComponents::from_meters_per_second(new_vel.x, new_vel.y, new_vel.z)?;

        states.write_components(
            index,
            CartesianStateComponents {
                position: new_position,
                velocity: new_velocity,
            },
        )?;
        Ok(())
    }
}

impl Integrator for RungeKutta4Integrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), IntegrationError> {
        let state = states.components(index)?;
        let dt_s = dt.seconds();
        let position = state.position.vector();
        let velocity = state.velocity.vector();

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

        let position = CartesianPositionComponents::from_meters(new_pos.x, new_pos.y, new_pos.z)?;
        let velocity =
            CartesianVelocityComponents::from_meters_per_second(new_vel.x, new_vel.y, new_vel.z)?;

        states.write_components(index, CartesianStateComponents { position, velocity })?;
        Ok(())
    }
}

fn advance_epoch(t: Epoch, seconds: f64) -> Result<Epoch, IntegrationError> {
    Ok(Epoch::from_seconds(t.seconds() + seconds)?)
}

fn components_from_vectors(
    position: Vector3<f64>,
    velocity: Vector3<f64>,
) -> Result<CartesianStateComponents, IntegrationError> {
    Ok(CartesianStateComponents {
        position: CartesianPositionComponents::from_meters(position.x, position.y, position.z)?,

        velocity: CartesianVelocityComponents::from_meters_per_second(
            velocity.x, velocity.y, velocity.z,
        )?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::{DomainError, DynamicsError, StateError};
    use crate::physics::constants::{MU_EARTH, R_EARTH};
    use crate::physics::diagnostics::OrbitalDiagnostics;
    use crate::physics::dynamics::TwoBodyGravity;
    use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
    use crate::physics::state::{
        CartesianAccelerationComponents, CartesianState, CartesianStateBuffer,
    };
    use crate::physics::types::{
        Acceleration, Angle, Distance, Eccentricity, GravitationalParameter, Position, Velocity,
    };
    use nalgebra::Vector3;

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    struct ConstantAcceleration {
        acceleration: Acceleration,
    }

    impl Dynamics for ConstantAcceleration {
        fn acceleration_at(
            &self,
            _state: CartesianStateComponents,
            _t: Epoch,
        ) -> Result<CartesianAccelerationComponents, DynamicsError> {
            let acceleration = self.acceleration.vector();
            CartesianAccelerationComponents::from_meters_per_second_squared(
                acceleration.x,
                acceleration.y,
                acceleration.z,
            )
            .map_err(|_| DynamicsError::InvalidAcceleration)
        }
    }

    struct FailingDynamics {
        error: DynamicsError,
    }

    impl Dynamics for FailingDynamics {
        fn acceleration_at(
            &self,
            _state: CartesianStateComponents,
            _t: Epoch,
        ) -> Result<CartesianAccelerationComponents, DynamicsError> {
            Err(self.error)
        }
    }

    fn state_at(position: Vector3<f64>, velocity: Vector3<f64>) -> Option<CartesianState> {
        Some(CartesianState::new(
            Position::from_vector(position).ok()?,
            Velocity::from_vector(velocity).ok()?,
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
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))
            .ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        integrator
            .step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(5.0).map_err(|_| "invalid time step")?,
            )
            .map_err(|_| "integration failed")?;
        let next = states.read_state(0).map_err(|_| "invalid next state")?;

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
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(1.0).map_err(|_| "invalid time step")?,
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
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(Vector3::zeros(), Vector3::zeros()).ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(2.0).map_err(|_| "invalid time step")?,
            ),
            Err(IntegrationError::Domain(DomainError::NotFinite)),
        ));
        Ok(())
    }

    #[test]
    fn semi_implicit_euler_rejects_nonfinite_position_update() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::zeros())
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(
            Vector3::new(f64::MAX, 0.0, 0.0),
            Vector3::new(f64::MAX, 0.0, 0.0),
        )
        .ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(2.0).map_err(|_| "invalid time step")?,
            ),
            Err(IntegrationError::Domain(DomainError::NotFinite)),
        ));
        Ok(())
    }

    #[test]
    fn runge_kutta4_matches_constant_acceleration_solution() -> TestResult {
        let integrator = RungeKutta4Integrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0))
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))
            .ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        integrator
            .step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(5.0).map_err(|_| "invalid time step")?,
            )
            .map_err(|_| "integration failed")?;
        let next = states.read_state(0).map_err(|_| "invalid next state")?;

        assert_close(next.velocity.vector().x, 1.0, EPS);
        assert_close(next.velocity.vector().y, 12.0, EPS);
        assert_close(next.velocity.vector().z, 3.0, EPS);
        assert_close(next.position.vector().x, 15.0, EPS);
        assert_close(next.position.vector().y, 55.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    #[test]
    fn velocity_verlet_matches_constant_acceleration_solution() -> TestResult {
        let integrator = VelocityVerletIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0))
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))
            .ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        integrator
            .step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(5.0).map_err(|_| "invalid time step")?,
            )
            .map_err(|_| "integration failed")?;
        let next = states.read_state(0).map_err(|_| "invalid next state")?;

        assert_close(next.velocity.vector().x, 1.0, EPS);
        assert_close(next.velocity.vector().y, 12.0, EPS);
        assert_close(next.velocity.vector().z, 3.0, EPS);
        assert_close(next.position.vector().x, 15.0, EPS);
        assert_close(next.position.vector().y, 55.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    #[test]
    fn leapfrog_matches_constant_acceleration_solution() -> TestResult {
        let integrator = LeapfrogIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0))
                .map_err(|_| "invalid acceleration")?,
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))
            .ok_or("invalid state")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        integrator
            .step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(5.0).map_err(|_| "invalid time step")?,
            )
            .map_err(|_| "integration failed")?;
        let next = states.read_state(0).map_err(|_| "invalid next state")?;

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
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(0, &state)
            .map_err(|_| "invalid state buffer")?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(1.0).map_err(|_| "invalid time step")?,
            ),
            Err(IntegrationError::Dynamics(
                DynamicsError::PositionAtGravityCenter
            )),
        ));
        Ok(())
    }

    #[test]
    fn integrator_rejects_invalid_body_index() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::zeros())
                .map_err(|_| "invalid acceleration")?,
        };
        let mut states = CartesianStateBuffer::<1>::new();

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                1,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(1.0).map_err(|_| "invalid time step")?,
            ),
            Err(IntegrationError::State(StateError::InvalidBodyIndex)),
        ));
        Ok(())
    }

    #[test]
    fn stepping_one_body_does_not_mutate_another_body() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 1.0, 0.0))
                .map_err(|_| "invalid acceleration")?,
        };
        let mut states = CartesianStateBuffer::<2>::new();
        let body_zero = state_at(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0))
            .ok_or("invalid body zero")?;
        let body_one = state_at(
            Vector3::new(10.0, 20.0, 30.0),
            Vector3::new(40.0, 50.0, 60.0),
        )
        .ok_or("invalid body one")?;
        states
            .write_from_state(0, &body_zero)
            .map_err(|_| "failed to write body zero")?;
        states
            .write_from_state(1, &body_one)
            .map_err(|_| "failed to write body one")?;

        integrator
            .step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
                TimeStep::from_seconds(1.0).map_err(|_| "invalid time step")?,
            )
            .map_err(|_| "integration failed")?;

        let unchanged = states
            .read_state(1)
            .map_err(|_| "failed to read body one")?;
        assert_close(
            unchanged.position.vector().x,
            body_one.position.vector().x,
            EPS,
        );
        assert_close(
            unchanged.position.vector().y,
            body_one.position.vector().y,
            EPS,
        );
        assert_close(
            unchanged.position.vector().z,
            body_one.position.vector().z,
            EPS,
        );
        assert_close(
            unchanged.velocity.vector().x,
            body_one.velocity.vector().x,
            EPS,
        );
        assert_close(
            unchanged.velocity.vector().y,
            body_one.velocity.vector().y,
            EPS,
        );
        assert_close(
            unchanged.velocity.vector().z,
            body_one.velocity.vector().z,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn two_independent_central_gravity_bodies_conserve_energy() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid earth mu")?;
        let gravity = TwoBodyGravity { mu };
        let integrator = RungeKutta4Integrator;
        let mut states = CartesianStateBuffer::<2>::new();
        let orbit_zero =
            circular_orbit(R_EARTH + 400_000.0, 0.0, mu).ok_or("invalid first circular orbit")?;
        let orbit_one =
            circular_orbit(R_EARTH + 800_000.0, 0.5, mu).ok_or("invalid second circular orbit")?;
        states
            .write_from_state(
                0,
                &orbit_zero
                    .to_cartesian_state()
                    .ok_or("invalid first state")?,
            )
            .map_err(|_| "failed to write first state")?;
        states
            .write_from_state(
                1,
                &orbit_one
                    .to_cartesian_state()
                    .ok_or("invalid second state")?,
            )
            .map_err(|_| "failed to write second state")?;

        let initial_zero = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial first diagnostics")?;
        let initial_one = OrbitalDiagnostics::from_states(&states.view(), 1, mu)
            .ok_or("invalid initial second diagnostics")?;

        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            0,
            orbit_zero.period().ok_or("invalid first period")?.seconds(),
        )?;
        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            1,
            orbit_one.period().ok_or("invalid second period")?.seconds(),
        )?;

        let final_zero = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final first diagnostics")?;
        let final_one = OrbitalDiagnostics::from_states(&states.view(), 1, mu)
            .ok_or("invalid final second diagnostics")?;

        assert_close(
            final_zero.specific_energy.square_meters_per_square_second(),
            initial_zero
                .specific_energy
                .square_meters_per_square_second(),
            1.0,
        );
        assert_close(
            final_one.specific_energy.square_meters_per_square_second(),
            initial_one
                .specific_energy
                .square_meters_per_square_second(),
            1.0,
        );
        Ok(())
    }

    #[test]
    fn velocity_verlet_conserves_circular_orbit_energy() -> TestResult {
        assert_circular_orbit_energy_conserved(VelocityVerletIntegrator, 1.0)
    }

    #[test]
    fn leapfrog_conserves_circular_orbit_energy() -> TestResult {
        assert_circular_orbit_energy_conserved(LeapfrogIntegrator, 1.0)
    }

    fn assert_circular_orbit_energy_conserved<I: Integrator>(
        integrator: I,
        tolerance: f64,
    ) -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid earth mu")?;
        let gravity = TwoBodyGravity { mu };
        let orbit = circular_orbit(R_EARTH + 400_000.0, 0.0, mu).ok_or("invalid circular orbit")?;
        let mut states = CartesianStateBuffer::<2>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write initial state")?;

        let initial = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?;
        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            0,
            orbit.period().ok_or("invalid period")?.seconds(),
        )?;
        let final_diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final diagnostics")?;

        assert_close(
            final_diagnostics
                .specific_energy
                .square_meters_per_square_second(),
            initial.specific_energy.square_meters_per_square_second(),
            tolerance,
        );
        Ok(())
    }

    fn circular_orbit(
        radius_m: f64,
        phase_rad: f64,
        mu: GravitationalParameter,
    ) -> Option<EllipticOrbit> {
        Some(EllipticOrbit {
            semi_major_axis: Distance::from_meters(radius_m).ok()?,
            eccentricity: Eccentricity::from_scalar(0.0).ok()?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(phase_rad).ok()?,
            direction: OrbitDirection::Prograde,
        })
    }

    fn propagate_one_period<I: Integrator>(
        integrator: &I,
        gravity: &TwoBodyGravity,
        states: &mut CartesianStateBuffer<2>,
        index: usize,
        period_s: f64,
    ) -> TestResult {
        let mut epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;
        let base_step = TimeStep::from_seconds(30.0).map_err(|_| "invalid base time step")?;

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(base_step.seconds().min(remaining_s))
                .map_err(|_| "invalid step")?;
            integrator
                .step(gravity, &mut states.view_mut(), index, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).map_err(|_| "invalid next epoch")?;
        }

        Ok(())
    }

    #[test]
    fn time_step_rejects_zero() {
        assert!(TimeStep::from_seconds(0.0).is_err());
    }

    #[test]
    fn time_step_rejects_nan() {
        assert!(TimeStep::from_seconds(f64::NAN).is_err());
    }

    // ── angular momentum conservation ──────────────────────────────
    //
    // For two-body gravity, angular momentum h = r × v is an exact
    // constant of motion: dh/dt = r × a = r × (-mu·r/r³) = 0
    // because the force is always radial (zero torque).
    //
    // Symplectic integrators (Verlet, Leapfrog) conserve h to near
    // machine precision. Non-symplectic integrators (Euler, RK4)
    // still preserve it — their angular momentum error is a 4th-order
    // truncation artifact from intermediate force evaluations at
    // positions that don't lie on the true orbit. We use per-integrator
    // tolerances: tight for symplectic, looser (but still orders of
    // magnitude below any physically detectable effect) for others.

    fn specific_angular_momentum_after_one_period<I: Integrator>(
        integrator: I,
        tolerance: f64,
    ) -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid earth mu")?;
        let gravity = TwoBodyGravity { mu };
        let orbit = circular_orbit(R_EARTH + 400_000.0, 0.0, mu).ok_or("invalid circular orbit")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;

        let initial_h = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?
            .specific_angular_momentum
            .square_meters_per_second();

        let mut epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;
        let period_s = orbit.period().ok_or("invalid period")?.seconds();
        let dt = TimeStep::from_seconds(30.0).map_err(|_| "invalid time step")?;

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_s))
                .map_err(|_| "invalid step")?;
            integrator
                .step(&gravity, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).map_err(|_| "invalid next epoch")?;
        }

        let final_h = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final diagnostics")?
            .specific_angular_momentum
            .square_meters_per_second();

        assert!(final_h.is_finite(), "angular momentum is non-finite");
        let error = (final_h - initial_h).abs();
        assert!(
            error <= tolerance,
            "angular momentum error {error} exceeds tolerance {tolerance} (h ≈ {initial_h:.0} m²/s)",
        );
        Ok(())
    }

    #[test]
    fn velocity_verlet_conserves_angular_momentum() -> TestResult {
        specific_angular_momentum_after_one_period(VelocityVerletIntegrator, 1.0)
    }

    #[test]
    fn leapfrog_conserves_angular_momentum() -> TestResult {
        specific_angular_momentum_after_one_period(LeapfrogIntegrator, 1.0)
    }

    #[test]
    fn rk4_preserves_angular_momentum() -> TestResult {
        specific_angular_momentum_after_one_period(RungeKutta4Integrator, 500.0)
    }

    #[test]
    fn euler_preserves_angular_momentum() -> TestResult {
        specific_angular_momentum_after_one_period(EulerIntegrator, 10_000.0)
    }

    // ── integrator energy fidelity ordering ────────────────────────

    fn energy_error_after_one_period<I: Integrator>(integrator: I) -> Result<f64, &'static str> {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid earth mu")?;
        let gravity = TwoBodyGravity { mu };
        let orbit = circular_orbit(R_EARTH + 400_000.0, 0.0, mu).ok_or("invalid circular orbit")?;
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;

        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?
            .specific_energy
            .square_meters_per_square_second();

        let mut epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;
        let period_s = orbit.period().ok_or("invalid period")?.seconds();
        let dt = TimeStep::from_seconds(30.0).map_err(|_| "invalid time step")?;

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_s))
                .map_err(|_| "invalid step")?;
            integrator
                .step(&gravity, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).map_err(|_| "invalid next epoch")?;
        }

        let final_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final diagnostics")?
            .specific_energy
            .square_meters_per_square_second();

        Ok((final_energy - initial_energy).abs())
    }

    #[test]
    fn rk4_conserves_energy_better_than_euler() -> TestResult {
        let euler_error = energy_error_after_one_period(EulerIntegrator)
            .map_err(|_| "failed querying euler energy error")?;
        let rk4_error = energy_error_after_one_period(RungeKutta4Integrator)
            .map_err(|_| "failed querying rk4 energy error")?;

        assert!(
            rk4_error < euler_error,
            "RK4 energy error {rk4_error} should be less than Euler error {euler_error}",
        );
        Ok(())
    }
}
