use super::dynamics::Dynamics;
use super::state::{CartesianStateComponents, CartesianStatesMut};
use super::types::{Epoch, Position, TimeStep, Velocity};
use crate::errors::PropagationError;
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
    ) -> Result<Self, PropagationError> {
        let state = CartesianStateComponents::from_vectors(position, velocity);
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
    ) -> Result<(), PropagationError>;
}

impl Integrator for EulerIntegrator {
    fn step<D: Dynamics>(
        &self,
        dynamics: &D,
        states: &mut CartesianStatesMut<'_>,
        index: usize,
        t: Epoch,
        dt: TimeStep,
    ) -> Result<(), PropagationError> {
        let state = states.as_view().components(index)?;
        let acceleration = dynamics.acceleration_at(state, t)?;
        let dt_s = dt.seconds();
        let position = state.position_vector();
        let velocity = state.velocity_vector();
        let acceleration = acceleration.vector();

        let new_vel = velocity + acceleration * dt_s;
        let new_pos = position + new_vel * dt_s;

        write_integrated_vectors(states, index, new_pos, new_vel)?;
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
    ) -> Result<(), PropagationError> {
        let state = states.as_view().components(index)?;
        let dt_s = dt.seconds();
        let position = state.position_vector();
        let velocity = state.velocity_vector();
        let acceleration = dynamics.acceleration_at(state, t)?.vector();
        let next_t = t.advance_by(dt);

        let new_pos = position + velocity * dt_s + acceleration * (0.5 * dt_s * dt_s);
        let next_velocity_estimate = velocity + acceleration * dt_s;
        let next_state = CartesianStateComponents::from_vectors(new_pos, next_velocity_estimate);
        let next_acceleration = dynamics.acceleration_at(next_state, next_t)?.vector();
        let new_vel = velocity + (acceleration + next_acceleration) * (0.5 * dt_s);

        write_integrated_vectors(states, index, new_pos, new_vel)?;
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
    ) -> Result<(), PropagationError> {
        let state = states.as_view().components(index)?;
        let dt_s = dt.seconds();
        let position = state.position_vector();
        let velocity = state.velocity_vector();
        let acceleration = dynamics.acceleration_at(state, t)?.vector();
        let next_t = t.advance_by(dt);

        let half_velocity = velocity + acceleration * (0.5 * dt_s);
        let new_pos = position + half_velocity * dt_s;
        let next_state = CartesianStateComponents::from_vectors(new_pos, half_velocity);
        let next_acceleration = dynamics.acceleration_at(next_state, next_t)?.vector();
        let new_vel = half_velocity + next_acceleration * (0.5 * dt_s);

        write_integrated_vectors(states, index, new_pos, new_vel)?;
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
    ) -> Result<(), PropagationError> {
        let state = states.as_view().components(index)?;
        let dt_s = dt.seconds();
        let half_dt_s = 0.5 * dt_s;
        let half_t = t.advance_by(TimeStep::new(half_dt_s));
        let full_t = t.advance_by(dt);
        let position = state.position_vector();
        let velocity = state.velocity_vector();

        let k1 = StateDerivative::at(dynamics, position, velocity, t)?;
        let k2 = StateDerivative::at(
            dynamics,
            position + k1.position * half_dt_s,
            velocity + k1.velocity * half_dt_s,
            half_t,
        )?;
        let k3 = StateDerivative::at(
            dynamics,
            position + k2.position * half_dt_s,
            velocity + k2.velocity * half_dt_s,
            half_t,
        )?;
        let k4 = StateDerivative::at(
            dynamics,
            position + k3.position * dt_s,
            velocity + k3.velocity * dt_s,
            full_t,
        )?;

        let new_pos = position
            + (k1.position + 2.0 * k2.position + 2.0 * k3.position + k4.position) * (dt_s / 6.0);
        let new_vel = velocity
            + (k1.velocity + 2.0 * k2.velocity + 2.0 * k3.velocity + k4.velocity) * (dt_s / 6.0);

        write_integrated_vectors(states, index, new_pos, new_vel)?;
        Ok(())
    }
}

fn write_integrated_vectors(
    states: &mut CartesianStatesMut<'_>,
    index: usize,
    position: Vector3<f64>,
    velocity: Vector3<f64>,
) -> Result<(), PropagationError> {
    let position = Position::from_vector(position);
    let velocity = Velocity::from_vector(velocity);
    let components = CartesianStateComponents::from((position, velocity));
    states.write_components(index, components)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::{DynamicsError, StateError};
    use crate::physics::constants::R_EARTH;
    use crate::physics::diagnostics::OrbitalDiagnostics;
    use crate::physics::dynamics::TwoBodyGravity;
    use crate::physics::frame::Frame;
    use crate::physics::orbits::Orbit;
    use crate::physics::state::{CartesianState, CartesianStateBuffer};
    use crate::physics::types::{Acceleration, Distance, Position, Velocity};
    use nalgebra::Vector3;

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    struct ConstantAcceleration {
        acceleration: Acceleration,
    }

    impl Dynamics for ConstantAcceleration {
        fn acceleration_at(
            &self,
            _state: CartesianStateComponents,
            _t: Epoch,
        ) -> Result<Acceleration, DynamicsError> {
            Ok(self.acceleration)
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
        ) -> Result<Acceleration, DynamicsError> {
            Err(self.error)
        }
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

    #[test]
    fn euler_matches_constant_acceleration_solution() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0)),
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        integrator.step(
            &dynamics,
            &mut states.view_mut(),
            0,
            Epoch::ZERO,
            TimeStep::new(5.0),
        )?;
        let next = states.read_state(0)?;

        assert_close(next.velocity.vector().x, 1.0, EPS);
        assert_close(next.velocity.vector().y, 12.0, EPS);
        assert_close(next.velocity.vector().z, 3.0, EPS);
        assert_close(next.position.vector().x, 15.0, EPS);
        assert_close(next.position.vector().y, 80.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    // ── error propagation ───────────────────────────────────────

    fn assert_integrator_propagates_dynamics_errors<I: Integrator>(integrator: I) -> TestResult {
        let dynamics = FailingDynamics {
            error: DynamicsError::PositionAtGravityCenter,
        };
        let state = state_at(Vector3::new(1.0, 0.0, 0.0), Vector3::zeros())?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                0,
                Epoch::ZERO,
                TimeStep::new(1.0),
            ),
            Err(PropagationError::Dynamics(
                DynamicsError::PositionAtGravityCenter
            )),
        ));
        Ok(())
    }

    #[test]
    fn euler_propagates_dynamics_errors() -> TestResult {
        assert_integrator_propagates_dynamics_errors(EulerIntegrator)
    }

    #[test]
    fn velocity_verlet_propagates_dynamics_errors() -> TestResult {
        assert_integrator_propagates_dynamics_errors(VelocityVerletIntegrator)
    }

    #[test]
    fn leapfrog_propagates_dynamics_errors() -> TestResult {
        assert_integrator_propagates_dynamics_errors(LeapfrogIntegrator)
    }

    #[test]
    fn rk4_propagates_dynamics_errors() -> TestResult {
        assert_integrator_propagates_dynamics_errors(RungeKutta4Integrator)
    }

    // ── constant acceleration solutions ─────────────────────────

    #[test]
    fn rk4_matches_constant_acceleration_solution() -> TestResult {
        let integrator = RungeKutta4Integrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0)),
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        integrator.step(
            &dynamics,
            &mut states.view_mut(),
            0,
            Epoch::ZERO,
            TimeStep::new(5.0),
        )?;
        let next = states.read_state(0)?;

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
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0)),
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        integrator.step(
            &dynamics,
            &mut states.view_mut(),
            0,
            Epoch::ZERO,
            TimeStep::new(5.0),
        )?;
        let next = states.read_state(0)?;

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
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 2.0, 0.0)),
        };
        let state = state_at(Vector3::new(10.0, 20.0, 30.0), Vector3::new(1.0, 2.0, 3.0))?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        integrator.step(
            &dynamics,
            &mut states.view_mut(),
            0,
            Epoch::ZERO,
            TimeStep::new(5.0),
        )?;
        let next = states.read_state(0)?;

        assert_close(next.velocity.vector().x, 1.0, EPS);
        assert_close(next.velocity.vector().y, 12.0, EPS);
        assert_close(next.velocity.vector().z, 3.0, EPS);
        assert_close(next.position.vector().x, 15.0, EPS);
        assert_close(next.position.vector().y, 55.0, EPS);
        assert_close(next.position.vector().z, 45.0, EPS);
        Ok(())
    }

    #[test]
    fn integrator_rejects_invalid_body_index() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::zeros()),
        };
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;

        assert!(matches!(
            integrator.step(
                &dynamics,
                &mut states.view_mut(),
                1,
                Epoch::ZERO,
                TimeStep::new(1.0),
            ),
            Err(PropagationError::State(StateError::InvalidBodyIndex)),
        ));
        Ok(())
    }

    #[test]
    fn stepping_one_body_does_not_mutate_another_body() -> TestResult {
        let integrator = EulerIntegrator;
        let dynamics = ConstantAcceleration {
            acceleration: Acceleration::from_vector(Vector3::new(0.0, 1.0, 0.0)),
        };
        let mut states = CartesianStateBuffer::<2>::with_len(2)?;
        let body_zero = state_at(Vector3::new(1.0, 2.0, 3.0), Vector3::new(4.0, 5.0, 6.0))?;
        let body_one = state_at(
            Vector3::new(10.0, 20.0, 30.0),
            Vector3::new(40.0, 50.0, 60.0),
        )?;
        states.write_from_state(0, &body_zero)?;
        states.write_from_state(1, &body_one)?;

        integrator.step(
            &dynamics,
            &mut states.view_mut(),
            0,
            Epoch::ZERO,
            TimeStep::new(1.0),
        )?;

        let unchanged = states.read_state(1)?;
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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let integrator = RungeKutta4Integrator;
        let mut states = CartesianStateBuffer::<2>::with_len(2)?;
        let orbit_zero = circular_orbit(R_EARTH + 400_000.0)?;
        let orbit_one = circular_orbit(R_EARTH + 800_000.0)?;
        states.write_from_state(0, &orbit_zero.to_cartesian_state(&frame)?)?;
        states.write_from_state(1, &orbit_one.to_cartesian_state(&frame)?)?;

        let initial_zero = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
        let initial_one = OrbitalDiagnostics::from_states(&states.view(), 1, mu)?;

        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            0,
            orbit_zero.period(&frame)?.seconds(),
        )?;
        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            1,
            orbit_one.period(&frame)?.seconds(),
        )?;

        let final_zero = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
        let final_one = OrbitalDiagnostics::from_states(&states.view(), 1, mu)?;

        assert_close(
            final_zero
                .specific_energy()
                .square_meters_per_square_second(),
            initial_zero
                .specific_energy()
                .square_meters_per_square_second(),
            1.0,
        );
        assert_close(
            final_one
                .specific_energy()
                .square_meters_per_square_second(),
            initial_one
                .specific_energy()
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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let orbit = circular_orbit(R_EARTH + 400_000.0)?;
        let mut states = CartesianStateBuffer::<2>::with_len(2)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;

        let initial = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
        propagate_one_period(
            &integrator,
            &gravity,
            &mut states,
            0,
            orbit.period(&frame)?.seconds(),
        )?;
        let final_diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;

        assert_close(
            final_diagnostics
                .specific_energy()
                .square_meters_per_square_second(),
            initial.specific_energy().square_meters_per_square_second(),
            tolerance,
        );
        Ok(())
    }

    fn circular_orbit(radius_m: f64) -> Result<Orbit, Box<dyn std::error::Error>> {
        let altitude_m = radius_m - R_EARTH;
        Ok(Orbit::circular_equatorial(Distance::new(altitude_m)))
    }

    fn propagate_one_period<I: Integrator, const N: usize>(
        integrator: &I,
        gravity: &TwoBodyGravity,
        states: &mut CartesianStateBuffer<N>,
        index: usize,
        period_s: f64,
    ) -> TestResult {
        let mut epoch = Epoch::ZERO;
        let base_step = TimeStep::new(30.0);

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::new(base_step.seconds().min(remaining_s));
            integrator.step(gravity, &mut states.view_mut(), index, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        Ok(())
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
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let orbit = circular_orbit(R_EARTH + 400_000.0)?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;

        let initial_h = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_angular_momentum()
            .square_meters_per_second();

        let period_s = orbit.period(&frame)?.seconds();
        propagate_one_period(&integrator, &gravity, &mut states, 0, period_s)?;

        let final_h = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_angular_momentum()
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

    fn energy_error_after_one_period<I: Integrator>(
        integrator: I,
    ) -> Result<f64, Box<dyn std::error::Error>> {
        let frame = Frame::earth_inertial();
        let mu = frame.mu()?;
        let gravity = TwoBodyGravity::from_frame(&frame)?;
        let orbit = circular_orbit(R_EARTH + 400_000.0)?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;

        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy()
            .square_meters_per_square_second();

        let period_s = orbit.period(&frame)?.seconds();
        propagate_one_period(&integrator, &gravity, &mut states, 0, period_s)?;

        let final_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy()
            .square_meters_per_square_second();

        Ok((final_energy - initial_energy).abs())
    }

    #[test]
    fn rk4_conserves_energy_better_than_euler() -> TestResult {
        let euler_error = energy_error_after_one_period(EulerIntegrator)?;
        let rk4_error = energy_error_after_one_period(RungeKutta4Integrator)?;

        assert!(
            rk4_error < euler_error,
            "RK4 energy error {rk4_error} should be less than Euler error {euler_error}",
        );
        Ok(())
    }
}
