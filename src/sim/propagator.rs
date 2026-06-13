use super::termination::{
    StopReason, TerminationCondition, TerminationContext, orbital_phase_angle_rad,
    radial_velocity_mps,
};
use crate::errors::{NumericalError, PropagationError};
use crate::physics::dynamics::Dynamics;
use crate::physics::integrators::Integrator;
use crate::physics::state::{CartesianState, CartesianStateBuffer};
use crate::physics::types::{Epoch, GravitationalParameter, TimeStep, Velocity};
use nalgebra::Vector3;

/// Discrete velocity change applied at a segment boundary (above dynamics).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImpulsiveBurn {
    /// Inertial Cartesian delta-v in meters per second.
    pub delta_v_mps: [f64; 3],
}

impl ImpulsiveBurn {
    /// Creates an inertial delta-v burn.
    pub fn inertial(delta_v_mps: [f64; 3]) -> Result<Self, PropagationError> {
        if !delta_v_mps.iter().all(|c| c.is_finite()) {
            return Err(PropagationError::Numerical(NumericalError::NotFinite {
                field: "delta_v",
            }));
        }
        Ok(Self { delta_v_mps })
    }

    /// Creates a prograde burn with the given speed increment.
    pub fn prograde(delta_v_mps: f64, state: &CartesianState) -> Result<Self, PropagationError> {
        let velocity = state.velocity.vector();
        let speed = velocity.norm();
        if speed <= 0.0 || !speed.is_finite() {
            return Err(PropagationError::InvalidBurn);
        }
        let direction = velocity / speed;
        Ok(Self {
            delta_v_mps: [
                direction.x * delta_v_mps,
                direction.y * delta_v_mps,
                direction.z * delta_v_mps,
            ],
        })
    }
}

/// One recorded state sample along a propagation.
#[derive(Debug, Clone, PartialEq)]
pub struct TrajectorySample {
    /// Simulation epoch of the sample.
    pub epoch: Epoch,
    /// Cartesian state at the sample.
    pub state: CartesianState,
    /// Mission segment index when recorded by a mission runner.
    pub segment_index: Option<usize>,
}

impl TrajectorySample {
    /// Creates a trajectory sample with an optional segment index.
    #[must_use]
    pub fn new(epoch: Epoch, state: CartesianState, segment_index: Option<usize>) -> Self {
        Self {
            epoch,
            state,
            segment_index,
        }
    }
}

/// Result of propagating until a termination condition or step budget.
#[derive(Debug, Clone, PartialEq)]
pub struct PropagationResult {
    /// Epoch after the final step.
    pub final_epoch: Epoch,
    /// State after the final step.
    pub final_state: CartesianState,
    /// Number of integrator steps taken.
    pub steps: usize,
    /// Why propagation stopped.
    pub stop_reason: StopReason,
    /// Sampled trajectory points collected during propagation.
    pub samples: Vec<TrajectorySample>,
}

/// Configuration for trajectory sampling during propagation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SampleConfig {
    /// Record a sample every N integrator steps (must be >= 1).
    pub every_steps: usize,
}

impl SampleConfig {
    /// Creates a sample config with a positive interval.
    pub fn every_steps(value: usize) -> Option<Self> {
        (value >= 1).then_some(Self { every_steps: value })
    }
}

/// Fixed-step propagator for a single indexed body in a state buffer.
#[derive(Debug)]
pub struct Propagator {
    epoch: Epoch,
    initial_epoch: Epoch,
    dt: TimeStep,
    max_steps: Option<usize>,
    sample_config: Option<SampleConfig>,
    states: CartesianStateBuffer<1>,
}

impl Propagator {
    /// Creates a propagator starting from `initial_state` at the given epoch.
    pub fn new(
        initial_state: CartesianState,
        initial_epoch: Epoch,
        dt: TimeStep,
    ) -> Result<Self, PropagationError> {
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &initial_state)?;

        Ok(Self {
            epoch: initial_epoch,
            initial_epoch,
            dt,
            max_steps: None,
            sample_config: None,
            states,
        })
    }

    /// Sets an optional safety limit on integrator steps per propagation call.
    #[must_use]
    pub fn with_max_steps(mut self, max_steps: usize) -> Self {
        self.max_steps = Some(max_steps);
        self
    }

    /// Enables trajectory sampling every N integrator steps.
    #[must_use]
    pub fn with_sampling(mut self, config: SampleConfig) -> Self {
        self.sample_config = Some(config);
        self
    }

    /// Resets epoch and state for a new episode.
    pub fn reset(&mut self, initial_state: CartesianState) -> Result<(), PropagationError> {
        self.epoch = self.initial_epoch;
        self.states.write_from_state(0, &initial_state)?;
        Ok(())
    }

    /// Reads the current body state at index 0.
    pub fn current_state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.read_state(0)?)
    }

    /// Applies an impulsive delta-v to body index 0.
    pub fn apply_impulsive_burn(&mut self, burn: ImpulsiveBurn) -> Result<(), PropagationError> {
        let state = self.states.read_state(0)?;
        let velocity = state.velocity.vector();
        let updated = Velocity::from_vector(Vector3::new(
            velocity.x + burn.delta_v_mps[0],
            velocity.y + burn.delta_v_mps[1],
            velocity.z + burn.delta_v_mps[2],
        ))?;
        let updated_state = CartesianState::new(state.position, updated);
        self.states.write_from_state(0, &updated_state)?;
        Ok(())
    }

    /// Propagates until `termination` is met, the step budget is exceeded, or
    /// no forward progress remains.
    pub fn propagate_until<I, D>(
        &mut self,
        integrator: &I,
        dynamics: &D,
        termination: TerminationCondition,
        mu: GravitationalParameter,
    ) -> Result<PropagationResult, PropagationError>
    where
        I: Integrator,
        D: Dynamics,
    {
        self.propagate_until_with_segment(integrator, dynamics, termination, mu, None)
    }

    /// Propagates until `termination` is met, tagging samples with `segment_index`.
    pub fn propagate_until_with_segment<I, D>(
        &mut self,
        integrator: &I,
        dynamics: &D,
        termination: TerminationCondition,
        mu: GravitationalParameter,
        segment_index: Option<usize>,
    ) -> Result<PropagationResult, PropagationError>
    where
        I: Integrator,
        D: Dynamics,
    {
        let segment_start_epoch = self.epoch;
        let mut steps = 0_usize;
        let mut samples = Vec::new();
        let needs_orbital_phase = termination.needs_orbital_phase();
        let mut current_orbital_phase_angle_rad = if needs_orbital_phase {
            orbital_phase_angle_rad(&self.current_state()?, mu)
        } else {
            None
        };
        let mut previous_orbital_angle_rad = current_orbital_phase_angle_rad;
        let mut previous_radial_velocity_mps = Some(radial_velocity_mps(&self.current_state()?));

        if self.sample_config.is_some() {
            let state = self.states.read_state(0)?;
            samples.push(TrajectorySample::new(self.epoch, state, segment_index));
        }

        loop {
            let state = self.states.read_state(0)?;
            let context = TerminationContext {
                start_epoch: segment_start_epoch,
                current_epoch: self.epoch,
                steps,
                state: &state,
                previous_orbital_angle_rad,
                previous_radial_velocity_mps,
                current_orbital_phase_angle_rad,
            };

            if let Some(reason) = termination.stop_reason(context) {
                return Ok(PropagationResult {
                    final_epoch: self.epoch,
                    final_state: state,
                    steps,
                    stop_reason: reason,
                    samples,
                });
            }

            if let Some(max_steps) = self.max_steps
                && steps >= max_steps
            {
                return Ok(PropagationResult {
                    final_epoch: self.epoch,
                    final_state: state,
                    steps,
                    stop_reason: StopReason::StepBudgetExceeded,
                    samples,
                });
            }

            let step = match termination.remaining(self.initial_epoch, self.epoch) {
                Some(remaining) if remaining.seconds() < self.dt.seconds() => remaining,
                _ => self.dt,
            };

            integrator.step(dynamics, &mut self.states.view_mut(), 0, self.epoch, step)?;

            self.epoch = self.epoch.advance_by(step);
            steps += 1;

            let state = self.states.read_state(0)?;
            if needs_orbital_phase {
                current_orbital_phase_angle_rad = orbital_phase_angle_rad(&state, mu);
            }
            previous_orbital_angle_rad = current_orbital_phase_angle_rad;
            previous_radial_velocity_mps = Some(radial_velocity_mps(&state));

            if let Some(config) = self.sample_config
                && steps.is_multiple_of(config.every_steps)
            {
                samples.push(TrajectorySample::new(self.epoch, state, segment_index));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Angle, Distance, Eccentricity, EllipticOrbit, GravitationalParameter, LeapfrogIntegrator,
        MU_EARTH, OrbitDirection, R_EARTH, StopReason, TerminationCondition, TwoBodyGravity,
    };

    fn leo_initial_state() -> CartesianState {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let orbit = EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 400_000.0).expect("radius"),
            Eccentricity::from_scalar(0.0).expect("eccentricity"),
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0).expect("phase"),
            OrbitDirection::Prograde,
        )
        .expect("orbit");
        orbit.to_cartesian_state().expect("state")
    }

    #[test]
    fn propagates_exactly_one_period() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let orbit = EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 400_000.0).expect("radius"),
            Eccentricity::from_scalar(0.0).expect("eccentricity"),
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0).expect("phase"),
            OrbitDirection::Prograde,
        )
        .expect("orbit");
        let period = orbit.period().expect("period");
        let dt = TimeStep::from_seconds(10.0).expect("dt");
        let gravity = TwoBodyGravity::new(mu);
        let integrator = LeapfrogIntegrator;

        let mut propagator = Propagator::new(
            leo_initial_state(),
            Epoch::from_seconds(0.0).expect("epoch"),
            dt,
        )
        .expect("propagator");
        let result = propagator
            .propagate_until(
                &integrator,
                &gravity,
                TerminationCondition::Duration(period),
                mu,
            )
            .expect("propagation");

        assert_eq!(result.stop_reason, StopReason::DurationMet);
        assert!((result.final_epoch.seconds() - period.seconds()).abs() < 1e-9);
        assert!(result.steps > 0);
    }

    #[test]
    fn respects_step_count_termination() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let gravity = TwoBodyGravity::new(mu);
        let integrator = LeapfrogIntegrator;
        let dt = TimeStep::from_seconds(10.0).expect("dt");

        let mut propagator = Propagator::new(
            leo_initial_state(),
            Epoch::from_seconds(0.0).expect("epoch"),
            dt,
        )
        .expect("propagator");
        let result = propagator
            .propagate_until(
                &integrator,
                &gravity,
                TerminationCondition::StepCount(3),
                mu,
            )
            .expect("propagation");

        assert_eq!(result.stop_reason, StopReason::StepCountMet);
        assert_eq!(result.steps, 3);
        assert!((result.final_epoch.seconds() - 30.0).abs() < 1e-9);
    }

    #[test]
    fn collects_samples_on_interval() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let gravity = TwoBodyGravity::new(mu);
        let integrator = LeapfrogIntegrator;
        let dt = TimeStep::from_seconds(10.0).expect("dt");

        let mut propagator = Propagator::new(
            leo_initial_state(),
            Epoch::from_seconds(0.0).expect("epoch"),
            dt,
        )
        .expect("propagator")
        .with_sampling(SampleConfig::every_steps(2).expect("sample config"));

        let result = propagator
            .propagate_until(
                &integrator,
                &gravity,
                TerminationCondition::StepCount(4),
                mu,
            )
            .expect("propagation");

        assert!(result.samples.len() >= 2);
    }

    #[test]
    fn stops_on_apoapsis_of_transfer_orbit() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let departure_radius_m = R_EARTH + 400_000.0;
        let arrival_radius_m = R_EARTH + 35_786_000.0;
        let departure = Distance::from_meters(departure_radius_m).expect("distance");
        let arrival = Distance::from_meters(arrival_radius_m).expect("distance");
        let (dv1, _) =
            crate::physics::maneuvers::hohmann_delta_v(mu, departure, arrival).expect("delta-v");

        let orbit = EllipticOrbit::new(
            Distance::from_meters(departure_radius_m).expect("radius"),
            Eccentricity::from_scalar(0.0).expect("eccentricity"),
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0).expect("phase"),
            OrbitDirection::Prograde,
        )
        .expect("orbit");
        let mut propagator = Propagator::new(
            orbit.to_cartesian_state().expect("state"),
            Epoch::from_seconds(0.0).expect("epoch"),
            TimeStep::from_seconds(30.0).expect("dt"),
        )
        .expect("propagator");
        propagator
            .apply_impulsive_burn(
                ImpulsiveBurn::prograde(dv1, &propagator.current_state().expect("state"))
                    .expect("burn"),
            )
            .expect("burn");

        let gravity = TwoBodyGravity::new(mu);
        let integrator = LeapfrogIntegrator;
        let result = propagator
            .propagate_until(
                &integrator,
                &gravity,
                TerminationCondition::apoapsis(mu),
                mu,
            )
            .expect("propagation");

        assert_eq!(result.stop_reason, StopReason::ApoapsisMet);
        let final_radius = result.final_state.position.vector().norm();
        assert!(
            (final_radius - arrival_radius_m).abs() / arrival_radius_m < 0.03,
            "expected transfer apoapsis near GEO, got {final_radius}"
        );
    }
}
