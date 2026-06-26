use super::config::BurnConfig;
use super::plan::PropagationPlan;
use super::termination::{StopReason, TerminationCondition, TerminationTracker};
use crate::errors::PropagationError;
use crate::physics::dynamics::ThrustWrapper;
use crate::physics::frame::Frame;
use crate::physics::state::{CartesianState, CartesianStateBuffer};
use crate::physics::thrust::ThrustMode;
use crate::physics::types::{Epoch, TimeStep, Velocity};

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

/// Result of a bounded environment step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AdvanceResult {
    pub(crate) final_epoch: Epoch,
    pub(crate) final_state: CartesianState,
    pub(crate) steps: usize,
    pub(crate) stop_reason: Option<StopReason>,
}

/// Fixed-step propagator for a single indexed body in a state buffer.
#[derive(Debug)]
pub(crate) struct Propagator {
    epoch: Epoch,
    propagation: PropagationPlan,
    states: CartesianStateBuffer<1>,
    thrust_mode: ThrustMode,
}

impl Propagator {
    /// Creates a propagator starting from `initial_state` at epoch zero.
    pub(crate) fn new(
        initial_state: CartesianState,
        frame: Frame,
        propagation: PropagationPlan,
    ) -> Result<Self, PropagationError> {
        frame.require_inertial_axes()?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &initial_state)?;

        Ok(Self {
            epoch: Epoch::ZERO,
            propagation,
            states,
            thrust_mode: ThrustMode::coast(),
        })
    }

    /// Resets epoch and state for a new episode.
    pub(crate) fn reset(&mut self, initial_state: CartesianState) -> Result<(), PropagationError> {
        self.epoch = Epoch::ZERO;
        self.states.write_from_state(0, &initial_state)?;
        self.thrust_mode = ThrustMode::coast();
        Ok(())
    }

    /// Reads the current body state at index 0.
    pub(crate) fn current_state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.read_state(0)?)
    }

    /// Sets the thrust mode for the next advance.
    pub(crate) fn set_thrust(&mut self, mode: ThrustMode) {
        self.thrust_mode = mode;
    }

    /// Applies an impulsive burn to body index 0.
    pub(crate) fn apply_burn(&mut self, mode: ThrustMode) -> Result<(), PropagationError> {
        let state = self.current_state()?;
        let dv = mode.resolve_delta_v(state)?;
        let new_vel = Velocity::from_vector(state.velocity.vector() + dv.vector());
        let updated = CartesianState::new(state.position, new_vel);
        self.states.write_from_state(0, &updated)?;
        Ok(())
    }

    /// Resolves a burn configuration against the current state and applies it.
    pub(crate) fn apply_burn_config(&mut self, config: BurnConfig) -> Result<(), PropagationError> {
        match config {
            BurnConfig::Inertial { delta_v_mps } => {
                self.apply_burn(ThrustMode::inertial(delta_v_mps))
            }
            BurnConfig::Prograde { delta_v_mps } => {
                self.apply_burn(ThrustMode::prograde(delta_v_mps))
            }
        }
    }

    /// Runs one propagation segment, collecting samples into the caller's sink.
    pub(crate) fn run_segment(
        &mut self,
        termination: &TerminationCondition,
        segment_index: Option<usize>,
    ) -> Result<PropagationResult, PropagationError> {
        let mut samples = Vec::new();
        let result = self.advance(
            termination,
            None,
            segment_index,
            Some(&mut samples),
            self.epoch,
        )?;
        let Some(stop_reason) = result.stop_reason else {
            return Err(PropagationError::SegmentWithoutStop);
        };
        Ok(PropagationResult {
            final_epoch: result.final_epoch,
            final_state: result.final_state,
            steps: result.steps,
            stop_reason,
            samples,
        })
    }

    /// Advances at most `max_steps` for environment stepping.
    pub(crate) fn advance_steps(
        &mut self,
        termination: &TerminationCondition,
        max_steps: usize,
    ) -> Result<AdvanceResult, PropagationError> {
        self.advance(termination, Some(max_steps), None, None, Epoch::ZERO)
    }

    fn advance(
        &mut self,
        termination: &TerminationCondition,
        max_steps: Option<usize>,
        segment_index: Option<usize>,
        mut samples: Option<&mut Vec<TrajectorySample>>,
        termination_start_epoch: Epoch,
    ) -> Result<AdvanceResult, PropagationError> {
        let mut steps = 0_usize;
        let mu = self.propagation.mu();
        let mut current_state = self.states.read_state(0)?;
        let mut termination_tracker = TerminationTracker::new(&current_state, mu);

        if let Some(samples) = samples.as_deref_mut() {
            samples.push(TrajectorySample::new(
                self.epoch,
                current_state,
                segment_index,
            ));
        }

        loop {
            let context = termination_tracker.context(
                termination_start_epoch,
                self.epoch,
                steps,
                &current_state,
            );

            if let Some(reason) = termination.stop_reason(context, mu) {
                return Ok(AdvanceResult {
                    final_epoch: self.epoch,
                    final_state: current_state,
                    steps,
                    stop_reason: Some(reason),
                });
            }

            if let Some(max_steps) = max_steps
                && steps >= max_steps
            {
                return Ok(AdvanceResult {
                    final_epoch: self.epoch,
                    final_state: current_state,
                    steps,
                    stop_reason: None,
                });
            }

            let step = match termination.remaining(termination_start_epoch, self.epoch) {
                Some(remaining) if remaining.seconds() < self.propagation.dt.seconds() => remaining,
                _ => self.propagation.dt,
            };

            self.step_once(step)?;
            self.epoch = self.epoch.advance_by(step);
            steps += 1;

            current_state = self.states.read_state(0)?;
            termination_tracker.advance(&current_state, mu);

            if let Some(samples) = samples.as_deref_mut()
                && steps.is_multiple_of(self.propagation.sample_every_steps)
            {
                samples.push(TrajectorySample::new(
                    self.epoch,
                    current_state,
                    segment_index,
                ));
            }
        }
    }

    fn step_once(&mut self, step: TimeStep) -> Result<(), PropagationError> {
        self.propagation.integrator.step(
            &ThrustWrapper {
                base: &self.propagation.dynamics,
                mode: &self.thrust_mode,
            },
            &mut self.states.view_mut(),
            0,
            self.epoch,
            step,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::MU_EARTH;
    use crate::physics::dynamics::{RuntimeDynamics, TwoBodyGravity};
    use crate::physics::frame::Frame;
    use crate::physics::integrators::IntegratorKind;
    use crate::physics::orbits::Orbit;
    use crate::physics::types::{Distance, Duration, GravitationalParameter};

    fn propagation_plan(sample_every_steps: usize) -> PropagationPlan {
        PropagationPlan {
            dt: TimeStep::new(10.0),
            integrator: IntegratorKind::Leapfrog,
            dynamics: RuntimeDynamics::TwoBody(TwoBodyGravity::new(mu())),
            mu: mu(),
            sample_every_steps,
        }
    }

    fn mu() -> GravitationalParameter {
        GravitationalParameter::new(MU_EARTH)
    }

    fn frame() -> Frame {
        Frame::earth_inertial()
    }

    fn initial_state() -> CartesianState {
        Orbit::circular_equatorial(Distance::new(400_000.0))
            .to_cartesian_state(&frame())
            .expect("state")
    }

    #[test]
    fn run_segment_samples_at_configured_cadence() {
        let plan = propagation_plan(2);
        let mut propagator = Propagator::new(initial_state(), frame(), plan).expect("propagator");

        let result = propagator
            .run_segment(&TerminationCondition::StepCount(5), Some(7))
            .expect("segment");

        assert_eq!(result.steps, 5);
        assert_eq!(result.stop_reason, StopReason::StepCountMet);
        assert_eq!(result.samples.len(), 3);
        assert!(
            result
                .samples
                .iter()
                .all(|sample| sample.segment_index == Some(7))
        );
    }

    #[test]
    fn duration_segment_uses_partial_final_step() {
        let plan = propagation_plan(1);
        let mut propagator = Propagator::new(initial_state(), frame(), plan).expect("propagator");

        let result = propagator
            .run_segment(&TerminationCondition::Duration(Duration::new(25.0)), None)
            .expect("segment");

        assert_eq!(result.steps, 3);
        assert_eq!(result.stop_reason, StopReason::DurationMet);
        assert!((result.final_epoch.seconds() - 25.0).abs() < 1.0e-12);
    }

    #[test]
    fn bounded_advance_can_end_without_terminal_stop() {
        let plan = propagation_plan(1);
        let mut propagator = Propagator::new(initial_state(), frame(), plan).expect("propagator");

        let result = propagator
            .advance_steps(&TerminationCondition::Duration(Duration::new(100.0)), 2)
            .expect("advance");

        assert_eq!(result.steps, 2);
        assert_eq!(result.stop_reason, None);
    }
}
