use super::burns::BurnCommand;
use super::config::IntegratorKind;
use super::plan::{PropagationPlan, RuntimeDynamics};
use super::termination::{StopReason, TerminationCondition, TerminationTracker};
use crate::errors::PropagationError;
use crate::physics::dynamics::Dynamics;
use crate::physics::integrators::{
    EulerIntegrator, Integrator, LeapfrogIntegrator, RungeKutta4Integrator,
    VelocityVerletIntegrator,
};
use crate::physics::state::{CartesianState, CartesianStateBuffer, CartesianStateComponents};
use crate::physics::types::{Acceleration, Epoch, TimeStep};

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
    initial_epoch: Epoch,
    propagation: PropagationPlan,
    states: CartesianStateBuffer<1>,
}

impl Propagator {
    /// Creates a propagator starting from `initial_state` at the given epoch.
    pub(crate) fn new(
        initial_state: CartesianState,
        initial_epoch: Epoch,
        propagation: PropagationPlan,
    ) -> Result<Self, PropagationError> {
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &initial_state)?;

        Ok(Self {
            epoch: initial_epoch,
            initial_epoch,
            propagation,
            states,
        })
    }

    /// Resets epoch and state for a new episode.
    pub(crate) fn reset(&mut self, initial_state: CartesianState) -> Result<(), PropagationError> {
        self.epoch = self.initial_epoch;
        self.states.write_from_state(0, &initial_state)?;
        Ok(())
    }

    /// Reads the current body state at index 0.
    pub(crate) fn current_state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.read_state(0)?)
    }

    /// Applies a runtime burn command to body index 0.
    pub(crate) fn apply_burn(
        &mut self,
        burn: BurnCommand,
    ) -> Result<(), crate::errors::MissionError> {
        let state = self.current_state()?;
        let updated = burn.apply_to(state, self.propagation.mu())?;
        self.states
            .write_from_state(0, &updated)
            .map_err(PropagationError::from)?;
        Ok(())
    }

    /// Runs one propagation segment, collecting samples into the caller's sink.
    pub(crate) fn run_segment(
        &mut self,
        termination: &TerminationCondition,
        segment_index: Option<usize>,
    ) -> Result<PropagationResult, PropagationError> {
        let mut samples = Vec::new();
        let mut sampling = SegmentSampling {
            every_steps: self.propagation.sample_every_steps,
            segment_index,
            samples: &mut samples,
        };
        let result = self.advance(
            termination,
            None,
            AccelerationCommand::default(),
            Some(&mut sampling),
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
        command: AccelerationCommand,
    ) -> Result<AdvanceResult, PropagationError> {
        self.advance(termination, Some(max_steps), command, None)
    }

    fn advance(
        &mut self,
        termination: &TerminationCondition,
        max_steps: Option<usize>,
        command: AccelerationCommand,
        mut samples: Option<&mut SegmentSampling<'_>>,
    ) -> Result<AdvanceResult, PropagationError> {
        let segment_start_epoch = self.epoch;
        let mut steps = 0_usize;
        let mut current_state = self.states.read_state(0)?;
        let mut termination_tracker = TerminationTracker::new(termination, &current_state);

        if let Some(samples) = samples.as_deref_mut() {
            samples.record(self.epoch, current_state, steps)?;
        }

        loop {
            let context =
                termination_tracker.context(segment_start_epoch, self.epoch, steps, &current_state);

            if let Some(reason) = termination.stop_reason(context) {
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

            let step = match termination.remaining(segment_start_epoch, self.epoch) {
                Some(remaining) if remaining.seconds() < self.propagation.dt.seconds() => remaining,
                _ => self.propagation.dt,
            };

            self.step_once(step, command)?;
            self.epoch = self.epoch.advance_by(step);
            steps += 1;

            current_state = self.states.read_state(0)?;
            termination_tracker.advance(termination, &current_state);

            if let Some(samples) = samples.as_deref_mut() {
                samples.record(self.epoch, current_state, steps)?;
            }
        }
    }

    fn step_once(
        &mut self,
        step: TimeStep,
        command: AccelerationCommand,
    ) -> Result<(), PropagationError> {
        let integrator = self.propagation.integrator;
        let states = &mut self.states.view_mut();
        match &self.propagation.dynamics {
            RuntimeDynamics::TwoBody(gravity) => integrator.step(
                &ControlledDynamics {
                    dynamics: gravity,
                    command,
                },
                states,
                self.epoch,
                step,
            ),
            RuntimeDynamics::J2(gravity) => integrator.step(
                &ControlledDynamics {
                    dynamics: gravity,
                    command,
                },
                states,
                self.epoch,
                step,
            ),
            RuntimeDynamics::J2Drag(composite) => integrator.step(
                &ControlledDynamics {
                    dynamics: composite,
                    command,
                },
                states,
                self.epoch,
                step,
            ),
        }
    }
}

struct SegmentSampling<'a> {
    every_steps: usize,
    segment_index: Option<usize>,
    samples: &'a mut Vec<TrajectorySample>,
}

impl SegmentSampling<'_> {
    fn record(
        &mut self,
        epoch: Epoch,
        state: CartesianState,
        steps: usize,
    ) -> Result<(), PropagationError> {
        if steps == 0 || steps.is_multiple_of(self.every_steps) {
            self.samples
                .push(TrajectorySample::new(epoch, state, self.segment_index));
        }
        Ok(())
    }
}

/// Piecewise-constant inertial acceleration command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct AccelerationCommand {
    acceleration_mps2: [f64; 3],
}

impl AccelerationCommand {
    /// Creates an inertial acceleration command in meters per second squared.
    pub(crate) fn inertial(acceleration_mps2: [f64; 3]) -> Result<Self, PropagationError> {
        if !acceleration_mps2
            .iter()
            .all(|component| component.is_finite())
        {
            return Err(PropagationError::Numerical(
                crate::errors::NumericalError::NotFinite {
                    field: "action acceleration",
                },
            ));
        }
        Ok(Self { acceleration_mps2 })
    }
}

impl Default for AccelerationCommand {
    fn default() -> Self {
        Self {
            acceleration_mps2: [0.0, 0.0, 0.0],
        }
    }
}

/// Adds a piecewise-constant inertial acceleration command above an arbitrary
/// base dynamics model.
struct ControlledDynamics<'a, D: Dynamics + ?Sized> {
    dynamics: &'a D,
    command: AccelerationCommand,
}

impl<D: Dynamics + ?Sized> Dynamics for ControlledDynamics<'_, D> {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        epoch: Epoch,
    ) -> Result<Acceleration, crate::errors::DynamicsError> {
        let base = self.dynamics.acceleration_at(state, epoch)?.vector();
        let total = nalgebra::Vector3::new(
            base.x + self.command.acceleration_mps2[0],
            base.y + self.command.acceleration_mps2[1],
            base.z + self.command.acceleration_mps2[2],
        );
        Ok(Acceleration::from_vector(total))
    }
}

impl IntegratorKind {
    fn step<D: Dynamics>(
        self,
        dynamics: &D,
        states: &mut crate::physics::state::CartesianStatesMut<'_>,
        epoch: Epoch,
        dt: TimeStep,
    ) -> Result<(), PropagationError> {
        match self {
            Self::Leapfrog => LeapfrogIntegrator.step(dynamics, states, 0, epoch, dt),
            Self::VelocityVerlet => VelocityVerletIntegrator.step(dynamics, states, 0, epoch, dt),
            Self::Euler => EulerIntegrator.step(dynamics, states, 0, epoch, dt),
            Self::RungeKutta4 => RungeKutta4Integrator.step(dynamics, states, 0, epoch, dt),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::CentralBody;
    use crate::physics::constants::MU_EARTH;
    use crate::physics::dynamics::TwoBodyGravity;
    use crate::physics::orbits::Orbit;
    use crate::physics::types::{Distance, Duration, GravitationalParameter};

    fn propagation_plan(sample_every_steps: usize) -> PropagationPlan {
        PropagationPlan {
            dt: TimeStep::new(10.0),
            integrator: IntegratorKind::Leapfrog,
            dynamics: RuntimeDynamics::TwoBody(TwoBodyGravity::new(mu())),
            sample_every_steps,
        }
    }

    fn mu() -> GravitationalParameter {
        GravitationalParameter::new(MU_EARTH)
    }

    fn initial_state() -> CartesianState {
        let body = CentralBody::earth().expect("body");
        Orbit::circular_equatorial(Distance::new(400_000.0))
            .to_cartesian_state(&body)
            .expect("state")
    }

    #[test]
    fn run_segment_samples_at_configured_cadence() {
        let plan = propagation_plan(2);
        let mut propagator =
            Propagator::new(initial_state(), Epoch::ZERO, plan).expect("propagator");

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
        let mut propagator =
            Propagator::new(initial_state(), Epoch::ZERO, plan).expect("propagator");

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
        let mut propagator =
            Propagator::new(initial_state(), Epoch::ZERO, plan).expect("propagator");

        let result = propagator
            .advance_steps(
                &TerminationCondition::Duration(Duration::new(100.0)),
                2,
                AccelerationCommand::default(),
            )
            .expect("advance");

        assert_eq!(result.steps, 2);
        assert_eq!(result.stop_reason, None);
    }
}
