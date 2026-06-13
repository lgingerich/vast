use super::burns::BurnCommand;
use super::config::IntegratorKind;
use super::plan::{PropagationPlan, RuntimeForceModel};
use super::termination::{
    StopReason, TerminationCondition, TerminationContext, orbital_phase_angle_rad,
    radial_velocity_mps,
};
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

/// Receives trajectory samples from the engine.
pub(crate) trait SampleSink {
    fn record(
        &mut self,
        epoch: Epoch,
        state: CartesianState,
        segment_index: Option<usize>,
    ) -> Result<(), PropagationError>;
}

/// Caller-owned trajectory sample collector.
#[derive(Debug, Default)]
pub(crate) struct VecSampleSink {
    samples: Vec<TrajectorySample>,
}

impl VecSampleSink {
    pub(crate) const fn new() -> Self {
        Self {
            samples: Vec::new(),
        }
    }

    pub(crate) fn into_samples(self) -> Vec<TrajectorySample> {
        self.samples
    }
}

impl SampleSink for VecSampleSink {
    fn record(
        &mut self,
        epoch: Epoch,
        state: CartesianState,
        segment_index: Option<usize>,
    ) -> Result<(), PropagationError> {
        self.samples
            .push(TrajectorySample::new(epoch, state, segment_index));
        Ok(())
    }
}

/// Fixed-step propagator for a single indexed body in a state buffer.
#[derive(Debug)]
pub(crate) struct Propagator {
    epoch: Epoch,
    initial_epoch: Epoch,
    dt: TimeStep,
    states: CartesianStateBuffer<1>,
}

impl Propagator {
    /// Creates a propagator starting from `initial_state` at the given epoch.
    pub(crate) fn new(
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
        force_model: RuntimeForceModel,
    ) -> Result<(), crate::errors::MissionError> {
        let state = self.current_state()?;
        let resolved = burn.resolve(&state, force_model.mu())?;
        let updated = resolved.apply_to(state)?;
        self.states
            .write_from_state(0, &updated)
            .map_err(PropagationError::from)?;
        Ok(())
    }

    /// Runs one propagation segment, collecting samples into the caller's sink.
    pub(crate) fn run_segment(
        &mut self,
        propagation: PropagationPlan,
        termination: &TerminationCondition,
        segment_index: Option<usize>,
        samples: &mut impl SampleSink,
    ) -> Result<PropagationResult, PropagationError> {
        let mut sink = SegmentSampleSink {
            every_steps: propagation.sample_every_steps,
            segment_index,
            inner: samples,
        };
        let result = self.advance(
            propagation,
            termination,
            None,
            AccelerationCommand::default(),
            Some(&mut sink),
        )?;
        Ok(PropagationResult {
            final_epoch: result.final_epoch,
            final_state: result.final_state,
            steps: result.steps,
            stop_reason: result.stop_reason.unwrap_or(StopReason::StepBudgetExceeded),
            samples: Vec::new(),
        })
    }

    /// Advances at most `max_steps` for environment stepping.
    pub(crate) fn advance_steps(
        &mut self,
        propagation: PropagationPlan,
        termination: &TerminationCondition,
        max_steps: usize,
        command: AccelerationCommand,
    ) -> Result<AdvanceResult, PropagationError> {
        self.advance(propagation, termination, Some(max_steps), command, None)
    }

    fn advance(
        &mut self,
        propagation: PropagationPlan,
        termination: &TerminationCondition,
        max_steps: Option<usize>,
        command: AccelerationCommand,
        mut samples: Option<&mut SegmentSampleSink<'_>>,
    ) -> Result<AdvanceResult, PropagationError> {
        let segment_start_epoch = self.epoch;
        let mut steps = 0_usize;
        let mut current_state = self.states.read_state(0)?;
        let mut current_orbital_phase_angle_rad = termination
            .needs_orbital_phase()
            .then(|| orbital_phase_angle_rad(&current_state, propagation.force_model.mu()))
            .flatten();
        let mut current_radial_velocity_mps = radial_velocity_mps(&current_state);
        let mut previous_orbital_angle_rad = None;
        let mut previous_radial_velocity_mps = None;

        if let Some(samples) = samples.as_deref_mut() {
            samples.record(self.epoch, current_state, steps)?;
        }

        loop {
            let context = TerminationContext {
                start_epoch: segment_start_epoch,
                current_epoch: self.epoch,
                steps,
                state: &current_state,
                previous_orbital_angle_rad,
                previous_radial_velocity_mps,
                current_orbital_phase_angle_rad,
            };

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
                Some(remaining) if remaining.seconds() < self.dt.seconds() => remaining,
                _ => self.dt,
            };

            previous_orbital_angle_rad = current_orbital_phase_angle_rad;
            previous_radial_velocity_mps = Some(current_radial_velocity_mps);
            self.step_once(propagation, step, command)?;
            self.epoch = self.epoch.advance_by(step);
            steps += 1;

            current_state = self.states.read_state(0)?;
            if termination.needs_orbital_phase() {
                current_orbital_phase_angle_rad =
                    orbital_phase_angle_rad(&current_state, propagation.force_model.mu());
            }
            current_radial_velocity_mps = radial_velocity_mps(&current_state);

            if let Some(samples) = samples.as_deref_mut() {
                samples.record(self.epoch, current_state, steps)?;
            }
        }
    }

    fn step_once(
        &mut self,
        propagation: PropagationPlan,
        step: TimeStep,
        command: AccelerationCommand,
    ) -> Result<(), PropagationError> {
        propagation.force_model.dispatch(|gravity| {
            if command.is_zero() {
                step_with_integrator(
                    propagation.integrator,
                    gravity,
                    &mut self.states.view_mut(),
                    self.epoch,
                    step,
                )
            } else {
                let dynamics = ControlledDynamics { gravity, command };
                step_with_integrator(
                    propagation.integrator,
                    &dynamics,
                    &mut self.states.view_mut(),
                    self.epoch,
                    step,
                )
            }
        })
    }
}

struct SegmentSampleSink<'a> {
    every_steps: usize,
    segment_index: Option<usize>,
    inner: &'a mut dyn SampleSink,
}

impl SegmentSampleSink<'_> {
    fn record(
        &mut self,
        epoch: Epoch,
        state: CartesianState,
        steps: usize,
    ) -> Result<(), PropagationError> {
        if steps == 0 || steps.is_multiple_of(self.every_steps) {
            self.inner.record(epoch, state, self.segment_index)?;
        }
        Ok(())
    }
}

/// Piecewise-constant inertial acceleration command.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AccelerationCommand {
    acceleration_mps2: [f64; 3],
}

impl AccelerationCommand {
    /// Creates an inertial acceleration command in meters per second squared.
    pub fn inertial(acceleration_mps2: [f64; 3]) -> Result<Self, PropagationError> {
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

    const fn is_zero(self) -> bool {
        self.acceleration_mps2[0] == 0.0
            && self.acceleration_mps2[1] == 0.0
            && self.acceleration_mps2[2] == 0.0
    }
}

impl Default for AccelerationCommand {
    fn default() -> Self {
        Self {
            acceleration_mps2: [0.0, 0.0, 0.0],
        }
    }
}

struct ControlledDynamics<'a> {
    gravity: &'a crate::physics::dynamics::TwoBodyGravity,
    command: AccelerationCommand,
}

impl Dynamics for ControlledDynamics<'_> {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        epoch: Epoch,
    ) -> Result<Acceleration, crate::errors::DynamicsError> {
        let gravity = self.gravity.acceleration_at(state, epoch)?.vector();
        let total = nalgebra::Vector3::new(
            gravity.x + self.command.acceleration_mps2[0],
            gravity.y + self.command.acceleration_mps2[1],
            gravity.z + self.command.acceleration_mps2[2],
        );
        Ok(Acceleration::from_vector(total)?)
    }
}

fn step_with_integrator<D: Dynamics>(
    integrator: IntegratorKind,
    dynamics: &D,
    states: &mut crate::physics::state::CartesianStatesMut<'_>,
    epoch: Epoch,
    dt: TimeStep,
) -> Result<(), PropagationError> {
    match integrator {
        IntegratorKind::Leapfrog => LeapfrogIntegrator.step(dynamics, states, 0, epoch, dt),
        IntegratorKind::VelocityVerlet => {
            VelocityVerletIntegrator.step(dynamics, states, 0, epoch, dt)
        }
        IntegratorKind::Euler => EulerIntegrator.step(dynamics, states, 0, epoch, dt),
        IntegratorKind::RungeKutta4 => RungeKutta4Integrator.step(dynamics, states, 0, epoch, dt),
    }
}
