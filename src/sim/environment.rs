use super::config::ScenarioConfig;
use super::engine::{AccelerationCommand, PropagationResult, Propagator, VecSampleSink};
use super::plan::RolloutPlan;
use super::termination::StopReason;
use crate::errors::{EnvironmentError, PropagationError};
use crate::physics::constants::R_EARTH;
use crate::physics::state::CartesianState;

/// Inertial thrust acceleration action in meters per second squared.
///
/// MVP accepts this input but does not yet apply it to propagation.
pub type Action = [f64; 3];

/// Flat observation of the environment state after a step.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Observation {
    /// Inertial Cartesian position in meters.
    pub position: [f64; 3],
    /// Inertial Cartesian velocity in meters per second.
    pub velocity: [f64; 3],
    /// Altitude above the Earth equatorial radius in meters.
    pub altitude_m: f64,
    /// Inertial speed in meters per second.
    pub speed_mps: f64,
}

/// Result of one environment step.
#[derive(Debug, Clone, PartialEq)]
pub struct StepResult {
    /// Observation after the step.
    pub observation: Observation,
    /// Reward for the transition (zero in MVP).
    pub reward: f64,
    /// Whether the episode finished.
    pub done: bool,
    /// Why propagation stopped, if this step reached a terminal condition.
    pub stop_reason: Option<StopReason>,
    /// Integrator steps advanced by this environment step.
    pub steps: usize,
}

/// Reinforcement-learning environment contract.
pub trait Environment {
    /// Resets the environment for a new episode.
    fn reset(&mut self, seed: u64) -> Result<Observation, EnvironmentError>;

    /// Applies one environment step.
    fn step(&mut self, action: Action) -> Result<StepResult, EnvironmentError>;
}

/// LEO circular-orbit rollout environment (MVP).
pub struct LeoCircularEnv {
    plan: RolloutPlan,
    propagator: Propagator,
    done: bool,
}

impl LeoCircularEnv {
    /// Builds an environment from a scenario configuration.
    pub fn from_config(config: &ScenarioConfig) -> Result<Self, EnvironmentError> {
        let plan = config.build()?;
        let initial_epoch =
            crate::physics::types::Epoch::from_seconds(0.0).map_err(PropagationError::from)?;
        let propagator = Propagator::new(plan.initial_state, initial_epoch, plan.propagation.dt)?;
        Ok(Self {
            plan,
            propagator,
            done: false,
        })
    }

    /// Runs a full rollout episode for artifact recording.
    pub fn run_episode(&mut self, seed: u64) -> Result<RolloutEpisodeResult, EnvironmentError> {
        let _ = self.reset(seed)?;
        let mut samples = VecSampleSink::new();
        let propagation = self.propagator.run_segment(
            self.plan.propagation,
            &self.plan.termination,
            None,
            &mut samples,
        )?;
        let mut propagation = PropagationResult {
            samples: samples.into_samples(),
            ..propagation
        };
        if propagation.samples.is_empty() {
            propagation
                .samples
                .push(super::engine::TrajectorySample::new(
                    propagation.final_epoch,
                    propagation.final_state,
                    None,
                ));
        }
        let observation = Observation::from_state(&propagation.final_state);
        self.done = true;
        Ok(RolloutEpisodeResult {
            observation,
            reward: 0.0,
            propagation,
        })
    }
}

impl Observation {
    pub(crate) fn from_state(state: &CartesianState) -> Self {
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let radius = position.norm();
        let altitude = radius - R_EARTH;
        let speed = velocity.norm();

        Self {
            position: [position.x, position.y, position.z],
            velocity: [velocity.x, velocity.y, velocity.z],
            altitude_m: altitude,
            speed_mps: speed,
        }
    }
}

/// Result of one full rollout episode for recording.
#[derive(Debug, Clone, PartialEq)]
pub struct RolloutEpisodeResult {
    /// Observation at the final episode state.
    pub observation: Observation,
    /// Episode reward (zero until rewards are implemented).
    pub reward: f64,
    /// Full propagation output including trajectory samples.
    pub propagation: PropagationResult,
}

impl Environment for LeoCircularEnv {
    fn reset(&mut self, _seed: u64) -> Result<Observation, EnvironmentError> {
        self.propagator.reset(self.plan.initial_state)?;
        self.done = false;
        Ok(Observation::from_state(&self.plan.initial_state))
    }

    fn step(&mut self, action: Action) -> Result<StepResult, EnvironmentError> {
        if self.done {
            let state = self.propagator.current_state()?;
            return Ok(StepResult {
                observation: Observation::from_state(&state),
                reward: 0.0,
                done: true,
                stop_reason: None,
                steps: 0,
            });
        }

        let command = AccelerationCommand::inertial(action)?;
        let result = self.propagator.advance_steps(
            self.plan.propagation,
            &self.plan.termination,
            self.plan.steps_per_env_step,
            command,
        )?;
        let done = result.stop_reason.is_some();
        self.done = done;

        Ok(StepResult {
            observation: Observation::from_state(&result.final_state),
            reward: 0.0,
            done,
            stop_reason: result.stop_reason,
            steps: result.steps,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::config::ScenarioConfig;

    #[test]
    fn reset_and_step_advances_bounded_horizon() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = "two_body"
            sample_every_steps = 5
            steps_per_env_step = 3
            "#,
        )
        .expect("scenario");

        let mut env = LeoCircularEnv::from_config(&config).expect("env");
        let _obs = env.reset(42).expect("reset");
        let result = env.step([0.0, 0.0, 0.0]).expect("step");

        assert!(!result.done);
        assert_eq!(result.stop_reason, None);
        assert_eq!(result.steps, 3);
    }

    #[test]
    fn run_episode_records_full_rollout() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = "two_body"
            sample_every_steps = 5
            "#,
        )
        .expect("scenario");

        let mut env = LeoCircularEnv::from_config(&config).expect("env");
        let result = env.run_episode(42).expect("episode");

        assert_eq!(result.propagation.stop_reason, StopReason::DurationMet);
        assert!(!result.propagation.samples.is_empty());
    }
}
