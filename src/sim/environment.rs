use super::propagator::PropagationResult;
use super::scenario::{BuiltScenario, ScenarioConfig};
use super::termination::StopReason;
use crate::errors::EnvironmentError;
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
    /// Why propagation stopped.
    pub stop_reason: StopReason,
    /// Full propagation output including trajectory samples.
    pub propagation: PropagationResult,
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
    scenario: BuiltScenario,
    seed: u64,
}

impl LeoCircularEnv {
    /// Builds an environment from a scenario configuration.
    pub fn from_config(config: &ScenarioConfig) -> Result<Self, EnvironmentError> {
        let scenario = config.build()?;
        Ok(Self { scenario, seed: 0 })
    }

    fn observation_from_state(
        &self,
        state: &CartesianState,
    ) -> Result<Observation, EnvironmentError> {
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let radius = position.norm();
        let altitude = radius - R_EARTH;
        let speed = velocity.norm();

        Ok(Observation {
            position: [position.x, position.y, position.z],
            velocity: [velocity.x, velocity.y, velocity.z],
            altitude_m: altitude,
            speed_mps: speed,
        })
    }
}

impl Environment for LeoCircularEnv {
    fn reset(&mut self, seed: u64) -> Result<Observation, EnvironmentError> {
        self.seed = seed;
        let initial_state = self.scenario.initial_state;
        self.scenario.propagator.reset(initial_state)?;
        self.observation_from_state(&initial_state)
    }

    fn step(&mut self, _action: Action) -> Result<StepResult, EnvironmentError> {
        let integrator = self.scenario.integrator;
        let propagation = integrator.propagate(&mut self.scenario)?;
        let observation = self.observation_from_state(&propagation.final_state)?;
        let done = matches!(
            propagation.stop_reason,
            StopReason::DurationMet | StopReason::StepCountMet
        );

        Ok(StepResult {
            observation,
            reward: 0.0,
            done,
            stop_reason: propagation.stop_reason,
            propagation,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::sim::scenario::ScenarioConfig;

    #[test]
    fn reset_and_step_one_episode() {
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
        let _obs = env.reset(42).expect("reset");
        let result = env.step([0.0, 0.0, 0.0]).expect("step");

        assert!(result.done);
        assert_eq!(result.stop_reason, StopReason::DurationMet);
        assert!(!result.propagation.samples.is_empty());
    }
}
