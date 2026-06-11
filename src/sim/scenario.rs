use serde::{Deserialize, Serialize};

use super::propagator::{Propagator, SampleConfig};
use super::termination::TerminationCondition;
use crate::errors::MissionError;
use crate::physics::constants::{MU_EARTH, R_EARTH};
use crate::physics::dynamics::TwoBodyGravity;
use crate::physics::integrators::{
    EulerIntegrator, LeapfrogIntegrator, RungeKutta4Integrator, VelocityVerletIntegrator,
};
use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
use crate::physics::state::CartesianState;
use crate::physics::types::{
    Angle, Distance, Duration, Eccentricity, GravitationalParameter, TimeStep,
};

/// Serializable scenario definition loaded from TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct ScenarioConfig {
    /// Human-readable scenario name.
    pub name: String,
    /// Circular orbit altitude above the equatorial radius, in meters.
    pub altitude_m: f64,
    /// Fixed integrator step size in seconds.
    pub dt_s: f64,
    /// Episode duration in seconds.
    pub duration_s: f64,
    /// Integrator name (`leapfrog`, `verlet`, `euler`, `rk4`).
    pub integrator: String,
    /// Force model name (`two_body` only in MVP).
    pub force_model: String,
    /// Sample trajectory every N integrator steps.
    pub sample_every_steps: usize,
}

impl ScenarioConfig {
    /// Loads a scenario from a TOML file path.
    pub fn from_file(path: &std::path::Path) -> Result<Self, MissionError> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_toml(&contents)
    }

    /// Parses a scenario from a TOML string.
    pub fn from_toml(contents: &str) -> Result<Self, MissionError> {
        Ok(toml::from_str(contents)?)
    }

    /// Builds runtime objects for propagation.
    pub fn build(&self) -> Result<BuiltScenario, MissionError> {
        if self.name.is_empty() {
            return Err(MissionError::InvalidField {
                field: "name",
                reason: "must not be empty".to_string(),
            });
        }

        let dt = TimeStep::from_seconds(self.dt_s)?;
        let duration = Duration::from_seconds(self.duration_s)?;
        if self.duration_s <= 0.0 {
            return Err(MissionError::InvalidField {
                field: "duration_s",
                reason: "must be positive for MVP rollouts".to_string(),
            });
        }

        let sample_every_steps = (self.sample_every_steps >= 1)
            .then_some(self.sample_every_steps)
            .ok_or_else(|| MissionError::InvalidField {
                field: "sample_every_steps",
                reason: "must be >= 1".to_string(),
            })?;

        let integrator = parse_integrator(&self.integrator)?;
        let gravity = parse_force_model(&self.force_model)?;

        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
        let radius = Distance::from_meters(R_EARTH + self.altitude_m)?;

        let orbit = EllipticOrbit {
            semi_major_axis: radius,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        };

        let initial_state =
            orbit
                .to_cartesian_state()
                .ok_or_else(|| MissionError::InvalidField {
                    field: "altitude_m",
                    reason: "initial Cartesian state is invalid".to_string(),
                })?;

        let propagator = Propagator::new(initial_state, dt)?.with_sampling(
            SampleConfig::every_steps(sample_every_steps).ok_or_else(|| {
                MissionError::InvalidField {
                    field: "sample_every_steps",
                    reason: "invalid sample config".to_string(),
                }
            })?,
        );

        Ok(BuiltScenario {
            config: self.clone(),
            integrator,
            gravity,
            propagator,
            initial_state,
            termination: TerminationCondition::Duration(duration),
            earth_radius_m: R_EARTH,
        })
    }
}

pub(crate) fn parse_integrator(name: &str) -> Result<IntegratorKind, MissionError> {
    match name {
        "leapfrog" => Ok(IntegratorKind::Leapfrog),
        "verlet" => Ok(IntegratorKind::VelocityVerlet),
        "euler" => Ok(IntegratorKind::Euler),
        "rk4" => Ok(IntegratorKind::RungeKutta4),
        other => Err(MissionError::UnknownIntegrator(other.to_string())),
    }
}

pub(crate) fn parse_force_model(name: &str) -> Result<TwoBodyGravity, MissionError> {
    match name {
        "two_body" => {
            let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
            Ok(TwoBodyGravity { mu })
        }
        other => Err(MissionError::UnknownForceModel(other.to_string())),
    }
}

/// Runtime scenario ready for propagation.
pub struct BuiltScenario {
    /// Source configuration.
    pub config: ScenarioConfig,
    /// Selected integrator.
    pub integrator: IntegratorKind,
    /// Central gravity model.
    pub gravity: TwoBodyGravity,
    /// Propagator with initial state and sampling configured.
    pub propagator: Propagator,
    /// Initial Cartesian state for episode resets.
    pub initial_state: CartesianState,
    /// Episode termination rule.
    pub termination: TerminationCondition,
    /// Earth equatorial radius in meters.
    pub earth_radius_m: f64,
}

/// Supported integrators for scenario dispatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntegratorKind {
    /// Leapfrog integrator.
    Leapfrog,
    /// Velocity Verlet integrator.
    VelocityVerlet,
    /// Semi-implicit Euler integrator.
    Euler,
    /// Fourth-order Runge-Kutta integrator.
    RungeKutta4,
}

impl IntegratorKind {
    /// Propagates using the selected integrator implementation.
    pub fn propagate(
        self,
        scenario: &mut BuiltScenario,
    ) -> Result<crate::sim::propagator::PropagationResult, crate::errors::PropagationError> {
        match self {
            Self::Leapfrog => scenario.propagator.propagate_until(
                &LeapfrogIntegrator,
                &scenario.gravity,
                scenario.termination.clone(),
                scenario.gravity.mu,
            ),
            Self::VelocityVerlet => scenario.propagator.propagate_until(
                &VelocityVerletIntegrator,
                &scenario.gravity,
                scenario.termination.clone(),
                scenario.gravity.mu,
            ),
            Self::Euler => scenario.propagator.propagate_until(
                &EulerIntegrator,
                &scenario.gravity,
                scenario.termination.clone(),
                scenario.gravity.mu,
            ),
            Self::RungeKutta4 => scenario.propagator.propagate_until(
                &RungeKutta4Integrator,
                &scenario.gravity,
                scenario.termination.clone(),
                scenario.gravity.mu,
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_leo_scenario_toml() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "leo_circular"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 5400.0
            integrator = "leapfrog"
            force_model = "two_body"
            sample_every_steps = 10
            "#,
        )
        .expect("scenario");

        assert_eq!(config.name, "leo_circular");
        let built = config.build().expect("built scenario");
        assert_eq!(built.integrator, IntegratorKind::Leapfrog);
    }
}
