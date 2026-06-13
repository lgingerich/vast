use serde::{Deserialize, Serialize};

use super::propagator::{Propagator, SampleConfig};
use super::termination::TerminationCondition;
use crate::errors::MissionError;
use crate::physics::constants::MU_EARTH;
use crate::physics::dynamics::TwoBodyGravity;
use crate::physics::integrators::{
    EulerIntegrator, LeapfrogIntegrator, RungeKutta4Integrator, VelocityVerletIntegrator,
};
use crate::physics::orbits::EllipticOrbit;
use crate::physics::orbits::OrbitDirection;
use crate::physics::state::CartesianState;
use crate::physics::types::{
    Angle, Distance, Duration, Eccentricity, Epoch, GravitationalParameter, TimeStep,
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
    /// Integrator.
    pub integrator: IntegratorKind,
    /// Force model.
    pub force_model: ForceModelKind,
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
    pub(crate) fn build(&self) -> Result<BuiltScenario, MissionError> {
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

        let gravity = create_gravity(self.force_model)?;
        let mu = gravity.mu();
        let radius = Distance::from_meters(crate::physics::constants::R_EARTH + self.altitude_m)?;

        let orbit = EllipticOrbit::new(
            radius,
            Eccentricity::from_scalar(0.0)?,
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0)?,
            OrbitDirection::Prograde,
        )?;

        let initial_state = orbit.to_cartesian_state()?;
        let propagator = create_propagator(initial_state, dt, self.sample_every_steps)?;

        Ok(BuiltScenario {
            propagation: PropagationSetup {
                integrator: self.integrator,
                gravity,
                propagator,
                initial_state,
            },
            termination: TerminationCondition::Duration(duration),
        })
    }
}

/// Runtime propagation state shared by scenarios and missions.
#[derive(Debug)]
pub struct PropagationSetup {
    /// Selected integrator.
    pub integrator: IntegratorKind,
    /// Central gravity model.
    pub gravity: TwoBodyGravity,
    /// Propagator with initial state and sampling configured.
    pub propagator: Propagator,
    /// Initial Cartesian state for episode resets.
    pub initial_state: CartesianState,
}

/// Runtime scenario ready for propagation.
pub struct BuiltScenario {
    /// Shared propagation machine.
    pub propagation: PropagationSetup,
    /// Episode termination rule.
    pub termination: TerminationCondition,
}

/// Supported integrators for scenario dispatch.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum IntegratorKind {
    /// Leapfrog integrator.
    Leapfrog,
    /// Velocity Verlet integrator.
    VelocityVerlet,
    /// Semi-implicit Euler integrator.
    Euler,
    /// Fourth-order Runge-Kutta integrator.
    #[serde(alias = "rk4")]
    RungeKutta4,
}

/// Supported force models for scenario configuration.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ForceModelKind {
    /// Two-body point-mass gravity.
    TwoBody,
}

impl IntegratorKind {
    /// Shared integrator dispatch for propagators and mission segments.
    pub(crate) fn dispatch(
        self,
        propagator: &mut Propagator,
        gravity: &TwoBodyGravity,
        termination: TerminationCondition,
        segment_index: Option<usize>,
    ) -> Result<crate::sim::propagator::PropagationResult, crate::errors::PropagationError> {
        let mu = gravity.mu();
        match self {
            Self::Leapfrog => propagator.propagate_until_with_segment(
                &LeapfrogIntegrator,
                gravity,
                termination,
                mu,
                segment_index,
            ),
            Self::VelocityVerlet => propagator.propagate_until_with_segment(
                &VelocityVerletIntegrator,
                gravity,
                termination,
                mu,
                segment_index,
            ),
            Self::Euler => propagator.propagate_until_with_segment(
                &EulerIntegrator,
                gravity,
                termination,
                mu,
                segment_index,
            ),
            Self::RungeKutta4 => propagator.propagate_until_with_segment(
                &RungeKutta4Integrator,
                gravity,
                termination,
                mu,
                segment_index,
            ),
        }
    }
}

/// Builds a gravity model from a force model kind.
pub(super) fn create_gravity(force_model: ForceModelKind) -> Result<TwoBodyGravity, MissionError> {
    match force_model {
        ForceModelKind::TwoBody => {
            let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
            Ok(TwoBodyGravity::new(mu))
        }
    }
}

/// Builds a propagator with sampling from a Cartesian initial state.
pub(super) fn create_propagator(
    initial_state: CartesianState,
    dt: TimeStep,
    sample_every_steps: usize,
) -> Result<Propagator, MissionError> {
    Ok(
        Propagator::new(initial_state, Epoch::from_seconds(0.0)?, dt)?.with_sampling(
            SampleConfig::every_steps(sample_every_steps).ok_or_else(|| {
                MissionError::InvalidField {
                    field: "sample_every_steps",
                    reason: "must be >= 1".to_string(),
                }
            })?,
        ),
    )
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
        assert_eq!(built.propagation.integrator, IntegratorKind::Leapfrog);
    }
}
