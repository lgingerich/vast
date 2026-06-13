use super::burns::BurnCommand;
use super::config::{
    BurnConfig, ForceModelKind, InitialOrbitConfig, IntegratorKind, MissionScenarioConfig,
    ScenarioConfig, StopConfig,
};
use super::termination::TerminationCondition;
use crate::errors::MissionError;
use crate::physics::constants::{MU_EARTH, R_EARTH};
use crate::physics::dynamics::TwoBodyGravity;
use crate::physics::elements::ClassicalOrbitalElements;
use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
use crate::physics::state::CartesianState;
use crate::physics::types::{
    Angle, Distance, Duration, Eccentricity, GravitationalParameter, TimeStep,
};

/// Validated runtime rollout scenario.
#[derive(Debug)]
pub(crate) struct RolloutPlan {
    pub(crate) initial_state: CartesianState,
    pub(crate) propagation: PropagationPlan,
    pub(crate) termination: TerminationCondition,
    pub(crate) steps_per_env_step: usize,
}

/// Validated runtime mission scenario.
#[derive(Debug)]
pub(crate) struct MissionPlan {
    pub(crate) initial_state: CartesianState,
    pub(crate) propagation: PropagationPlan,
    pub(crate) initial_burns: Vec<BurnCommand>,
    pub(crate) segments: Vec<MissionSegmentPlan>,
}

/// Runtime propagation settings shared by rollout and mission plans.
#[derive(Debug, Clone, Copy)]
pub(crate) struct PropagationPlan {
    pub(crate) dt: TimeStep,
    pub(crate) integrator: IntegratorKind,
    pub(crate) force_model: RuntimeForceModel,
    pub(crate) sample_every_steps: usize,
}

/// Runtime force model stack.
#[derive(Debug, Clone, Copy)]
pub(crate) enum RuntimeForceModel {
    TwoBody(GravitationalParameter),
}

impl RuntimeForceModel {
    pub(crate) const fn mu(self) -> GravitationalParameter {
        match self {
            Self::TwoBody(mu) => mu,
        }
    }

    pub(crate) fn dispatch<T>(
        self,
        mut run: impl FnMut(&TwoBodyGravity) -> Result<T, crate::errors::PropagationError>,
    ) -> Result<T, crate::errors::PropagationError> {
        match self {
            Self::TwoBody(mu) => run(&TwoBodyGravity::new(mu)),
        }
    }
}

/// Runtime mission segment.
#[derive(Debug, Clone, PartialEq)]
pub(crate) struct MissionSegmentPlan {
    pub(crate) termination: TerminationCondition,
    pub(crate) burns: Vec<BurnCommand>,
}

impl ScenarioConfig {
    /// Builds validated runtime objects for rollout execution.
    pub(crate) fn build(&self) -> Result<RolloutPlan, MissionError> {
        validate_name(&self.name)?;
        if self.duration_s <= 0.0 {
            return Err(MissionError::InvalidField {
                field: "duration_s",
                reason: "must be positive for rollouts".to_string(),
            });
        }
        if self.steps_per_env_step == 0 {
            return Err(MissionError::InvalidField {
                field: "steps_per_env_step",
                reason: "must be >= 1".to_string(),
            });
        }

        let propagation = propagation_plan(
            self.dt_s,
            self.integrator,
            self.force_model,
            self.sample_every_steps,
        )?;
        let mu = propagation.force_model.mu();
        let initial_state = circular_initial_state(self.altitude_m, mu)?;
        let duration = Duration::from_seconds(self.duration_s)?;

        Ok(RolloutPlan {
            initial_state,
            propagation,
            termination: TerminationCondition::Duration(duration),
            steps_per_env_step: self.steps_per_env_step,
        })
    }
}

impl MissionScenarioConfig {
    /// Builds validated runtime objects for mission execution.
    pub(crate) fn build(&self) -> Result<MissionPlan, MissionError> {
        validate_name(&self.name)?;
        if self.kind != "mission" {
            return Err(MissionError::InvalidField {
                field: "kind",
                reason: "must be `mission`".to_string(),
            });
        }
        if self.segments.is_empty() {
            return Err(MissionError::InvalidField {
                field: "segments",
                reason: "mission scenarios require at least one segment".to_string(),
            });
        }

        let propagation = propagation_plan(
            self.propagation.dt_s,
            self.propagation.integrator,
            self.propagation.force_model,
            self.propagation.sample_every_steps,
        )?;
        let mu = propagation.force_model.mu();
        let initial_state = initial_state(&self.initial_orbit, mu)?;
        let initial_burns = self
            .initial_burns
            .iter()
            .copied()
            .map(burn_command)
            .collect::<Result<Vec<_>, _>>()?;
        let segments = self
            .segments
            .iter()
            .map(|segment| {
                let termination = termination(&segment.stop, mu)?;
                let burns = segment
                    .burns
                    .iter()
                    .copied()
                    .map(burn_command)
                    .collect::<Result<Vec<_>, _>>()?;
                Ok(MissionSegmentPlan { termination, burns })
            })
            .collect::<Result<Vec<_>, MissionError>>()?;

        Ok(MissionPlan {
            initial_state,
            propagation,
            initial_burns,
            segments,
        })
    }
}

fn validate_name(name: &str) -> Result<(), MissionError> {
    if name.is_empty() {
        return Err(MissionError::InvalidField {
            field: "name",
            reason: "must not be empty".to_string(),
        });
    }
    Ok(())
}

fn propagation_plan(
    dt_s: f64,
    integrator: IntegratorKind,
    force_model: ForceModelKind,
    sample_every_steps: usize,
) -> Result<PropagationPlan, MissionError> {
    if sample_every_steps == 0 {
        return Err(MissionError::InvalidField {
            field: "sample_every_steps",
            reason: "must be >= 1".to_string(),
        });
    }
    Ok(PropagationPlan {
        dt: TimeStep::from_seconds(dt_s)?,
        integrator,
        force_model: runtime_force_model(force_model)?,
        sample_every_steps,
    })
}

fn runtime_force_model(force_model: ForceModelKind) -> Result<RuntimeForceModel, MissionError> {
    match force_model {
        ForceModelKind::TwoBody => {
            let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
            Ok(RuntimeForceModel::TwoBody(mu))
        }
    }
}

fn initial_state(
    initial_orbit: &InitialOrbitConfig,
    mu: GravitationalParameter,
) -> Result<CartesianState, MissionError> {
    match initial_orbit {
        InitialOrbitConfig::Circular { altitude_m } => circular_initial_state(*altitude_m, mu),
        InitialOrbitConfig::Classical {
            semi_major_axis_m,
            eccentricity,
            inclination_deg,
            raan_deg,
            arg_periapsis_deg,
            true_anomaly_deg,
        } => {
            let elements = ClassicalOrbitalElements::new(
                Distance::from_meters(*semi_major_axis_m)?,
                Eccentricity::from_scalar(*eccentricity)?,
                Angle::from_degrees(*inclination_deg)?,
                Angle::from_degrees(*raan_deg)?,
                Angle::from_degrees(*arg_periapsis_deg)?,
                Angle::from_degrees(*true_anomaly_deg)?,
            );
            Ok(elements.to_cartesian_state(mu)?)
        }
    }
}

fn circular_initial_state(
    altitude_m: f64,
    mu: GravitationalParameter,
) -> Result<CartesianState, MissionError> {
    let radius = Distance::from_meters(R_EARTH + altitude_m)?;
    let orbit = EllipticOrbit::new(
        radius,
        Eccentricity::from_scalar(0.0)?,
        mu,
        Angle::ZERO,
        Angle::ZERO,
        Angle::ZERO,
        Angle::ZERO,
        OrbitDirection::Prograde,
    )?;
    Ok(orbit.to_cartesian_state()?)
}

fn termination(
    stop: &StopConfig,
    mu: GravitationalParameter,
) -> Result<TerminationCondition, MissionError> {
    match stop {
        StopConfig::Apoapsis => Ok(TerminationCondition::apoapsis(mu)),
        StopConfig::Periapsis => Ok(TerminationCondition::periapsis(mu)),
        StopConfig::Duration { duration_s } => Ok(TerminationCondition::Duration(
            Duration::from_seconds(*duration_s)?,
        )),
        StopConfig::StepCount { steps } => Ok(TerminationCondition::StepCount(*steps)),
    }
}

fn burn_command(config: BurnConfig) -> Result<BurnCommand, MissionError> {
    match config {
        BurnConfig::Inertial { delta_v_mps } => BurnCommand::inertial(delta_v_mps),
        BurnConfig::Prograde { delta_v_mps } => BurnCommand::prograde(delta_v_mps),
        BurnConfig::Circularize => Ok(BurnCommand::circularize()),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rollout_toml(extra: &str) -> String {
        format!(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = "two_body"
            sample_every_steps = 5
            {extra}
            "#
        )
    }

    #[test]
    fn rollout_build_rejects_zero_env_horizon() {
        let config =
            ScenarioConfig::from_toml(&rollout_toml("steps_per_env_step = 0")).expect("config");

        let error = config.build().expect_err("zero horizon should fail");

        assert!(error.to_string().contains("steps_per_env_step"));
    }

    #[test]
    fn rollout_build_rejects_zero_sample_interval() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = "two_body"
            sample_every_steps = 0
            "#,
        )
        .expect("config");

        let error = config
            .build()
            .expect_err("zero sample interval should fail");

        assert!(error.to_string().contains("sample_every_steps"));
    }
}
