use super::burns::BurnCommand;
use super::config::{
    BurnConfig, ForceModelKind, InitialOrbitConfig, IntegratorKind, MissionScenarioConfig,
    ScenarioConfig, StopConfig,
};
use super::termination::TerminationCondition;
use crate::errors::MissionError;
use crate::physics::dynamics::{
    AtmosphericDrag, CompositeDynamics, ExponentialAtmosphere, J2Gravity, TwoBodyGravity,
};
use crate::physics::elements::{ClassicalOrbitalElements, OrbitalElements};
use crate::physics::frame::Frame;
use crate::physics::orbits::Orbit;
use crate::physics::state::CartesianState;
use crate::physics::types::{
    Angle, AreaPerMass, Distance, Duration, Eccentricity, GravitationalParameter, TimeStep,
};

/// Validated runtime rollout scenario.
#[derive(Debug)]
pub(crate) struct RolloutPlan {
    pub(crate) initial_state: CartesianState,
    pub(crate) frame: Frame,
    pub(crate) equatorial_radius: Distance,
    pub(crate) propagation: PropagationPlan,
    pub(crate) termination: TerminationCondition,
    pub(crate) steps_per_env_step: usize,
}

/// Validated runtime mission scenario.
#[derive(Debug)]
pub(crate) struct MissionPlan {
    pub(crate) initial_state: CartesianState,
    pub(crate) frame: Frame,
    pub(crate) propagation: PropagationPlan,
    pub(crate) initial_burns: Vec<BurnCommand>,
    pub(crate) segments: Vec<MissionSegmentPlan>,
}

/// Runtime propagation settings shared by rollout and mission plans.
#[derive(Debug)]
pub(crate) struct PropagationPlan {
    pub(crate) dt: TimeStep,
    pub(crate) integrator: IntegratorKind,
    pub(crate) dynamics: RuntimeDynamics,
    pub(crate) sample_every_steps: usize,
}

impl PropagationPlan {
    /// Returns the gravitational parameter of the configured force model.
    pub(crate) const fn mu(&self) -> GravitationalParameter {
        self.dynamics.mu()
    }
}

/// Resolved force model used by the propagator.
///
/// Closed enum so the stepping path can dispatch without trait objects and
/// stay allocation-free. Each variant owns its fully-validated dynamics model.
#[derive(Debug)]
pub(crate) enum RuntimeDynamics {
    /// Point-mass central gravity.
    TwoBody(TwoBodyGravity),
    /// Central gravity with J2 perturbation.
    J2(J2Gravity),
    /// J2 gravity composed with exponential atmospheric drag.
    J2Drag(CompositeDynamics<J2Gravity, AtmosphericDrag>),
}

impl RuntimeDynamics {
    /// Returns the gravitational parameter of the central body.
    pub(crate) const fn mu(&self) -> GravitationalParameter {
        match self {
            Self::TwoBody(gravity) => gravity.mu(),
            Self::J2(gravity) => gravity.mu(),
            Self::J2Drag(composite) => composite.first().mu(),
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

        let frame = self.body.inertial_frame();
        let propagation = propagation_plan(
            self.dt_s,
            self.integrator,
            &frame,
            &self.force_model,
            self.sample_every_steps,
        )?;
        let initial_state = circular_initial_state(self.altitude_m, &frame)?;
        let duration = Duration::new(self.duration_s);

        Ok(RolloutPlan {
            initial_state,
            frame,
            equatorial_radius: Distance::new(frame.equatorial_radius()?.meters()),
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

        let frame = self.propagation.body.inertial_frame();
        let propagation = propagation_plan(
            self.propagation.dt_s,
            self.propagation.integrator,
            &frame,
            &self.propagation.force_model,
            self.propagation.sample_every_steps,
        )?;
        let mu = propagation.mu();
        let initial_state = initial_state(&self.initial_orbit, &frame)?;
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
            frame,
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
    frame: &Frame,
    force_model: &ForceModelKind,
    sample_every_steps: usize,
) -> Result<PropagationPlan, MissionError> {
    if sample_every_steps == 0 {
        return Err(MissionError::InvalidField {
            field: "sample_every_steps",
            reason: "must be >= 1".to_string(),
        });
    }
    Ok(PropagationPlan {
        dt: TimeStep::new(dt_s),
        integrator,
        dynamics: runtime_dynamics(force_model, frame)?,
        sample_every_steps,
    })
}

/// Resolves a configured force model into validated runtime dynamics.
fn runtime_dynamics(
    force_model: &ForceModelKind,
    frame: &Frame,
) -> Result<RuntimeDynamics, MissionError> {
    match force_model {
        ForceModelKind::TwoBody => Ok(RuntimeDynamics::TwoBody(TwoBodyGravity::from_frame(frame)?)),
        ForceModelKind::J2 => Ok(RuntimeDynamics::J2(J2Gravity::from_frame(frame)?)),
        ForceModelKind::J2Drag {
            area_per_mass_m2_kg,
        } => {
            let atmosphere = ExponentialAtmosphere::from_frame(frame)?;
            let drag = AtmosphericDrag::new(atmosphere, AreaPerMass::new(*area_per_mass_m2_kg));
            let gravity = J2Gravity::from_frame(frame)?;
            Ok(RuntimeDynamics::J2Drag(CompositeDynamics::new(
                gravity, drag,
            )))
        }
    }
}

fn initial_state(
    initial_orbit: &InitialOrbitConfig,
    frame: &Frame,
) -> Result<CartesianState, MissionError> {
    match initial_orbit {
        InitialOrbitConfig::Circular { altitude_m } => circular_initial_state(*altitude_m, frame),
        InitialOrbitConfig::Classical {
            semi_major_axis_m,
            eccentricity,
            inclination_deg,
            raan_deg,
            arg_periapsis_deg,
            true_anomaly_deg,
        } => {
            let elements = ClassicalOrbitalElements::new(
                Distance::new(*semi_major_axis_m),
                Eccentricity::new(*eccentricity),
                Angle::from_degrees(*inclination_deg),
                Angle::from_degrees(*raan_deg),
                Angle::from_degrees(*arg_periapsis_deg),
                Angle::from_degrees(*true_anomaly_deg),
            );
            Ok(OrbitalElements::Classical(elements).to_cartesian_state(frame)?)
        }
    }
}

fn circular_initial_state(altitude_m: f64, frame: &Frame) -> Result<CartesianState, MissionError> {
    let orbit = Orbit::circular_equatorial(Distance::new(altitude_m));
    Ok(orbit.to_cartesian_state(frame)?)
}

fn termination(
    stop: &StopConfig,
    mu: GravitationalParameter,
) -> Result<TerminationCondition, MissionError> {
    match stop {
        StopConfig::Apoapsis => Ok(TerminationCondition::apoapsis(mu)),
        StopConfig::Periapsis => Ok(TerminationCondition::periapsis(mu)),
        StopConfig::Duration { duration_s } => {
            Ok(TerminationCondition::Duration(Duration::new(*duration_s)))
        }
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
            force_model = { kind = "two_body" }
            sample_every_steps = 0
            "#,
        )
        .expect("config");

        let error = config
            .build()
            .expect_err("zero sample interval should fail");

        assert!(error.to_string().contains("sample_every_steps"));
    }

    #[test]
    fn j2_force_model_resolves_to_j2_gravity() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "j2_test"
            body = "earth"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = { kind = "j2" }
            sample_every_steps = 5
            "#,
        )
        .expect("config");

        let plan = config.build().expect("plan");
        assert!(matches!(plan.propagation.dynamics, RuntimeDynamics::J2(_)));
    }

    #[test]
    fn j2_drag_force_model_resolves_to_composite() {
        let config = ScenarioConfig::from_toml(
            r#"
            name = "drag_test"
            body = "earth"
            altitude_m = 250_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "runge_kutta4"
            force_model = { kind = "j2_drag", area_per_mass_m2_kg = 0.02 }
            sample_every_steps = 5
            "#,
        )
        .expect("config");

        let plan = config.build().expect("plan");
        assert!(matches!(
            plan.propagation.dynamics,
            RuntimeDynamics::J2Drag(_)
        ));
    }

    #[test]
    fn j2_scenario_precesses_raan_relative_to_two_body() {
        // An inclined orbit under J2 should precess its RAAN; under pure
        // two-body the RAAN is fixed. Propagating both for several orbits
        // and comparing the final RAAN distinguishes the two models.
        let inclination_deg = 51.6_f64;
        let two_body_final_raan = final_raan_deg(inclination_deg, "{ kind = \"two_body\" }");
        let j2_final_raan = final_raan_deg(inclination_deg, "{ kind = \"j2\" }");

        assert!(
            (two_body_final_raan - j2_final_raan).abs() > 0.1,
            "J2 RAAN {j2_final_raan} deg should differ from two-body {two_body_final_raan} deg"
        );
    }

    #[test]
    fn drag_scenario_loses_energy_relative_to_two_body() {
        // Drag dissipates orbital energy; two-body conserves it. Propagating
        // a low orbit under both models for several orbits and comparing the
        // specific energy change separates them.
        let two_body_energy_delta = energy_delta("{ kind = \"two_body\" }");
        let drag_energy_delta = energy_delta("{ kind = \"j2_drag\", area_per_mass_m2_kg = 0.02 }");

        assert!(
            two_body_energy_delta.abs() < 0.01,
            "two-body energy should be conserved, got delta {two_body_energy_delta}"
        );
        assert!(
            drag_energy_delta < -1.0e-3,
            "drag should lose energy, got delta {drag_energy_delta}"
        );
    }

    fn rollout_toml(extra: &str) -> String {
        format!(
            r#"
            name = "test"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = {{ kind = "two_body" }}
            sample_every_steps = 5
            {extra}
            "#
        )
    }

    /// Propagates an inclined circular orbit under the given force model and
    /// returns the final RAAN in degrees.
    fn final_raan_deg(inclination_deg: f64, force_model_inline: &str) -> f64 {
        use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};

        let mission_toml = format!(
            r#"
            name = "raan"
            kind = "mission"

            [initial_orbit]
            kind = "classical"
            semi_major_axis_m = {semi_major_axis}
            eccentricity = 0.0
            inclination_deg = {inclination_deg}
            raan_deg = 0.0
            arg_periapsis_deg = 0.0
            true_anomaly_deg = 0.0

            [propagation]
            dt_s = 10.0
            integrator = "leapfrog"
            force_model = {force_model_inline}
            sample_every_steps = 1000

            [[segments]]
            stop = {{ kind = "duration", duration_s = 30_000.0 }}
            "#,
            semi_major_axis = 6_378_137.0_f64 + 400_000.0,
        );

        let config: MissionScenarioConfig = toml::from_str(&mission_toml).expect("mission");
        let result = crate::sim::mission::run_mission(&config).expect("run");
        let elements = OrbitalElements::from_state(
            &result.final_state,
            crate::physics::frame::Frame::earth_inertial()
                .mu()
                .expect("earth mu"),
            OrbitalElementClassification::default(),
        )
        .expect("elements");
        elements
            .right_ascension_of_ascending_node()
            .map_or(0.0, |angle| angle.degrees())
    }

    /// Propagates a low circular orbit under the given force model and returns
    /// the change in specific orbital energy (J/kg) over the episode.
    fn energy_delta(force_model_inline: &str) -> f64 {
        use crate::physics::diagnostics::OrbitalDiagnostics;

        let toml = format!(
            r#"
            name = "energy"
            altitude_m = 250_000.0
            dt_s = 10.0
            duration_s = 7_200.0
            integrator = "runge_kutta4"
            force_model = {force_model_inline}
            sample_every_steps = 1000
            "#,
        );
        let config = ScenarioConfig::from_toml(&toml).expect("config");
        let mut env = crate::sim::environment::LeoCircularEnv::from_config(&config).expect("env");
        let episode = env.run_episode(0).expect("episode");
        let initial = &episode.propagation.samples[0].state;
        let final_state = &episode.propagation.final_state;
        let mu = crate::physics::frame::Frame::earth_inertial()
            .mu()
            .expect("earth mu");
        let initial_energy = OrbitalDiagnostics::from_state(initial, mu)
            .expect("initial diagnostics")
            .specific_energy()
            .square_meters_per_square_second();
        let final_energy = OrbitalDiagnostics::from_state(final_state, mu)
            .expect("final diagnostics")
            .specific_energy()
            .square_meters_per_square_second();
        final_energy - initial_energy
    }
}
