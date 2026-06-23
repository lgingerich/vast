use serde::{Deserialize, Serialize};

use crate::errors::MissionError;
use crate::physics::frame::{Axes, BodyId, Frame, Origin};

/// Loaded scenario kind for the runner.
#[derive(Debug, Clone, PartialEq)]
pub enum LoadedScenario {
    /// Single-rollout RL scenario.
    Rollout(ScenarioConfig),
    /// Multi-segment mission scenario.
    Mission(MissionScenarioConfig),
}

impl LoadedScenario {
    /// Loads a scenario file, detecting rollout vs mission format.
    pub fn from_file(path: &std::path::Path) -> Result<Self, MissionError> {
        let contents = std::fs::read_to_string(path)?;
        Self::from_toml(&contents)
    }

    /// Parses a scenario string, detecting rollout vs mission format.
    pub fn from_toml(contents: &str) -> Result<Self, MissionError> {
        let value: toml::Value = toml::from_str(contents)?;
        let is_mission = value
            .get("kind")
            .and_then(toml::Value::as_str)
            .is_some_and(|kind| kind == "mission")
            || value.get("segments").is_some();

        if is_mission {
            Ok(Self::Mission(value.try_into()?))
        } else {
            Ok(Self::Rollout(value.try_into()?))
        }
    }
}

/// Serializable rollout scenario definition loaded from TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioConfig {
    /// Human-readable scenario name.
    pub name: String,
    /// Central body around which the scenario orbits.
    #[serde(default = "default_body_config")]
    pub body: BodyConfig,
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
    /// Integrator steps advanced by one environment step.
    #[serde(default = "default_steps_per_env_step")]
    pub steps_per_env_step: usize,
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
}

fn default_steps_per_env_step() -> usize {
    1
}

fn default_body_config() -> BodyConfig {
    BodyConfig::Earth
}

/// Serializable mission scenario loaded from TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MissionScenarioConfig {
    /// Human-readable scenario name.
    pub name: String,
    /// Scenario kind marker (`mission`).
    #[serde(default = "default_mission_kind")]
    pub kind: String,
    /// Initial orbit definition.
    pub initial_orbit: InitialOrbitConfig,
    /// Propagation settings shared by all segments.
    pub propagation: PropagationConfig,
    /// Burns applied before the first segment.
    #[serde(default)]
    pub initial_burns: Vec<BurnConfig>,
    /// Ordered mission segments.
    pub segments: Vec<MissionSegmentConfig>,
}

fn default_mission_kind() -> String {
    "mission".to_string()
}

/// Initial orbit specification for mission scenarios.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum InitialOrbitConfig {
    /// Circular equatorial orbit at the given altitude.
    Circular {
        /// Altitude above the equatorial radius in meters.
        altitude_m: f64,
    },
    /// Classical Keplerian elements at epoch zero.
    Classical {
        /// Semi-major axis in meters.
        semi_major_axis_m: f64,
        /// Orbital eccentricity.
        eccentricity: f64,
        /// Inclination in degrees.
        inclination_deg: f64,
        /// Right ascension of the ascending node in degrees.
        raan_deg: f64,
        /// Argument of periapsis in degrees.
        arg_periapsis_deg: f64,
        /// True anomaly in degrees.
        true_anomaly_deg: f64,
    },
}

/// Shared propagation settings for mission scenarios.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PropagationConfig {
    /// Fixed integrator step size in seconds.
    pub dt_s: f64,
    /// Integrator.
    pub integrator: IntegratorKind,
    /// Central body shared by all mission segments.
    #[serde(default = "default_body_config")]
    pub body: BodyConfig,
    /// Force model.
    pub force_model: ForceModelKind,
    /// Sample trajectory every N integrator steps.
    pub sample_every_steps: usize,
}

/// One mission segment in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MissionSegmentConfig {
    /// Segment stop condition.
    pub stop: StopConfig,
    /// Burns applied when this segment completes.
    #[serde(default)]
    pub burns: Vec<BurnConfig>,
}

/// Segment stop condition in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum StopConfig {
    /// Stop at the first detected apoapsis crossing.
    Apoapsis,
    /// Stop at the first detected periapsis crossing.
    Periapsis,
    /// Stop after an elapsed segment duration.
    Duration {
        /// Elapsed segment duration in seconds.
        duration_s: f64,
    },
    /// Stop after a fixed number of integrator steps.
    StepCount {
        /// Integrator step count.
        steps: usize,
    },
}

/// Burn specification in TOML.
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum BurnConfig {
    /// Inertial Cartesian delta-v in meters per second.
    Inertial {
        /// Inertial Cartesian delta-v in meters per second.
        delta_v_mps: [f64; 3],
    },
    /// Prograde delta-v magnitude in meters per second.
    Prograde {
        /// Prograde delta-v magnitude in meters per second.
        delta_v_mps: f64,
    },
    /// Match the local circular speed at the current radius.
    Circularize,
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

/// Supported central bodies for scenario configuration.
///
/// New named bodies can be added here as their scenarios become active.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BodyConfig {
    /// Earth using WGS 84 / EGM2008 constants and a loose LEO atmosphere.
    Earth,
}

impl BodyConfig {
    /// Resolves this body into the inertial frame used for orbit construction.
    pub const fn inertial_frame(&self) -> Frame {
        match self {
            Self::Earth => Frame::new(Origin::CelestialBody(BodyId::Earth), Axes::J2000Eq),
        }
    }
}

/// Supported force models for scenario configuration.
///
/// Each variant resolves to a concrete `Dynamics` implementation in the
/// runtime plan. Physical constants such as gravitational parameter,
/// equatorial radius, J2, and atmosphere come from the scenario's chosen
/// central body; only spacecraft or model-specific parameters remain here.
/// Always written as an inline table with a `kind` tag in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind")]
pub enum ForceModelKind {
    /// Two-body point-mass gravity.
    TwoBody,
    /// Central-body gravity with the body's J2 zonal harmonic.
    J2,
    /// J2 gravity plus exponential atmospheric drag.
    J2Drag {
        /// Drag area per unit mass (C_d * A / m) in m^2/kg.
        area_per_mass_m2_kg: f64,
    },
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn rollout_config_rejects_unknown_fields() {
        let error = ScenarioConfig::from_toml(
            r#"
            name = "bad"
            body = "earth"
            altitude_m = 400_000.0
            dt_s = 10.0
            duration_s = 100.0
            integrator = "leapfrog"
            force_model = { kind = "two_body" }
            sample_every_steps = 5
            typo_field = true
            "#,
        )
        .expect_err("unknown field should fail");

        assert!(matches!(error, MissionError::ParseFile(_)));
    }

    #[test]
    fn mission_config_parses_tagged_stops_and_burns() {
        let loaded = LoadedScenario::from_toml(
            r#"
            name = "mission"
            kind = "mission"

            [initial_orbit]
            kind = "circular"
            altitude_m = 400_000.0

            [propagation]
            dt_s = 10.0
            body = "earth"
            integrator = "leapfrog"
            force_model = { kind = "two_body" }
            sample_every_steps = 5

            [[initial_burns]]
            kind = "prograde"
            delta_v_mps = 1.0

            [[segments]]
            stop = { kind = "duration", duration_s = 100.0 }

            [[segments.burns]]
            kind = "circularize"
            "#,
        )
        .expect("mission");

        let LoadedScenario::Mission(config) = loaded else {
            panic!("expected mission config");
        };
        assert_eq!(config.initial_burns.len(), 1);
        assert_eq!(config.segments[0].burns, vec![BurnConfig::Circularize]);
    }
}
