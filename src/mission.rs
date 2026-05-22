use serde::{Deserialize, Serialize};

use crate::constants::R_EARTH;
use crate::dynamics::TwoBodyGravity;
use crate::errors::{MissionError, ScenarioError};
use crate::integrators::{
    EulerIntegrator, LeapfrogIntegrator, RungeKutta4Integrator, VelocityVerletIntegrator,
};
use crate::orbits::{CircularOrbit, CircularOrbitOrientation, OrbitDirection};
use crate::propagator::{
    ImpulsiveBurn, PropagationResult, Propagator, SampleConfig, TrajectorySample,
};
use crate::scenario::{IntegratorKind, parse_force_model, parse_integrator};
use crate::state::CartesianState;
use crate::termination::{StopReason, TerminationCondition};
use crate::types::{Angle, Distance, Duration, GravitationalParameter, TimeStep};

/// Serializable mission scenario loaded from TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
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
#[serde(tag = "kind", rename_all = "snake_case")]
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
pub struct PropagationConfig {
    /// Fixed integrator step size in seconds.
    pub dt_s: f64,
    /// Integrator name (`leapfrog`, `verlet`, `euler`, `rk4`).
    pub integrator: String,
    /// Force model name (`two_body` only in MVP).
    pub force_model: String,
    /// Sample trajectory every N integrator steps.
    pub sample_every_steps: usize,
}

/// One mission segment in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct MissionSegmentConfig {
    /// Segment stop condition.
    pub stop: StopConfig,
    /// Burns applied when this segment completes.
    #[serde(default)]
    pub burns: Vec<BurnConfig>,
}

/// Segment stop condition in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum StopConfig {
    /// Named orbital stop (`apoapsis`, `periapsis`).
    Named(String),
    /// Table-based stop specification.
    Table(StopTable),
}

/// Table form of a segment stop condition.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct StopTable {
    /// Elapsed segment duration in seconds.
    pub duration_s: Option<f64>,
    /// Integrator step count.
    pub steps: Option<usize>,
}

/// Burn specification in TOML.
#[derive(Debug, Clone, PartialEq, Serialize, Deserialize)]
pub struct BurnConfig {
    /// Inertial Cartesian delta-v in meters per second.
    pub delta_v_mps: Option<[f64; 3]>,
    /// Prograde delta-v magnitude in meters per second.
    pub prograde_delta_v_mps: Option<f64>,
    /// Match the local circular speed at the current radius.
    #[serde(default)]
    pub circularize: bool,
}

/// Loaded scenario kind for the runner.
#[derive(Debug, Clone, PartialEq)]
pub enum LoadedScenario {
    /// Single-rollout RL scenario.
    Rollout(crate::scenario::ScenarioConfig),
    /// Multi-segment mission scenario.
    Mission(MissionScenarioConfig),
}

impl LoadedScenario {
    /// Loads a scenario file, detecting rollout vs mission format.
    pub fn from_file(path: &std::path::Path) -> Result<Self, ScenarioError> {
        let contents = std::fs::read_to_string(path)
            .map_err(|error| ScenarioError::ReadFile(error.to_string()))?;
        Self::from_toml(&contents)
    }

    /// Parses a scenario string, detecting rollout vs mission format.
    pub fn from_toml(contents: &str) -> Result<Self, ScenarioError> {
        let value: toml::Value = toml::from_str(contents)
            .map_err(|error| ScenarioError::ParseFile(error.to_string()))?;

        let is_mission = value
            .get("kind")
            .and_then(toml::Value::as_str)
            .is_some_and(|kind| kind == "mission")
            || value.get("segments").is_some();

        if is_mission {
            let mission: MissionScenarioConfig = value
                .try_into()
                .map_err(|error: toml::de::Error| ScenarioError::ParseFile(error.to_string()))?;
            Ok(Self::Mission(mission))
        } else {
            Ok(Self::Rollout(crate::scenario::ScenarioConfig::from_toml(
                contents,
            )?))
        }
    }
}

impl MissionScenarioConfig {
    /// Builds runtime objects for mission execution.
    pub fn build(&self) -> Result<BuiltMissionScenario, MissionError> {
        if self.name.is_empty() {
            return Err(MissionError::InvalidField {
                field: "name",
                reason: "must not be empty".to_string(),
            });
        }
        if self.segments.is_empty() {
            return Err(MissionError::InvalidField {
                field: "segments",
                reason: "mission scenarios require at least one segment".to_string(),
            });
        }

        let dt = TimeStep::from_seconds(self.propagation.dt_s).ok_or_else(|| {
            MissionError::InvalidField {
                field: "propagation.dt_s",
                reason: "must be finite and positive".to_string(),
            }
        })?;
        let sample_every_steps = (self.propagation.sample_every_steps >= 1)
            .then_some(self.propagation.sample_every_steps)
            .ok_or_else(|| MissionError::InvalidField {
                field: "propagation.sample_every_steps",
                reason: "must be >= 1".to_string(),
            })?;

        let integrator =
            parse_integrator(&self.propagation.integrator).map_err(MissionError::Scenario)?;
        let gravity =
            parse_force_model(&self.propagation.force_model).map_err(MissionError::Scenario)?;
        let mu = gravity.mu;

        let initial_state = build_initial_state(&self.initial_orbit, mu)?;
        let propagator = Propagator::new(initial_state, dt)
            .map_err(MissionError::Propagation)?
            .with_sampling(
                SampleConfig::every_steps(sample_every_steps).ok_or_else(|| {
                    MissionError::InvalidField {
                        field: "propagation.sample_every_steps",
                        reason: "invalid sample config".to_string(),
                    }
                })?,
            );

        let initial_burns = self
            .initial_burns
            .iter()
            .map(build_burn_spec)
            .collect::<Result<Vec<_>, _>>()?;

        let segments = self
            .segments
            .iter()
            .map(|segment| build_segment(segment, mu))
            .collect::<Result<Vec<_>, _>>()?;

        Ok(BuiltMissionScenario {
            config: self.clone(),
            integrator,
            gravity,
            propagator,
            initial_state,
            initial_burns,
            segments,
            earth_radius_m: R_EARTH,
        })
    }
}

/// Runtime mission scenario ready for execution.
pub struct BuiltMissionScenario {
    /// Source configuration.
    pub config: MissionScenarioConfig,
    /// Selected integrator.
    pub integrator: IntegratorKind,
    /// Central gravity model.
    pub gravity: TwoBodyGravity,
    /// Propagator with initial state and sampling configured.
    pub propagator: Propagator,
    /// Initial Cartesian state for resets.
    pub initial_state: CartesianState,
    /// Burns applied before the first segment.
    pub initial_burns: Vec<BurnSpec>,
    /// Ordered mission segments.
    pub segments: Vec<MissionSegment>,
    /// Earth equatorial radius in meters.
    pub earth_radius_m: f64,
}

/// One runtime mission segment.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionSegment {
    /// Segment termination rule.
    pub termination: TerminationCondition,
    /// Burns applied when this segment completes.
    pub burns: Vec<BurnSpec>,
}

/// Runtime burn specification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BurnSpec {
    /// Inertial Cartesian delta-v in meters per second.
    Inertial([f64; 3]),
    /// Prograde delta-v magnitude in meters per second.
    Prograde(f64),
    /// Match the local circular speed at the current radius.
    Circularize,
}

/// Result of one mission segment.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionSegmentResult {
    /// Zero-based segment index.
    pub segment_index: usize,
    /// Propagation output for the segment.
    pub propagation: PropagationResult,
    /// Burns applied after the segment completed.
    pub burns_applied: usize,
    /// Stop reason for the segment.
    pub stop_reason: StopReason,
}

/// Result of executing a full mission plan.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionResult {
    /// Per-segment propagation results.
    pub segments: Vec<MissionSegmentResult>,
    /// Combined trajectory samples across all segments.
    pub samples: Vec<TrajectorySample>,
    /// Final Cartesian state after all segments and burns.
    pub final_state: CartesianState,
    /// Final simulation epoch.
    pub final_epoch: crate::types::Epoch,
}

/// Executes multi-segment mission plans using the shared propagator.
pub struct MissionRunner;

impl MissionRunner {
    /// Runs a built mission scenario once from its initial state.
    pub fn run(scenario: &mut BuiltMissionScenario) -> Result<MissionResult, MissionError> {
        scenario
            .propagator
            .reset(scenario.initial_state)
            .map_err(MissionError::Propagation)?;

        let mut all_samples = Vec::new();
        let mut segment_results = Vec::new();

        for burn in &scenario.initial_burns {
            apply_burn(&mut scenario.propagator, *burn, scenario.gravity.mu)?;
        }

        for (segment_index, segment) in scenario.segments.clone().into_iter().enumerate() {
            let propagation = propagate_segment(scenario, segment.termination, segment_index)?;
            all_samples.extend(propagation.samples.iter().cloned());

            let mut burns_applied = 0_usize;
            for burn in &segment.burns {
                apply_burn(&mut scenario.propagator, *burn, scenario.gravity.mu)?;
                burns_applied += 1;
            }

            segment_results.push(MissionSegmentResult {
                segment_index,
                stop_reason: propagation.stop_reason,
                propagation,
                burns_applied,
            });
        }

        let final_state = scenario
            .propagator
            .current_state()
            .map_err(MissionError::Propagation)?;
        let final_epoch = segment_results
            .last()
            .map(|segment| segment.propagation.final_epoch)
            .unwrap_or_else(|| crate::types::Epoch::from_seconds(0.0).expect("epoch"));

        Ok(MissionResult {
            segments: segment_results,
            samples: all_samples,
            final_state,
            final_epoch,
        })
    }
}

fn propagate_segment(
    scenario: &mut BuiltMissionScenario,
    termination: TerminationCondition,
    segment_index: usize,
) -> Result<PropagationResult, MissionError> {
    let mu = scenario.gravity.mu;
    match scenario.integrator {
        IntegratorKind::Leapfrog => scenario
            .propagator
            .propagate_until_with_segment(
                &LeapfrogIntegrator,
                &scenario.gravity,
                termination,
                mu,
                Some(segment_index),
            )
            .map_err(MissionError::Propagation),
        IntegratorKind::VelocityVerlet => scenario
            .propagator
            .propagate_until_with_segment(
                &VelocityVerletIntegrator,
                &scenario.gravity,
                termination,
                mu,
                Some(segment_index),
            )
            .map_err(MissionError::Propagation),
        IntegratorKind::Euler => scenario
            .propagator
            .propagate_until_with_segment(
                &EulerIntegrator,
                &scenario.gravity,
                termination,
                mu,
                Some(segment_index),
            )
            .map_err(MissionError::Propagation),
        IntegratorKind::RungeKutta4 => scenario
            .propagator
            .propagate_until_with_segment(
                &RungeKutta4Integrator,
                &scenario.gravity,
                termination,
                mu,
                Some(segment_index),
            )
            .map_err(MissionError::Propagation),
    }
}

fn apply_burn(
    propagator: &mut Propagator,
    burn: BurnSpec,
    mu: GravitationalParameter,
) -> Result<(), MissionError> {
    let impulsive = match burn {
        BurnSpec::Inertial(delta_v_mps) => ImpulsiveBurn::inertial(delta_v_mps),
        BurnSpec::Prograde(delta_v_mps) => {
            let state = propagator
                .current_state()
                .map_err(MissionError::Propagation)?;
            ImpulsiveBurn::prograde(delta_v_mps, &state).map_err(MissionError::Propagation)?
        }
        BurnSpec::Circularize => {
            let state = propagator
                .current_state()
                .map_err(MissionError::Propagation)?;
            let position = state.position.vector();
            let velocity = state.velocity.vector();
            let radius_m = position.norm();
            let mu = mu.meters_cubed_per_second_squared();
            if radius_m <= 0.0 {
                return Err(MissionError::InvalidField {
                    field: "burns.circularize",
                    reason: "state is invalid for circularization".to_string(),
                });
            }

            let radial_hat = position / radius_m;
            let radial_speed_mps = velocity.dot(&radial_hat);
            let tangential_velocity = velocity - radial_hat * radial_speed_mps;
            let tangential_speed_mps = tangential_velocity.norm();
            if tangential_speed_mps <= 0.0 {
                return Err(MissionError::InvalidField {
                    field: "burns.circularize",
                    reason: "tangential velocity is undefined".to_string(),
                });
            }

            let target_tangential_speed_mps = (mu / radius_m).sqrt();
            let tangential_hat = tangential_velocity / tangential_speed_mps;
            let delta_v = tangential_hat * (target_tangential_speed_mps - tangential_speed_mps)
                - radial_hat * radial_speed_mps;
            ImpulsiveBurn::inertial([delta_v.x, delta_v.y, delta_v.z])
        }
    };
    propagator
        .apply_impulsive_burn(impulsive)
        .map_err(MissionError::Propagation)
}

fn build_initial_state(
    initial_orbit: &InitialOrbitConfig,
    mu: GravitationalParameter,
) -> Result<CartesianState, MissionError> {
    match initial_orbit {
        InitialOrbitConfig::Circular { altitude_m } => {
            let radius = Distance::from_meters(R_EARTH + altitude_m).ok_or_else(|| {
                MissionError::InvalidField {
                    field: "initial_orbit.altitude_m",
                    reason: "orbital radius is invalid".to_string(),
                }
            })?;
            let orbit = CircularOrbit {
                radius,
                mu,
                orientation: CircularOrbitOrientation::equatorial(),
                phase: Angle::from_radians(0.0).ok_or_else(|| MissionError::InvalidField {
                    field: "initial_orbit",
                    reason: "initial phase is invalid".to_string(),
                })?,
                direction: OrbitDirection::Prograde,
            };
            orbit
                .to_cartesian_state()
                .ok_or_else(|| MissionError::InvalidField {
                    field: "initial_orbit",
                    reason: "failed to build circular initial state".to_string(),
                })
        }
        InitialOrbitConfig::Classical {
            semi_major_axis_m,
            eccentricity,
            inclination_deg,
            raan_deg,
            arg_periapsis_deg,
            true_anomaly_deg,
        } => {
            use crate::diagnostics::Eccentricity;
            use crate::elements::ClassicalOrbitalElements;

            let elements = ClassicalOrbitalElements {
                semi_major_axis: Distance::from_meters(*semi_major_axis_m).ok_or_else(|| {
                    MissionError::InvalidField {
                        field: "initial_orbit.semi_major_axis_m",
                        reason: "semi-major axis is invalid".to_string(),
                    }
                })?,
                eccentricity: Eccentricity::from_scalar(*eccentricity).ok_or_else(|| {
                    MissionError::InvalidField {
                        field: "initial_orbit.eccentricity",
                        reason: "eccentricity is invalid".to_string(),
                    }
                })?,
                inclination: degrees_to_angle(*inclination_deg, "initial_orbit.inclination_deg")?,
                right_ascension_of_ascending_node: degrees_to_angle(
                    *raan_deg,
                    "initial_orbit.raan_deg",
                )?,
                argument_of_periapsis: degrees_to_angle(
                    *arg_periapsis_deg,
                    "initial_orbit.arg_periapsis_deg",
                )?,
                true_anomaly: degrees_to_angle(
                    *true_anomaly_deg,
                    "initial_orbit.true_anomaly_deg",
                )?,
            };
            elements
                .to_cartesian_state(mu)
                .ok_or_else(|| MissionError::InvalidField {
                    field: "initial_orbit",
                    reason: "failed to build classical initial state".to_string(),
                })
        }
    }
}

fn degrees_to_angle(degrees: f64, field: &'static str) -> Result<Angle, MissionError> {
    Angle::from_radians(degrees.to_radians()).ok_or_else(|| MissionError::InvalidField {
        field,
        reason: "angle is invalid".to_string(),
    })
}

fn build_burn_spec(burn: &BurnConfig) -> Result<BurnSpec, MissionError> {
    let mode_count = burn.delta_v_mps.is_some() as u8
        + burn.prograde_delta_v_mps.is_some() as u8
        + u8::from(burn.circularize);
    if mode_count != 1 {
        return Err(MissionError::InvalidField {
            field: "burns",
            reason: "specify exactly one of delta_v_mps, prograde_delta_v_mps, or circularize"
                .to_string(),
        });
    }

    if burn.circularize {
        return Ok(BurnSpec::Circularize);
    }

    match (burn.delta_v_mps, burn.prograde_delta_v_mps) {
        (Some(delta_v_mps), None) => Ok(BurnSpec::Inertial(delta_v_mps)),
        (None, Some(prograde_delta_v_mps)) => {
            if prograde_delta_v_mps.is_finite() {
                Ok(BurnSpec::Prograde(prograde_delta_v_mps))
            } else {
                Err(MissionError::InvalidField {
                    field: "burns.prograde_delta_v_mps",
                    reason: "must be finite".to_string(),
                })
            }
        }
        _ => unreachable!("burn mode count validated above"),
    }
}

fn build_segment(
    segment: &MissionSegmentConfig,
    mu: GravitationalParameter,
) -> Result<MissionSegment, MissionError> {
    let termination = parse_stop_config(&segment.stop, mu)?;
    let burns = segment
        .burns
        .iter()
        .map(build_burn_spec)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(MissionSegment { termination, burns })
}

fn parse_stop_config(
    stop: &StopConfig,
    mu: GravitationalParameter,
) -> Result<TerminationCondition, MissionError> {
    match stop {
        StopConfig::Named(name) => match name.as_str() {
            "apoapsis" => Ok(TerminationCondition::apoapsis(mu)),
            "periapsis" => Ok(TerminationCondition::periapsis(mu)),
            other => Err(MissionError::InvalidField {
                field: "segments.stop",
                reason: format!("unknown stop `{other}`"),
            }),
        },
        StopConfig::Table(table) => match (table.duration_s, table.steps) {
            (Some(duration_s), None) => {
                let duration = Duration::from_seconds(duration_s).ok_or_else(|| {
                    MissionError::InvalidField {
                        field: "segments.stop.duration_s",
                        reason: "must be finite and non-negative".to_string(),
                    }
                })?;
                Ok(TerminationCondition::Duration(duration))
            }
            (None, Some(steps)) => Ok(TerminationCondition::StepCount(steps)),
            (Some(_), Some(_)) => Err(MissionError::InvalidField {
                field: "segments.stop",
                reason: "specify either duration_s or steps, not both".to_string(),
            }),
            (None, None) => Err(MissionError::InvalidField {
                field: "segments.stop",
                reason: "must specify duration_s or steps".to_string(),
            }),
        },
    }
}

/// Analytical two-body Hohmann transfer delta-vs between circular orbits.
#[must_use]
pub fn hohmann_delta_v_mps(
    mu: f64,
    departure_radius_m: f64,
    arrival_radius_m: f64,
) -> Option<(f64, f64)> {
    if !mu.is_finite() || mu <= 0.0 {
        return None;
    }
    if !departure_radius_m.is_finite()
        || departure_radius_m <= 0.0
        || !arrival_radius_m.is_finite()
        || arrival_radius_m <= 0.0
    {
        return None;
    }

    let semi_major_axis = 0.5 * (departure_radius_m + arrival_radius_m);
    let departure_circular_speed = (mu / departure_radius_m).sqrt();
    let departure_transfer_speed = (mu * (2.0 / departure_radius_m - 1.0 / semi_major_axis)).sqrt();
    let arrival_transfer_speed = (mu * (2.0 / arrival_radius_m - 1.0 / semi_major_axis)).sqrt();
    let arrival_circular_speed = (mu / arrival_radius_m).sqrt();

    Some((
        departure_transfer_speed - departure_circular_speed,
        arrival_circular_speed - arrival_transfer_speed,
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{MU_EARTH, R_EARTH};
    use crate::elements::OrbitalElements;
    use crate::types::GravitationalParameter;

    fn elements_semi_major_axis_m(elements: &OrbitalElements) -> f64 {
        match elements {
            OrbitalElements::Classical(elements) => elements.semi_major_axis.meters(),
            OrbitalElements::CircularInclined(elements) => elements.semi_major_axis.meters(),
            OrbitalElements::CircularEquatorial(elements) => elements.semi_major_axis.meters(),
            OrbitalElements::EquatorialElliptic(elements) => elements.semi_major_axis.meters(),
        }
    }

    #[test]
    fn parses_mission_scenario_toml() {
        let config = MissionScenarioConfig {
            name: "test".to_string(),
            kind: "mission".to_string(),
            initial_orbit: InitialOrbitConfig::Circular {
                altitude_m: 400_000.0,
            },
            propagation: PropagationConfig {
                dt_s: 10.0,
                integrator: "leapfrog".to_string(),
                force_model: "two_body".to_string(),
                sample_every_steps: 5,
            },
            initial_burns: vec![BurnConfig {
                delta_v_mps: None,
                prograde_delta_v_mps: Some(100.0),
                circularize: false,
            }],
            segments: vec![MissionSegmentConfig {
                stop: StopConfig::Named("apoapsis".to_string()),
                burns: vec![],
            }],
        };
        let built = config.build().expect("built mission");
        assert_eq!(built.segments.len(), 1);
    }

    #[test]
    fn hohmann_transfer_reaches_target_altitude() {
        let departure_altitude_m = 400_000.0;
        let arrival_altitude_m = 35_786_000.0;
        let departure_radius_m = R_EARTH + departure_altitude_m;
        let arrival_radius_m = R_EARTH + arrival_altitude_m;
        let (dv1, _dv2) = hohmann_delta_v_mps(MU_EARTH, departure_radius_m, arrival_radius_m)
            .expect("hohmann delta-v");

        let config = MissionScenarioConfig {
            name: "hohmann".to_string(),
            kind: "mission".to_string(),
            initial_orbit: InitialOrbitConfig::Circular {
                altitude_m: departure_altitude_m,
            },
            propagation: PropagationConfig {
                dt_s: 30.0,
                integrator: "leapfrog".to_string(),
                force_model: "two_body".to_string(),
                sample_every_steps: 20,
            },
            initial_burns: vec![BurnConfig {
                delta_v_mps: None,
                prograde_delta_v_mps: Some(dv1),
                circularize: false,
            }],
            segments: vec![
                MissionSegmentConfig {
                    stop: StopConfig::Named("apoapsis".to_string()),
                    burns: vec![BurnConfig {
                        delta_v_mps: None,
                        prograde_delta_v_mps: None,
                        circularize: true,
                    }],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Table(StopTable {
                        duration_s: Some(86_400.0),
                        steps: None,
                    }),
                    burns: vec![],
                },
            ],
        };

        let mut built = config.build().expect("built mission");
        let result = MissionRunner::run(&mut built).expect("mission run");
        let final_radius = result.final_state.position.vector().norm();

        let seg0_radius = result.segments[0]
            .propagation
            .final_state
            .position
            .vector()
            .norm();
        assert!(
            (seg0_radius - arrival_radius_m).abs() / arrival_radius_m < 0.03,
            "segment 0 should stop near transfer apoapsis, got {seg0_radius}"
        );
        assert!(
            (final_radius - arrival_radius_m).abs() / arrival_radius_m < 0.02,
            "expected GEO radius, got {final_radius}"
        );
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let elements =
            OrbitalElements::from_cartesian_state(&result.final_state, mu).expect("elements");
        let semi_major_axis = elements_semi_major_axis_m(&elements);
        assert!(
            (semi_major_axis - arrival_radius_m).abs() / arrival_radius_m < 0.02,
            "expected GEO semi-major axis, got {semi_major_axis}"
        );
        assert_eq!(result.segments.len(), 2);
        assert_eq!(result.segments[0].stop_reason, StopReason::ApoapsisMet);
    }

    #[test]
    fn loads_hohmann_transfer_scenario_file() {
        let loaded =
            LoadedScenario::from_file(std::path::Path::new("scenarios/hohmann_transfer.toml"))
                .expect("loaded scenario");
        let LoadedScenario::Mission(config) = loaded else {
            panic!("expected mission scenario");
        };
        let built = config.build().expect("built mission");
        assert_eq!(built.segments.len(), 2);
    }
}
