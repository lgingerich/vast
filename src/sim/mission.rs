use serde::{Deserialize, Serialize};

use super::propagator::{ImpulsiveBurn, PropagationResult, Propagator, TrajectorySample};
use super::scenario::{self, ForceModelKind, IntegratorKind, PropagationSetup};
use super::termination::{StopReason, TerminationCondition};
use crate::errors::MissionError;
use crate::physics::constants::R_EARTH;
use crate::physics::state::CartesianState;
use crate::physics::types::{Angle, Distance, Duration, Eccentricity, GravitationalParameter};

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
    /// Integrator.
    pub integrator: IntegratorKind,
    /// Force model.
    pub force_model: ForceModelKind,
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
    Rollout(crate::sim::scenario::ScenarioConfig),
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
            let mission: MissionScenarioConfig = value.try_into()?;
            Ok(Self::Mission(mission))
        } else {
            let rollout: crate::sim::scenario::ScenarioConfig = value.try_into()?;
            Ok(Self::Rollout(rollout))
        }
    }
}

impl MissionScenarioConfig {
    /// Builds runtime objects for mission execution.
    pub(crate) fn build(&self) -> Result<BuiltMissionScenario, MissionError> {
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

        let dt = crate::physics::types::TimeStep::from_seconds(self.propagation.dt_s)?;

        let integrator = self.propagation.integrator;
        let gravity = scenario::create_gravity(self.propagation.force_model)?;
        let mu = gravity.mu();

        let initial_state = build_initial_state(&self.initial_orbit, mu)?;
        let propagator =
            scenario::create_propagator(initial_state, dt, self.propagation.sample_every_steps)?;

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
            propagation: PropagationSetup {
                integrator,
                gravity,
                propagator,
                initial_state,
            },
            initial_burns,
            segments,
        })
    }
}

/// Runtime mission scenario ready for execution.
pub struct BuiltMissionScenario {
    /// Shared propagation machine.
    pub propagation: PropagationSetup,
    /// Burns applied before the first segment.
    pub initial_burns: Vec<BurnSpec>,
    /// Ordered mission segments.
    pub segments: Vec<MissionSegment>,
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
    pub final_epoch: crate::physics::types::Epoch,
}

/// Runs a built mission scenario once from its initial state.
pub fn run_mission(config: &MissionScenarioConfig) -> Result<MissionResult, MissionError> {
    let mut scenario = config.build()?;
    scenario
        .propagation
        .propagator
        .reset(scenario.propagation.initial_state)?;

    let mut all_samples = Vec::new();
    let mut segment_results = Vec::new();

    for burn in &scenario.initial_burns {
        apply_burn(
            &mut scenario.propagation.propagator,
            *burn,
            scenario.propagation.gravity.mu(),
        )?;
    }

    let segment_count = scenario.segments.len();
    for segment_index in 0..segment_count {
        let termination = scenario.segments[segment_index].termination.clone();
        let propagation = propagate_segment(&mut scenario, &termination, segment_index)?;
        all_samples.extend(propagation.samples.iter().cloned());

        let burns: Vec<BurnSpec> = scenario.segments[segment_index].burns.clone();
        let mut burns_applied = 0_usize;
        for burn in &burns {
            apply_burn(
                &mut scenario.propagation.propagator,
                *burn,
                scenario.propagation.gravity.mu(),
            )?;
            burns_applied += 1;
        }

        segment_results.push(MissionSegmentResult {
            segment_index,
            stop_reason: propagation.stop_reason,
            propagation,
            burns_applied,
        });
    }

    let final_state = scenario.propagation.propagator.current_state()?;
    // Segments are validated as non-empty at build time.
    let final_epoch = segment_results
        .last()
        .map(|segment| segment.propagation.final_epoch)
        .expect("at least one segment is guaranteed by build()");

    Ok(MissionResult {
        segments: segment_results,
        samples: all_samples,
        final_state,
        final_epoch,
    })
}

fn propagate_segment(
    scenario: &mut BuiltMissionScenario,
    termination: &TerminationCondition,
    segment_index: usize,
) -> Result<PropagationResult, MissionError> {
    Ok(scenario.propagation.integrator.dispatch(
        &mut scenario.propagation.propagator,
        &scenario.propagation.gravity,
        termination.clone(),
        Some(segment_index),
    )?)
}

fn apply_burn(
    propagator: &mut Propagator,
    burn: BurnSpec,
    mu: GravitationalParameter,
) -> Result<(), MissionError> {
    let impulsive = match burn {
        BurnSpec::Inertial(delta_v_mps) => ImpulsiveBurn::inertial(delta_v_mps)?,
        BurnSpec::Prograde(delta_v_mps) => {
            let state = propagator.current_state()?;
            ImpulsiveBurn::prograde(delta_v_mps, &state)?
        }
        BurnSpec::Circularize => {
            let state = propagator.current_state()?;
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
            ImpulsiveBurn::inertial([delta_v.x, delta_v.y, delta_v.z])?
        }
    };
    propagator.apply_impulsive_burn(impulsive)?;
    Ok(())
}

fn build_initial_state(
    initial_orbit: &InitialOrbitConfig,
    mu: GravitationalParameter,
) -> Result<CartesianState, MissionError> {
    match initial_orbit {
        InitialOrbitConfig::Circular { altitude_m } => {
            let radius = Distance::from_meters(R_EARTH + altitude_m)?;
            let orbit = crate::physics::orbits::EllipticOrbit::new(
                radius,
                Eccentricity::from_scalar(0.0)?,
                mu,
                Angle::ZERO,
                Angle::ZERO,
                Angle::ZERO,
                Angle::from_radians(0.0)?,
                crate::physics::orbits::OrbitDirection::Prograde,
            )?;
            Ok(orbit.to_cartesian_state()?)
        }
        InitialOrbitConfig::Classical {
            semi_major_axis_m,
            eccentricity,
            inclination_deg,
            raan_deg,
            arg_periapsis_deg,
            true_anomaly_deg,
        } => {
            use crate::physics::elements::ClassicalOrbitalElements;
            use crate::physics::types::Eccentricity;

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
        _ => Err(MissionError::InvalidField {
            field: "burns",
            reason: "specify exactly one of delta_v_mps, prograde_delta_v_mps, or circularize"
                .to_string(),
        }),
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
                let duration = Duration::from_seconds(duration_s)?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};
    use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};
    use crate::physics::types::GravitationalParameter;

    fn elements_semi_major_axis_m(elements: &OrbitalElements) -> f64 {
        match elements {
            OrbitalElements::Classical(elements) => elements.semi_major_axis().meters(),
            OrbitalElements::CircularInclined(elements) => elements.semi_major_axis().meters(),
            OrbitalElements::CircularEquatorial(elements) => elements.semi_major_axis().meters(),
            OrbitalElements::EquatorialElliptic(elements) => elements.semi_major_axis().meters(),
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
                integrator: IntegratorKind::Leapfrog,
                force_model: ForceModelKind::TwoBody,
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
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let departure = Distance::from_meters(departure_radius_m).expect("distance");
        let arrival = Distance::from_meters(arrival_radius_m).expect("distance");
        let (dv1, _dv2) = crate::physics::maneuvers::hohmann_delta_v(mu, departure, arrival)
            .expect("hohmann delta-v");

        let config = MissionScenarioConfig {
            name: "hohmann".to_string(),
            kind: "mission".to_string(),
            initial_orbit: InitialOrbitConfig::Circular {
                altitude_m: departure_altitude_m,
            },
            propagation: PropagationConfig {
                dt_s: 30.0,
                integrator: IntegratorKind::Leapfrog,
                force_model: ForceModelKind::TwoBody,
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

        let result = run_mission(&config).expect("mission run");
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
        let elements = OrbitalElements::from_state(
            &result.final_state,
            mu,
            OrbitalElementClassification::default(),
        )
        .expect("elements");
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

    #[test]
    fn bielliptic_delta_v_rejects_invalid_intermediate_radius() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let departure = Distance::from_meters(R_EARTH + 400_000.0).expect("distance");
        let arrival = Distance::from_meters(R_EARTH + 35_786_000.0).expect("distance");
        let intermediate = Distance::from_meters(R_EARTH + 400_000.0).expect("distance");
        assert!(
            crate::physics::maneuvers::bielliptic_delta_v(mu, departure, arrival, intermediate)
                .is_none()
        );
    }

    #[test]
    fn bielliptic_transfer_reaches_target_altitude() {
        let departure_altitude_m = 400_000.0;
        let arrival_altitude_m = 35_786_000.0;
        let departure_radius_m = R_EARTH + departure_altitude_m;
        let arrival_radius_m = R_EARTH + arrival_altitude_m;
        let intermediate_radius_m = 10.0 * arrival_radius_m;
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        let departure = Distance::from_meters(departure_radius_m).expect("distance");
        let arrival = Distance::from_meters(arrival_radius_m).expect("distance");
        let intermediate = Distance::from_meters(intermediate_radius_m).expect("distance");
        let (dv1, dv2, _dv3) =
            crate::physics::maneuvers::bielliptic_delta_v(mu, departure, arrival, intermediate)
                .expect("bi-elliptic delta-v");

        let dt_s = 30.0_f64;
        let first_leg_steps = (493_332.6 / dt_s).round() as usize;
        let second_leg_steps = (555_699.3 / dt_s).round() as usize;
        let phasing_half_orbit_steps = (43_082.0 / dt_s).round() as usize;
        let on_station_steps = (86_400.0 / dt_s).round() as usize;

        let config = MissionScenarioConfig {
            name: "bielliptic".to_string(),
            kind: "mission".to_string(),
            initial_orbit: InitialOrbitConfig::Classical {
                semi_major_axis_m: departure_radius_m,
                eccentricity: 0.0,
                inclination_deg: 28.5,
                raan_deg: 45.0,
                arg_periapsis_deg: 0.0,
                true_anomaly_deg: 0.0,
            },
            propagation: PropagationConfig {
                dt_s,
                integrator: IntegratorKind::RungeKutta4,
                force_model: ForceModelKind::TwoBody,
                sample_every_steps: 60,
            },
            initial_burns: vec![BurnConfig {
                delta_v_mps: None,
                prograde_delta_v_mps: Some(dv1),
                circularize: false,
            }],
            segments: vec![
                MissionSegmentConfig {
                    stop: StopConfig::Table(StopTable {
                        duration_s: None,
                        steps: Some(first_leg_steps),
                    }),
                    burns: vec![BurnConfig {
                        delta_v_mps: None,
                        prograde_delta_v_mps: Some(dv2),
                        circularize: false,
                    }],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Table(StopTable {
                        duration_s: None,
                        steps: Some(second_leg_steps),
                    }),
                    burns: vec![BurnConfig {
                        delta_v_mps: None,
                        prograde_delta_v_mps: None,
                        circularize: true,
                    }],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Table(StopTable {
                        duration_s: None,
                        steps: Some(phasing_half_orbit_steps),
                    }),
                    burns: vec![],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Table(StopTable {
                        duration_s: None,
                        steps: Some(on_station_steps),
                    }),
                    burns: vec![],
                },
            ],
        };

        let result = run_mission(&config).expect("mission run");
        let final_radius = result.final_state.position.vector().norm();

        let seg0_radius = result.segments[0]
            .propagation
            .final_state
            .position
            .vector()
            .norm();
        assert!(
            (seg0_radius - intermediate_radius_m).abs() / intermediate_radius_m < 0.03,
            "segment 0 should stop near transfer apoapsis, got {seg0_radius}"
        );
        let seg1_radius = result.segments[1]
            .propagation
            .final_state
            .position
            .vector()
            .norm();
        assert!(
            (seg1_radius - arrival_radius_m).abs() / arrival_radius_m < 0.03,
            "segment 1 should stop near GEO periapsis, got {seg1_radius}"
        );
        assert!(
            (final_radius - arrival_radius_m).abs() / arrival_radius_m < 0.02,
            "expected GEO radius, got {final_radius}"
        );
        assert_eq!(result.segments.len(), 4);
        assert_eq!(result.segments[0].stop_reason, StopReason::StepCountMet);
        assert_eq!(result.segments[1].stop_reason, StopReason::StepCountMet);
    }

    #[test]
    fn loads_bielliptic_transfer_scenario_file() {
        let loaded =
            LoadedScenario::from_file(std::path::Path::new("scenarios/bielliptic_leo_to_geo.toml"))
                .expect("loaded scenario");
        let LoadedScenario::Mission(config) = loaded else {
            panic!("expected mission scenario");
        };
        let built = config.build().expect("built mission");
        assert_eq!(built.segments.len(), 4);
        assert_eq!(built.initial_burns.len(), 1);
    }
}
