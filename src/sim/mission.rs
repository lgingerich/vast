use super::config::MissionScenarioConfig;
use super::engine::{PropagationResult, Propagator};
use crate::errors::MissionError;
use crate::physics::state::CartesianState;

/// Result of one mission segment.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionSegmentResult {
    /// Zero-based segment index.
    pub segment_index: usize,
    /// Propagation output for the segment.
    pub propagation: PropagationResult,
    /// Burns applied after the segment completed.
    pub burns_applied: usize,
}

/// Result of executing a full mission plan.
#[derive(Debug, Clone, PartialEq)]
pub struct MissionResult {
    /// Per-segment propagation results.
    pub segments: Vec<MissionSegmentResult>,
    /// Final Cartesian state after all segments and burns.
    pub final_state: CartesianState,
    /// Final simulation epoch.
    pub final_epoch: crate::physics::types::Epoch,
}

/// Runs a mission scenario once from its initial state.
pub fn run_mission(config: &MissionScenarioConfig) -> Result<MissionResult, MissionError> {
    let plan = config.build()?;
    let mut propagator = Propagator::new(
        plan.initial_state,
        crate::physics::types::Epoch::from_seconds(0.0)?,
        plan.propagation,
    )?;

    for burn in &plan.initial_burns {
        propagator.apply_burn(*burn)?;
    }

    let mut segment_results = Vec::with_capacity(plan.segments.len());
    let mut final_epoch = crate::physics::types::Epoch::from_seconds(0.0)?;

    for (segment_index, segment) in plan.segments.iter().enumerate() {
        let propagation = propagator.run_segment(&segment.termination, Some(segment_index))?;
        final_epoch = propagation.final_epoch;

        let mut burns_applied = 0_usize;
        for burn in &segment.burns {
            propagator.apply_burn(*burn)?;
            burns_applied += 1;
        }

        segment_results.push(MissionSegmentResult {
            segment_index,
            propagation,
            burns_applied,
        });
    }

    let final_state = propagator.current_state()?;

    Ok(MissionResult {
        segments: segment_results,
        final_state,
        final_epoch,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};
    use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};
    use crate::physics::types::{Distance, GravitationalParameter};
    use crate::sim::config::{
        BurnConfig, ForceModelKind, InitialOrbitConfig, IntegratorKind, LoadedScenario,
        MissionSegmentConfig, PropagationConfig, StopConfig,
    };

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
            initial_burns: vec![BurnConfig::Prograde { delta_v_mps: 100.0 }],
            segments: vec![MissionSegmentConfig {
                stop: StopConfig::Apoapsis,
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
            initial_burns: vec![BurnConfig::Prograde { delta_v_mps: dv1 }],
            segments: vec![
                MissionSegmentConfig {
                    stop: StopConfig::Apoapsis,
                    burns: vec![BurnConfig::Circularize],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Duration {
                        duration_s: 86_400.0,
                    },
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
        assert_eq!(
            result.segments[0].propagation.stop_reason,
            crate::StopReason::ApoapsisMet
        );
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
    fn segment_duration_partial_steps_are_segment_relative() {
        let config = MissionScenarioConfig {
            name: "duration_segments".to_string(),
            kind: "mission".to_string(),
            initial_orbit: InitialOrbitConfig::Circular {
                altitude_m: 400_000.0,
            },
            propagation: PropagationConfig {
                dt_s: 10.0,
                integrator: IntegratorKind::Leapfrog,
                force_model: ForceModelKind::TwoBody,
                sample_every_steps: 1,
            },
            initial_burns: vec![],
            segments: vec![
                MissionSegmentConfig {
                    stop: StopConfig::Duration { duration_s: 25.0 },
                    burns: vec![],
                },
                MissionSegmentConfig {
                    stop: StopConfig::Duration { duration_s: 15.0 },
                    burns: vec![],
                },
            ],
        };

        let result = run_mission(&config).expect("mission run");

        assert!((result.segments[0].propagation.final_epoch.seconds() - 25.0).abs() < 1.0e-9);
        assert!((result.final_epoch.seconds() - 40.0).abs() < 1.0e-9);
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
