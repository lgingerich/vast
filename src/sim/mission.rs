use super::config::MissionScenarioConfig;
use super::engine::{PropagationResult, Propagator};
use crate::errors::MissionError;
use crate::physics::state::CartesianState;
use crate::physics::types::Epoch;

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
    pub final_epoch: Epoch,
}

/// Runs a mission scenario once from its initial state.
pub fn run_mission(config: &MissionScenarioConfig) -> Result<MissionResult, MissionError> {
    let plan = config.build()?;
    let mut propagator = Propagator::new(plan.initial_state, plan.frame, plan.propagation)?;

    for burn in &plan.initial_burns {
        propagator.apply_burn_config(*burn)?;
    }

    let mut segment_results = Vec::with_capacity(plan.segments.len());
    let mut final_epoch = Epoch::ZERO;

    for (segment_index, segment) in plan.segments.iter().enumerate() {
        let propagation = propagator.run_segment(&segment.termination, Some(segment_index))?;
        final_epoch = propagation.final_epoch;

        let burns_applied = segment.burns.len();
        for burn in &segment.burns {
            propagator.apply_burn_config(*burn)?;
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
    use crate::sim::config::{
        BodyConfig, BurnConfig, ForceModelEntry, InitialOrbitConfig, IntegratorSelection,
        LoadedScenario, MissionSegmentConfig, PropagationConfig, StopConfig,
    };

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
                integrator: IntegratorSelection::Leapfrog,
                body: BodyConfig::Earth,
                force_models: vec![ForceModelEntry::TwoBody],
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
                integrator: IntegratorSelection::Leapfrog,
                body: BodyConfig::Earth,
                force_models: vec![ForceModelEntry::TwoBody],
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
