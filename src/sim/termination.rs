use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};
use crate::physics::state::CartesianState;
use crate::physics::types::{Duration, Epoch, GravitationalParameter, TimeStep};

const DEFAULT_ORBITAL_MIN_STEPS: usize = 1;
const ORBITAL_ANGLE_TOLERANCE_RAD: f64 = 0.05;

/// Why propagation stopped.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopReason {
    /// Target elapsed duration was reached.
    DurationMet,
    /// Target step count was reached.
    StepCountMet,
    /// Propagation hit the configured step budget.
    StepBudgetExceeded,
    /// An apoapsis crossing was detected at the step boundary.
    ApoapsisMet,
    /// A periapsis crossing was detected at the step boundary.
    PeriapsisMet,
}

/// End-state rule for a propagation segment.
#[derive(Debug, Clone, PartialEq)]
pub enum TerminationCondition {
    /// Stop once elapsed time from the start epoch reaches this duration.
    Duration(Duration),
    /// Stop after this many integrator steps.
    StepCount(usize),
    /// Stop on the first apoapsis after `min_steps` integrator steps.
    Apoapsis {
        /// Gravitational parameter for element extraction.
        mu: GravitationalParameter,
        /// Minimum integrator steps before an apoapsis can trigger.
        min_steps: usize,
    },
    /// Stop on the first periapsis after `min_steps` integrator steps.
    Periapsis {
        /// Gravitational parameter for element extraction.
        mu: GravitationalParameter,
        /// Minimum integrator steps before a periapsis can trigger.
        min_steps: usize,
    },
    /// Stop when any child condition is satisfied (first match wins).
    AnyOf(Vec<TerminationCondition>),
}

impl TerminationCondition {
    /// Creates an apoapsis stop with the default minimum step guard.
    #[must_use]
    pub fn apoapsis(mu: GravitationalParameter) -> Self {
        Self::Apoapsis {
            mu,
            min_steps: DEFAULT_ORBITAL_MIN_STEPS,
        }
    }

    /// Creates a periapsis stop with the default minimum step guard.
    #[must_use]
    pub fn periapsis(mu: GravitationalParameter) -> Self {
        Self::Periapsis {
            mu,
            min_steps: DEFAULT_ORBITAL_MIN_STEPS,
        }
    }

    /// Returns the stop reason if the condition is satisfied.
    #[must_use]
    pub fn stop_reason(&self, context: TerminationContext<'_>) -> Option<StopReason> {
        match self {
            Self::Duration(duration) => {
                let elapsed = context.current_epoch.duration_since(context.start_epoch);
                (elapsed >= *duration).then_some(StopReason::DurationMet)
            }
            Self::StepCount(limit) => (context.steps >= *limit).then_some(StopReason::StepCountMet),
            Self::Apoapsis { mu, min_steps } => apoapsis_met(*mu, *min_steps, context),
            Self::Periapsis { mu, min_steps } => periapsis_met(*mu, *min_steps, context),
            Self::AnyOf(conditions) => conditions
                .iter()
                .find_map(|condition| condition.stop_reason(context)),
        }
    }

    /// Remaining time step until a duration-based stop, if applicable.
    #[must_use]
    pub fn remaining(&self, start_epoch: Epoch, current_epoch: Epoch) -> Option<TimeStep> {
        match self {
            Self::Duration(duration) => {
                let elapsed = current_epoch.duration_since(start_epoch);
                if elapsed >= *duration {
                    return None;
                }
                let remaining_seconds = duration.seconds() - elapsed.seconds();
                TimeStep::from_seconds(remaining_seconds).ok()
            }
            Self::AnyOf(conditions) => conditions
                .iter()
                .filter_map(|condition| condition.remaining(start_epoch, current_epoch))
                .min_by(|left, right| {
                    left.seconds()
                        .partial_cmp(&right.seconds())
                        .unwrap_or(std::cmp::Ordering::Equal)
                }),
            Self::StepCount(_) | Self::Apoapsis { .. } | Self::Periapsis { .. } => None,
        }
    }

    /// Whether this condition requires orbital phase angle computation.
    #[must_use]
    pub fn needs_orbital_phase(&self) -> bool {
        match self {
            Self::Duration(_) | Self::StepCount(_) => false,
            Self::Apoapsis { .. } | Self::Periapsis { .. } => true,
            Self::AnyOf(conditions) => conditions.iter().any(|c| c.needs_orbital_phase()),
        }
    }
}

/// Inputs for evaluating a termination condition at a step boundary.
#[derive(Debug, Clone, Copy)]
pub struct TerminationContext<'a> {
    /// Segment start epoch.
    pub start_epoch: Epoch,
    /// Current epoch after the latest step.
    pub current_epoch: Epoch,
    /// Integrator steps taken in this segment.
    pub steps: usize,
    /// Current Cartesian state.
    pub state: &'a CartesianState,
    /// Previous orbital phase angle in radians, if known.
    pub previous_orbital_angle_rad: Option<f64>,
    /// Previous radial velocity in meters per second, if known.
    pub previous_radial_velocity_mps: Option<f64>,
    /// Pre-computed current orbital phase angle in radians, if available.
    ///
    /// When supplied by the caller, `orbital_crossing` uses this value
    /// instead of re-computing `OrbitalElements::from_state`.
    pub current_orbital_phase_angle_rad: Option<f64>,
}

fn apoapsis_met(
    mu: GravitationalParameter,
    min_steps: usize,
    context: TerminationContext<'_>,
) -> Option<StopReason> {
    if context.steps < min_steps {
        return None;
    }
    if radial_velocity_crossed(context, true) {
        return Some(StopReason::ApoapsisMet);
    }
    orbital_crossing(mu, context, std::f64::consts::PI).then_some(StopReason::ApoapsisMet)
}

fn periapsis_met(
    mu: GravitationalParameter,
    min_steps: usize,
    context: TerminationContext<'_>,
) -> Option<StopReason> {
    if context.steps < min_steps {
        return None;
    }
    if radial_velocity_crossed(context, false) {
        return Some(StopReason::PeriapsisMet);
    }
    orbital_crossing(mu, context, 0.0).then_some(StopReason::PeriapsisMet)
}

fn radial_velocity_crossed(context: TerminationContext<'_>, apoapsis: bool) -> bool {
    let Some(previous) = context.previous_radial_velocity_mps else {
        return false;
    };
    let current = radial_velocity_mps(context.state);
    if apoapsis {
        previous > 0.0 && current <= 0.0
    } else {
        previous < 0.0 && current >= 0.0
    }
}

fn radial_velocity_mps(state: &CartesianState) -> f64 {
    let position = state.position.vector();
    let velocity = state.velocity.vector();
    let radius = position.norm();
    if radius <= 0.0 {
        return 0.0;
    }
    position.dot(&velocity) / radius
}

fn orbital_crossing(
    mu: GravitationalParameter,
    context: TerminationContext<'_>,
    target_rad: f64,
) -> bool {
    let current_angle = match context.current_orbital_phase_angle_rad {
        Some(angle) => angle,
        None => {
            let Some(computed) = orbital_phase_angle_rad(context.state, mu) else {
                return false;
            };
            computed
        }
    };
    let Some(previous_angle) = context.previous_orbital_angle_rad else {
        return angle_near(current_angle, target_rad);
    };

    crossed_angle(previous_angle, current_angle, target_rad)
        || angle_near(current_angle, target_rad)
}

fn orbital_phase_angle_rad(state: &CartesianState, mu: GravitationalParameter) -> Option<f64> {
    let elements =
        OrbitalElements::from_state(state, mu, OrbitalElementClassification::default()).ok()?;
    Some(elements.orbital_phase_angle().radians())
}

fn normalize_angle_rad(angle_rad: f64) -> f64 {
    let mut angle = angle_rad % std::f64::consts::TAU;
    if angle < 0.0 {
        angle += std::f64::consts::TAU;
    }
    angle
}

fn angle_near(angle_rad: f64, target_rad: f64) -> bool {
    let delta = (normalize_angle_rad(angle_rad) - normalize_angle_rad(target_rad)).abs();
    delta <= ORBITAL_ANGLE_TOLERANCE_RAD
        || (std::f64::consts::TAU - delta).abs() <= ORBITAL_ANGLE_TOLERANCE_RAD
}

fn crossed_angle(previous_rad: f64, current_rad: f64, target_rad: f64) -> bool {
    let previous = normalize_angle_rad(previous_rad);
    let current = normalize_angle_rad(current_rad);
    let target = normalize_angle_rad(target_rad);

    if previous <= current {
        previous < target && target <= current + ORBITAL_ANGLE_TOLERANCE_RAD
    } else {
        previous < target || target <= current + ORBITAL_ANGLE_TOLERANCE_RAD
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Angle, Distance, Eccentricity, EllipticOrbit, GravitationalParameter, MU_EARTH,
        OrbitDirection, R_EARTH,
    };

    #[test]
    fn duration_met_when_elapsed_reaches_target() {
        let start = Epoch::from_seconds(0.0).expect("epoch");
        let current = Epoch::from_seconds(100.0).expect("epoch");
        let condition =
            TerminationCondition::Duration(Duration::from_seconds(100.0).expect("duration"));
        let state = sample_state();

        assert_eq!(
            condition.stop_reason(TerminationContext {
                start_epoch: start,
                current_epoch: current,
                steps: 0,
                state: &state,
                previous_orbital_angle_rad: None,
                previous_radial_velocity_mps: None,
                current_orbital_phase_angle_rad: None,
            }),
            Some(StopReason::DurationMet)
        );
    }

    #[test]
    fn step_count_met_after_limit() {
        let start = Epoch::from_seconds(0.0).expect("epoch");
        let current = Epoch::from_seconds(1.0).expect("epoch");
        let condition = TerminationCondition::StepCount(5);
        let state = sample_state();

        assert_eq!(
            condition.stop_reason(TerminationContext {
                start_epoch: start,
                current_epoch: current,
                steps: 4,
                state: &state,
                previous_orbital_angle_rad: None,
                previous_radial_velocity_mps: None,
                current_orbital_phase_angle_rad: None,
            }),
            None
        );
        assert_eq!(
            condition.stop_reason(TerminationContext {
                start_epoch: start,
                current_epoch: current,
                steps: 5,
                state: &state,
                previous_orbital_angle_rad: None,
                previous_radial_velocity_mps: None,
                current_orbital_phase_angle_rad: None,
            }),
            Some(StopReason::StepCountMet)
        );
    }

    fn sample_state() -> crate::CartesianState {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).expect("mu");
        EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 400_000.0).expect("radius"),
            Eccentricity::from_scalar(0.0).expect("eccentricity"),
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0).expect("phase"),
            OrbitDirection::Prograde,
        )
        .expect("orbit")
        .to_cartesian_state()
        .expect("state")
    }
}
