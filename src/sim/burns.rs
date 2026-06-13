use crate::errors::{MissionError, NumericalError, PropagationError};
use crate::physics::state::CartesianState;
use crate::physics::types::{GravitationalParameter, Velocity};
use nalgebra::Vector3;

/// Runtime impulsive burn command applied at a step boundary.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) enum BurnCommand {
    /// Inertial Cartesian delta-v in meters per second.
    Inertial([f64; 3]),
    /// Prograde delta-v magnitude in meters per second.
    Prograde(f64),
    /// Match the local circular speed at the current radius.
    Circularize,
}

impl BurnCommand {
    pub(crate) fn inertial(delta_v_mps: [f64; 3]) -> Result<Self, MissionError> {
        if !delta_v_mps.iter().all(|component| component.is_finite()) {
            return Err(MissionError::Numerical(NumericalError::NotFinite {
                field: "delta_v",
            }));
        }
        Ok(Self::Inertial(delta_v_mps))
    }

    pub(crate) fn prograde(delta_v_mps: f64) -> Result<Self, MissionError> {
        if !delta_v_mps.is_finite() {
            return Err(MissionError::Numerical(NumericalError::NotFinite {
                field: "delta_v",
            }));
        }
        Ok(Self::Prograde(delta_v_mps))
    }

    pub(crate) const fn circularize() -> Self {
        Self::Circularize
    }

    pub(crate) fn resolve(
        self,
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<ResolvedBurn, MissionError> {
        match self {
            Self::Inertial(delta_v_mps) => Ok(ResolvedBurn { delta_v_mps }),
            Self::Prograde(delta_v_mps) => ResolvedBurn::prograde(delta_v_mps, state),
            Self::Circularize => ResolvedBurn::circularize(state, mu),
        }
    }
}

/// Concrete inertial delta-v ready to apply to a state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub(crate) struct ResolvedBurn {
    delta_v_mps: [f64; 3],
}

impl ResolvedBurn {
    fn prograde(delta_v_mps: f64, state: &CartesianState) -> Result<Self, MissionError> {
        let velocity = state.velocity.vector();
        let speed = velocity.norm();
        if speed <= 0.0 || !speed.is_finite() {
            return Err(MissionError::Propagation(PropagationError::InvalidBurn));
        }
        let direction = velocity / speed;
        Ok(Self {
            delta_v_mps: [
                direction.x * delta_v_mps,
                direction.y * delta_v_mps,
                direction.z * delta_v_mps,
            ],
        })
    }

    fn circularize(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<Self, MissionError> {
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
        Ok(Self {
            delta_v_mps: [delta_v.x, delta_v.y, delta_v.z],
        })
    }

    pub(crate) fn apply_to(
        self,
        state: CartesianState,
    ) -> Result<CartesianState, PropagationError> {
        let velocity = state.velocity.vector();
        let updated = Velocity::from_vector(Vector3::new(
            velocity.x + self.delta_v_mps[0],
            velocity.y + self.delta_v_mps[1],
            velocity.z + self.delta_v_mps[2],
        ))?;
        Ok(CartesianState::new(state.position, updated))
    }
}
