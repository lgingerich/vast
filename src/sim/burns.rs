use crate::errors::{MissionError, NumericalError, PropagationError};
use crate::physics::maneuvers::{circularize_delta_v, prograde_delta_v};
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

    pub(crate) fn apply_to(
        self,
        state: CartesianState,
        mu: GravitationalParameter,
    ) -> Result<CartesianState, MissionError> {
        let delta_v = match self {
            Self::Inertial(delta_v_mps) => {
                Velocity::from_vector(Vector3::new(delta_v_mps[0], delta_v_mps[1], delta_v_mps[2]))
            }
            Self::Prograde(delta_v_mps) => prograde_delta_v(delta_v_mps, &state)?,
            Self::Circularize => circularize_delta_v(&state, mu)?,
        };
        apply_delta_v(state, delta_v).map_err(MissionError::from)
    }
}

fn apply_delta_v(
    state: CartesianState,
    delta_v: Velocity,
) -> Result<CartesianState, PropagationError> {
    let velocity = state.velocity.vector();
    let delta_v = delta_v.vector();
    let updated = Velocity::from_vector(Vector3::new(
        velocity.x + delta_v.x,
        velocity.y + delta_v.y,
        velocity.z + delta_v.z,
    ));
    Ok(CartesianState::new(state.position, updated))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};
    use crate::physics::types::{Position, Velocity};

    fn mu() -> GravitationalParameter {
        GravitationalParameter::new(MU_EARTH)
    }

    fn state(position: Vector3<f64>, velocity: Vector3<f64>) -> CartesianState {
        CartesianState::new(
            Position::from_vector(position),
            Velocity::from_vector(velocity),
        )
    }

    #[test]
    fn inertial_burn_rejects_non_finite_delta_v() {
        let error = BurnCommand::inertial([f64::NAN, 0.0, 0.0])
            .expect_err("non-finite delta-v should fail");

        assert!(matches!(
            error,
            MissionError::Numerical(NumericalError::NotFinite { field: "delta_v" })
        ));
    }

    #[test]
    fn prograde_burn_adds_along_velocity_direction() {
        let initial = state(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let updated = BurnCommand::prograde(10.0)
            .expect("burn")
            .apply_to(initial, mu())
            .expect("applied");

        let velocity = updated.velocity.vector();
        assert!((velocity.x - 0.0).abs() < 1.0e-12);
        assert!((velocity.y - 7_710.0).abs() < 1.0e-12);
        assert!((velocity.z - 0.0).abs() < 1.0e-12);
    }

    #[test]
    fn circularize_removes_radial_velocity_and_matches_circular_speed() {
        let radius_m = R_EARTH + 400_000.0;
        let initial = state(
            Vector3::new(radius_m, 0.0, 0.0),
            Vector3::new(25.0, 7_500.0, 0.0),
        );

        let updated = BurnCommand::circularize()
            .apply_to(initial, mu())
            .expect("applied");

        let velocity = updated.velocity.vector();
        let circular_speed = (MU_EARTH / radius_m).sqrt();
        assert!(velocity.x.abs() < 1.0e-9);
        assert!((velocity.y - circular_speed).abs() < 1.0e-9);
        assert!(velocity.z.abs() < 1.0e-9);
    }

    #[test]
    fn circularize_rejects_undefined_tangential_velocity() {
        let initial = state(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::new(25.0, 0.0, 0.0),
        );

        let error = BurnCommand::circularize()
            .apply_to(initial, mu())
            .expect_err("purely radial velocity cannot circularize");

        assert!(matches!(
            error,
            MissionError::Numerical(NumericalError::ZeroNorm {
                field: "tangential velocity"
            })
        ));
    }
}
