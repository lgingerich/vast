use crate::errors::{DynamicsError, NumericalError};
use crate::physics::state::{CartesianState, CartesianStateComponents};
use crate::physics::types::{Acceleration, Velocity};
use nalgebra::Vector3;

/// Thrust guidance mode: the direction logic, decoupled from magnitude semantics.
///
/// The same mode resolves to an `Acceleration` for continuous thrust (integrated as
/// a force by the dynamics model) or to a `Velocity` delta-v for impulsive burns
/// (applied instantaneously at a segment boundary). Magnitudes are interpreted as
/// m/s² in the acceleration path and m/s in the delta-v path.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ThrustMode {
    /// No thrust applied.
    Coast,
    /// Fully specified inertial vector. Magnitude and direction are explicit;
    /// no state-dependent resolution needed.
    Inertial(Vector3<f64>),
    /// Thrust direction aligned with instantaneous velocity.
    /// Magnitude is acceleration (m/s²) for continuous, delta-v (m/s) for impulse.
    Prograde(f64),
}

impl ThrustMode {
    /// Creates an inertial thrust mode from Cartesian components.
    ///
    /// Panics in debug builds if any component is not finite.
    pub fn inertial(components: [f64; 3]) -> Self {
        debug_assert!(components.iter().all(|c| c.is_finite()));
        Self::Inertial(Vector3::new(components[0], components[1], components[2]))
    }

    /// No-thrust sentinel.
    pub const fn coast() -> Self {
        Self::Coast
    }

    /// Creates a prograde thrust mode from a magnitude.
    ///
    /// Panics in debug builds if the magnitude is not finite.
    pub fn prograde(magnitude: f64) -> Self {
        debug_assert!(magnitude.is_finite());
        Self::Prograde(magnitude)
    }

    /// Resolves this guidance mode to an inertial acceleration for continuous thrust.
    ///
    /// Prograde direction comes from instantaneous velocity; inertial is a passthrough.
    /// Returns an error if the velocity has zero norm (prograde direction is undefined).
    pub fn resolve_acceleration(
        &self,
        state: CartesianStateComponents,
    ) -> Result<Acceleration, DynamicsError> {
        match self {
            Self::Coast => Ok(Acceleration::ZERO),
            Self::Inertial(v) => Ok(Acceleration::from_vector(*v)),
            Self::Prograde(magnitude) => {
                let v = state.velocity_vector();
                let speed = v.norm();
                if speed == 0.0 {
                    return Err(DynamicsError::Numerical(NumericalError::ZeroNorm {
                        field: "velocity",
                    }));
                }
                Ok(Acceleration::from_vector(v / speed * *magnitude))
            }
        }
    }

    /// Resolves this guidance mode to an inertial delta-v for an impulsive burn.
    ///
    /// Prograde direction comes from the spacecraft's own velocity.
    /// Returns an error if the velocity has zero norm.
    pub fn resolve_delta_v(&self, state: CartesianState) -> Result<Velocity, DynamicsError> {
        match self {
            Self::Coast => Ok(Velocity::from_vector(Vector3::zeros())),
            Self::Inertial(v) => Ok(Velocity::from_vector(*v)),
            Self::Prograde(delta_v_mps) => {
                let velocity = state.velocity.vector();
                let speed = velocity.norm();
                if speed == 0.0 {
                    return Err(DynamicsError::Numerical(NumericalError::ZeroNorm {
                        field: "velocity",
                    }));
                }
                Ok(Velocity::from_vector(velocity / speed * *delta_v_mps))
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::R_EARTH;
    use crate::physics::types::Position;

    fn state(position: Vector3<f64>, velocity: Vector3<f64>) -> CartesianState {
        CartesianState::new(
            Position::from_vector(position),
            Velocity::from_vector(velocity),
        )
    }

    #[test]
    fn prograde_burn_adds_along_velocity_direction() {
        let initial = state(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let dv = ThrustMode::prograde(10.0)
            .resolve_delta_v(initial)
            .expect("resolved");

        let velocity = initial.velocity.vector();
        let expected = velocity / velocity.norm() * 10.0;
        assert!((dv.vector().x - expected.x).abs() < 1.0e-12);
        assert!((dv.vector().y - expected.y).abs() < 1.0e-12);
        assert!((dv.vector().z - expected.z).abs() < 1.0e-12);
    }

    #[test]
    fn resolve_acceleration_prograde_matches_direction() {
        let state_components = CartesianStateComponents::from_vectors(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let accel = ThrustMode::prograde(0.5)
            .resolve_acceleration(state_components)
            .expect("resolved");

        let a = accel.vector();
        assert!(a.x.abs() < 1.0e-12);
        assert!((a.y - 0.5).abs() < 1.0e-12);
        assert!(a.z.abs() < 1.0e-12);
        assert!((a.norm() - 0.5).abs() < 1.0e-12);
    }

    #[test]
    fn resolve_acceleration_inertial_passes_vector_through() {
        let state_components = CartesianStateComponents::from_vectors(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::zeros(),
        );

        let accel = ThrustMode::inertial([1.0, 2.0, 3.0])
            .resolve_acceleration(state_components)
            .expect("resolved");

        assert_eq!(accel.vector(), Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn resolve_acceleration_coast_is_zero() {
        let state_components = CartesianStateComponents::from_vectors(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::zeros(),
        );

        let accel = ThrustMode::Coast
            .resolve_acceleration(state_components)
            .expect("resolved");

        assert_eq!(accel, Acceleration::ZERO);
    }

    #[test]
    fn resolve_delta_v_coast_is_zero() {
        let initial = state(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let dv = ThrustMode::Coast
            .resolve_delta_v(initial)
            .expect("resolved");

        assert_eq!(dv.vector(), Vector3::zeros());
    }
}
