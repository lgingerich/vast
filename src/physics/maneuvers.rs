use crate::errors::NumericalError;
use crate::physics::state::CartesianState;
use crate::physics::types::{Distance, GravitationalParameter, Velocity};
use nalgebra::Vector3;

/// Inertial delta-v vector for a burn aligned with the current velocity.
pub fn prograde_delta_v(
    delta_v_mps: f64,
    state: &CartesianState,
) -> Result<Velocity, NumericalError> {
    if !delta_v_mps.is_finite() {
        return Err(NumericalError::NotFinite { field: "delta_v" });
    }

    let velocity = state.velocity.vector();
    let speed = velocity.norm();
    if speed <= 0.0 {
        return Err(NumericalError::ZeroNorm { field: "velocity" });
    }

    Ok(Velocity::from_vector(velocity / speed * delta_v_mps))
}

/// Inertial delta-v vector that removes radial velocity and matches circular speed.
pub fn circularize_delta_v(
    state: &CartesianState,
    mu: GravitationalParameter,
) -> Result<Velocity, NumericalError> {
    let position = state.position.vector();
    let velocity = state.velocity.vector();
    let radius_m = position.norm();
    if radius_m <= 0.0 {
        return Err(NumericalError::ZeroNorm { field: "position" });
    }

    let radial_hat = position / radius_m;
    let radial_speed_mps = velocity.dot(&radial_hat);
    let tangential_velocity = velocity - radial_hat * radial_speed_mps;
    let tangential_speed_mps = tangential_velocity.norm();
    if tangential_speed_mps <= 0.0 {
        return Err(NumericalError::ZeroNorm {
            field: "tangential velocity",
        });
    }

    let target_tangential_speed_mps = (mu.meters_cubed_per_second_squared() / radius_m).sqrt();
    let tangential_hat = tangential_velocity / tangential_speed_mps;
    let delta_v = tangential_hat * (target_tangential_speed_mps - tangential_speed_mps)
        - radial_hat * radial_speed_mps;
    Ok(Velocity::from_vector(Vector3::new(
        delta_v.x, delta_v.y, delta_v.z,
    )))
}

/// Analytical two-body bi-elliptic transfer delta-vs between circular orbits.
///
/// Returns `(departure_burn, apoapsis_burn, arrival_burn)` in meters per second.
/// The arrival burn is negative when the transfer reaches the target radius
/// faster than circular speed; use circularization at that boundary.
///
/// The intermediate apoapsis radius must exceed both `departure_radius` and
/// `arrival_radius`.
#[must_use]
#[allow(dead_code)]
pub fn bielliptic_delta_v(
    mu: GravitationalParameter,
    departure_radius: Distance,
    arrival_radius: Distance,
    intermediate_radius: Distance,
) -> Option<(f64, f64, f64)> {
    let mu_m3ps2 = mu.meters_cubed_per_second_squared();
    let departure_radius_m = departure_radius.meters();
    let arrival_radius_m = arrival_radius.meters();
    let intermediate_radius_m = intermediate_radius.meters();

    if intermediate_radius_m <= departure_radius_m.max(arrival_radius_m) {
        return None;
    }

    let first_semi_major_axis = 0.5 * (departure_radius_m + intermediate_radius_m);
    let departure_circular_speed = (mu_m3ps2 / departure_radius_m).sqrt();
    let departure_transfer_speed =
        (mu_m3ps2 * (2.0 / departure_radius_m - 1.0 / first_semi_major_axis)).sqrt();
    let first_transfer_speed_at_intermediate =
        (mu_m3ps2 * (2.0 / intermediate_radius_m - 1.0 / first_semi_major_axis)).sqrt();

    let second_semi_major_axis = 0.5 * (arrival_radius_m + intermediate_radius_m);
    let second_transfer_speed_at_intermediate =
        (mu_m3ps2 * (2.0 / intermediate_radius_m - 1.0 / second_semi_major_axis)).sqrt();
    let arrival_transfer_speed =
        (mu_m3ps2 * (2.0 / arrival_radius_m - 1.0 / second_semi_major_axis)).sqrt();
    let arrival_circular_speed = (mu_m3ps2 / arrival_radius_m).sqrt();

    let dv1 = departure_transfer_speed - departure_circular_speed;
    let dv2 = second_transfer_speed_at_intermediate - first_transfer_speed_at_intermediate;
    let dv3 = arrival_circular_speed - arrival_transfer_speed;

    if !dv1.is_finite() || !dv2.is_finite() || !dv3.is_finite() {
        return None;
    }

    Some((dv1, dv2, dv3))
}

/// Analytical two-body Hohmann transfer delta-vs between circular orbits.
///
/// Returns `(departure_burn, arrival_burn)` in meters per second.
/// The arrival burn is negative when the transfer reaches the target radius
/// faster than circular speed; use circularization at that boundary.
#[must_use]
#[allow(dead_code)]
pub fn hohmann_delta_v(
    mu: GravitationalParameter,
    departure_radius: Distance,
    arrival_radius: Distance,
) -> Option<(f64, f64)> {
    let mu_m3ps2 = mu.meters_cubed_per_second_squared();
    let departure_radius_m = departure_radius.meters();
    let arrival_radius_m = arrival_radius.meters();

    let semi_major_axis = 0.5 * (departure_radius_m + arrival_radius_m);
    let departure_circular_speed = (mu_m3ps2 / departure_radius_m).sqrt();
    let departure_transfer_speed =
        (mu_m3ps2 * (2.0 / departure_radius_m - 1.0 / semi_major_axis)).sqrt();
    let arrival_transfer_speed =
        (mu_m3ps2 * (2.0 / arrival_radius_m - 1.0 / semi_major_axis)).sqrt();
    let arrival_circular_speed = (mu_m3ps2 / arrival_radius_m).sqrt();

    let dv1 = departure_transfer_speed - departure_circular_speed;
    let dv2 = arrival_circular_speed - arrival_transfer_speed;

    if !dv1.is_finite() || !dv2.is_finite() {
        return None;
    }

    Some((dv1, dv2))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::R_EARTH;
    use crate::physics::types::{Position, Velocity};
    use nalgebra::Vector3;

    fn state(position: Vector3<f64>, velocity: Vector3<f64>) -> CartesianState {
        CartesianState::new(
            Position::from_vector(position),
            Velocity::from_vector(velocity),
        )
    }

    #[test]
    fn bielliptic_rejects_invalid_intermediate_radius() {
        let mu = GravitationalParameter::new(3.986e14);
        let departure = Distance::new(R_EARTH + 400_000.0);
        let arrival = Distance::new(R_EARTH + 35_786_000.0);
        let intermediate = Distance::new(R_EARTH + 400_000.0);
        assert!(bielliptic_delta_v(mu, departure, arrival, intermediate).is_none());
    }

    #[test]
    fn prograde_delta_v_rejects_zero_velocity() {
        let state = state(
            Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0),
            Vector3::zeros(),
        );

        assert!(matches!(
            prograde_delta_v(10.0, &state),
            Err(NumericalError::ZeroNorm { field: "velocity" })
        ));
    }

    #[test]
    fn circularize_delta_v_removes_radial_velocity() {
        let mu = GravitationalParameter::new(3.986e14);
        let radius_m = R_EARTH + 400_000.0;
        let state = state(
            Vector3::new(radius_m, 0.0, 0.0),
            Vector3::new(20.0, 7_500.0, 0.0),
        );

        let delta_v = circularize_delta_v(&state, mu).expect("delta-v").vector();
        let updated_velocity = state.velocity.vector() + delta_v;
        let circular_speed = (mu.meters_cubed_per_second_squared() / radius_m).sqrt();

        assert!(updated_velocity.x.abs() < 1.0e-9);
        assert!((updated_velocity.y - circular_speed).abs() < 1.0e-9);
        assert!(updated_velocity.z.abs() < 1.0e-9);
    }
}
