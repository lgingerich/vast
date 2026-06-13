use crate::physics::types::{Distance, GravitationalParameter};

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

    #[test]
    fn bielliptic_rejects_invalid_intermediate_radius() {
        let mu =
            GravitationalParameter::from_meters_cubed_per_second_squared(3.986e14).expect("mu");
        let departure = Distance::from_meters(R_EARTH + 400_000.0).expect("distance");
        let arrival = Distance::from_meters(R_EARTH + 35_786_000.0).expect("distance");
        let intermediate = Distance::from_meters(R_EARTH + 400_000.0).expect("distance");
        assert!(bielliptic_delta_v(mu, departure, arrival, intermediate).is_none());
    }
}
