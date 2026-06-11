#![allow(missing_docs)]

use vast::{
    Angle, CartesianStateBuffer, Distance, Duration, Eccentricity, EllipticOrbit, Epoch,
    GravitationalParameter, Integrator, J2_EARTH, MU_EARTH, OblateBodyGravity, OrbitDirection,
    OrbitalElements, R_EARTH, TimeStep, TwoBodyGravity, VelocityVerletIntegrator,
};

const ALTITUDE_M: f64 = 600_000.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 30.0;
const PROPAGATION_DAYS: f64 = 300.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
    let orbit = EllipticOrbit {
        semi_major_axis: Distance::from_meters(R_EARTH + ALTITUDE_M)?,
        eccentricity: Eccentricity::from_scalar(0.0)?,
        mu,
        inclination: Angle::from_radians(INCLINATION_DEG.to_radians())?,
        right_ascension_of_ascending_node: Angle::from_radians(RAAN_DEG.to_radians())?,
        argument_of_periapsis: Angle::ZERO,
        true_anomaly: Angle::from_radians(0.0)?,
        direction: OrbitDirection::Prograde,
    };
    let initial_state = orbit.to_cartesian_state()?;
    let two_body_gravity = TwoBodyGravity { mu };
    let j2_gravity = OblateBodyGravity {
        mu,
        equatorial_radius: Distance::from_meters(R_EARTH)?,
        j2: J2_EARTH,
    };
    let integrator = VelocityVerletIntegrator;
    let dt = TimeStep::from_seconds(STEP_SECONDS)?;
    let duration = Duration::from_seconds(PROPAGATION_DAYS * 86_400.0)?;
    // First-order J2 secular RAAN drift for comparison.
    let mean_motion =
        (mu.meters_cubed_per_second_squared() / orbit.semi_major_axis.meters().powi(3)).sqrt();
    let radius_ratio = R_EARTH / orbit.semi_major_axis.meters();
    let analytical_raan_rad_per_sec = -1.5
        * J2_EARTH
        * mean_motion
        * radius_ratio
        * radius_ratio
        * INCLINATION_DEG.to_radians().cos();
    let analytical_drift_per_day = analytical_raan_rad_per_sec.to_degrees() * 86_400.0;
    let mut two_body_states = CartesianStateBuffer::<1>::new();
    let mut j2_states = CartesianStateBuffer::<1>::new();
    two_body_states.write_from_state(0, &initial_state)?;
    j2_states.write_from_state(0, &initial_state)?;

    let initial_raan = raan_degrees(&OrbitalElements::from_states(
        &two_body_states.view(),
        0,
        mu,
    )?)
    .ok_or("initial RAAN is undefined")?;

    propagate(
        &integrator,
        &two_body_gravity,
        &mut two_body_states,
        duration,
        dt,
    )?;
    propagate(&integrator, &j2_gravity, &mut j2_states, duration, dt)?;

    let final_two_body_raan = raan_degrees(&OrbitalElements::from_states(
        &two_body_states.view(),
        0,
        mu,
    )?)
    .ok_or("final two-body RAAN is undefined")?;
    let final_j2_raan = raan_degrees(&OrbitalElements::from_states(&j2_states.view(), 0, mu)?)
        .ok_or("final J2 RAAN is undefined")?;
    let two_body_drift = angle_delta_degrees(final_two_body_raan, initial_raan);
    let j2_drift = angle_delta_degrees(final_j2_raan, initial_raan);
    let simulated_drift_per_day = j2_drift / PROPAGATION_DAYS;

    println!(
        "J2 precession demo: altitude {:.0} km | inclination {:.1} deg",
        ALTITUDE_M / 1000.0,
        INCLINATION_DEG,
    );
    println!(
        "elapsed: {:.2} days | step: {:.1} s",
        duration.seconds() / 86_400.0,
        dt.seconds(),
    );
    println!("initial RAAN: {:.6} deg", initial_raan);
    println!(
        "two-body RAAN: {:.6} deg | drift: {:+.6} deg/day",
        final_two_body_raan,
        two_body_drift / PROPAGATION_DAYS,
    );
    println!(
        "J2 RAAN: {:.6} deg | drift: {:+.6} deg/day",
        final_j2_raan, simulated_drift_per_day,
    );
    println!(
        "analytical J2 drift: {:+.6} deg/day | error: {:+.6} deg/day",
        analytical_drift_per_day,
        simulated_drift_per_day - analytical_drift_per_day,
    );

    Ok(())
}

fn propagate<I: Integrator, D: vast::Dynamics>(
    integrator: &I,
    dynamics: &D,
    states: &mut CartesianStateBuffer<1>,
    duration: Duration,
    dt: TimeStep,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut epoch = Epoch::from_seconds(0.0)?;

    while epoch.seconds() < duration.seconds() {
        let remaining_seconds = duration.seconds() - epoch.seconds();
        let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))?;
        integrator.step(dynamics, &mut states.view_mut(), 0, epoch, step)?;
        epoch = epoch.advance_by(step);
    }

    Ok(())
}

fn raan_degrees(elements: &OrbitalElements) -> Option<f64> {
    match elements {
        OrbitalElements::Classical(elements) => Some(
            elements
                .right_ascension_of_ascending_node
                .radians()
                .to_degrees(),
        ),
        OrbitalElements::CircularInclined(elements) => Some(
            elements
                .right_ascension_of_ascending_node
                .radians()
                .to_degrees(),
        ),
        OrbitalElements::CircularEquatorial(_) | OrbitalElements::EquatorialElliptic(_) => None,
    }
}

fn angle_delta_degrees(final_degrees: f64, initial_degrees: f64) -> f64 {
    let delta = (final_degrees - initial_degrees).rem_euclid(360.0);
    if delta > 180.0 { delta - 360.0 } else { delta }
}
