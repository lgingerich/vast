#![allow(missing_docs)]

use vast::{
    Angle, Distance, Duration, EllipticOrbit, GravitationalParameter, J2Coefficient, J2_EARTH,
    MU_EARTH, OblateBodyGravity, Orbit, SimBuilder, TimeStep, TwoBodyGravity,
    VelocityVerletIntegrator, R_EARTH,
};

const ALTITUDE_KM: f64 = 600.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 30.0;
const PROPAGATION_DAYS: f64 = 3.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let orbit_spec = Orbit::CircularInclined {
        altitude: Distance::new(ALTITUDE_KM * 1000.0)?,
        inclination: Angle::from_degrees(INCLINATION_DEG)?,
        raan: Angle::from_degrees(RAAN_DEG)?,
    };
    let elliptic: EllipticOrbit = orbit_spec.try_into()?;
    let initial_raan_deg = RAAN_DEG;
    let duration = Duration::new(PROPAGATION_DAYS * 86_400.0)?;
    let dt = TimeStep::new(STEP_SECONDS)?;

    // --- analytical prediction ---
    let mean_motion =
        (MU_EARTH / elliptic.semi_major_axis().meters().powi(3)).sqrt();
    let radius_ratio = R_EARTH / elliptic.semi_major_axis().meters();
    let analytical_drift_rad_per_sec = -1.5
        * J2_EARTH
        * mean_motion
        * radius_ratio
        * radius_ratio
        * INCLINATION_DEG.to_radians().cos();
    let analytical_drift_deg_per_day = analytical_drift_rad_per_sec.to_degrees() * 86_400.0;

    // --- run two-body simulation ---
    let two_body = SimBuilder::new()
        .orbit(orbit_spec)?
        .dynamics(TwoBodyGravity::earth())
        .integrator(VelocityVerletIntegrator)
        .timestep(dt)
        .duration(duration)?;

    // --- run J2 simulation ---
    let j2_dynamics = OblateBodyGravity::new(
        GravitationalParameter::earth(),
        Distance::new(R_EARTH)?,
        J2Coefficient::new(J2_EARTH)?,
    );
    let j2 = SimBuilder::new()
        .orbit(orbit_spec)?
        .dynamics(j2_dynamics)
        .integrator(VelocityVerletIntegrator)
        .timestep(dt)
        .duration(duration)?;

    // --- inspect results ---
    let final_two_body_raan = two_body
        .elements
        .right_ascension_of_ascending_node()
        .map(|a| a.degrees())
        .ok_or("two-body RAAN is undefined")?;
    let final_j2_raan = j2
        .elements
        .right_ascension_of_ascending_node()
        .map(|a| a.degrees())
        .ok_or("J2 RAAN is undefined")?;

    let two_body_drift = angle_delta_degrees(final_two_body_raan, initial_raan_deg);
    let j2_drift = angle_delta_degrees(final_j2_raan, initial_raan_deg);
    let simulated_drift_deg_per_day = j2_drift / PROPAGATION_DAYS;

    // --- print report ---
    println!(
        "J2 precession demo: altitude {:.0} km | inclination {:.1}°",
        elliptic.semi_major_axis().meters() / 1000.0 - R_EARTH / 1000.0,
        INCLINATION_DEG,
    );
    println!(
        "elapsed: {:.2} days | step: {:.1} s",
        duration.seconds() / 86_400.0,
        dt.seconds(),
    );
    println!("initial RAAN: {:.6}°", initial_raan_deg);
    println!(
        "two-body RAAN: {:.6}° | drift: {:+.6}°/day",
        final_two_body_raan,
        two_body_drift / PROPAGATION_DAYS,
    );
    println!(
        "J2 RAAN: {:.6}° | drift: {:+.6}°/day",
        final_j2_raan, simulated_drift_deg_per_day,
    );
    println!(
        "analytical J2 drift: {:+.6}°/day | error: {:+.6}°/day",
        analytical_drift_deg_per_day,
        simulated_drift_deg_per_day - analytical_drift_deg_per_day,
    );

    Ok(())
}

fn angle_delta_degrees(final_degrees: f64, initial_degrees: f64) -> f64 {
    let delta = (final_degrees - initial_degrees).rem_euclid(360.0);
    if delta > 180.0 { delta - 360.0 } else { delta }
}
