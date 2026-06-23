#![allow(missing_docs)]

use vast::{
    Angle, Distance, Duration, Frame, GravitationalParameter, Orbit, OrbitalDiagnostics,
    OrbitalElementClassification, OrbitalElements, SimBuilder, TimeStep, TwoBodyGravity,
    VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = 600.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 10.0;
const PERIODS: f64 = 3.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let frame = Frame::earth_inertial();
    let spec = Orbit::circular_inclined(
        Distance::new(ALTITUDE_KM * 1000.0),
        Angle::from_degrees(INCLINATION_DEG),
        Angle::from_degrees(RAAN_DEG),
    );
    let period = spec.period(&frame)?;
    let num_periods = Duration::new(period.seconds() * PERIODS);
    let dt = TimeStep::new(STEP_SECONDS);
    let mu: GravitationalParameter = frame.mu()?;

    // --- run simulation ---
    let mut sim = SimBuilder::new(frame)
        .orbit(spec)?
        .dynamics(TwoBodyGravity::from_frame(&frame)?)
        .integrator(VelocityVerletIntegrator)
        .build()?;
    let initial_state = sim.state()?;
    let initial_diag = OrbitalDiagnostics::from_state(&initial_state, mu)?;
    sim.advance(num_periods, dt)?;

    // --- inspect results ---
    let final_state = sim.state()?;
    let final_diag = OrbitalDiagnostics::from_state(&final_state, mu)?;
    let radius_drift = (final_diag.radius().meters() - initial_diag.radius().meters()).abs();
    let energy_error = (final_diag
        .specific_energy()
        .square_meters_per_square_second()
        - initial_diag
            .specific_energy()
            .square_meters_per_square_second())
    .abs();
    let h_error = (final_diag
        .specific_angular_momentum()
        .square_meters_per_second()
        - initial_diag
            .specific_angular_momentum()
            .square_meters_per_second())
    .abs();
    let h = final_state
        .position
        .vector()
        .cross(&final_state.velocity.vector());
    let h_hat = h.normalize();

    // --- print report ---
    println!(
        "Inclined circular orbit: altitude {:.0} km | inclination {:.1}° | RAAN {:.1}°",
        ALTITUDE_KM, INCLINATION_DEG, RAAN_DEG,
    );
    println!(
        "propagated: {:.1} periods | elapsed: {:.1} s | steps: {}",
        PERIODS,
        sim.epoch().seconds(),
        sim.steps(),
    );
    println!(
        "radius drift: {:.6} km | energy error: {:.6} m^2/s^2 | h error: {:.6} m^2/s",
        radius_drift.max(f64::EPSILON) / 1000.0,
        energy_error,
        h_error,
    );
    println!("eccentricity: {:.9}", final_diag.eccentricity().scalar(),);
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        h_hat.x, h_hat.y, h_hat.z,
    );
    let elements =
        OrbitalElements::from_state(&final_state, mu, OrbitalElementClassification::default())?;
    println!("{:?}", elements);

    Ok(())
}
