#![allow(missing_docs)]

use vast::{
    Angle, Distance, Duration, Frame, GravitationalParameter, Orbit, OrbitalElementClassification,
    OrbitalElements, SimBuilder, TimeStep, TwoBodyGravity, VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = 600.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 10.0;
const PERIODS: f64 = 3.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
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

    let mut sim = SimBuilder::new(frame)
        .orbit(spec)?
        .dynamics(TwoBodyGravity::from_frame(&frame)?)
        .integrator(VelocityVerletIntegrator)
        .build()?;
    sim.advance(num_periods, dt)?;

    let final_state = sim.state()?;
    let h = final_state
        .position
        .vector()
        .cross(&final_state.velocity.vector());
    let h_hat = h.normalize();
    let radius_drift =
        (final_state.position.vector().norm() - spec.semi_major_axis(&frame)?.meters()).abs();

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
        "radius drift: {:.6} km",
        radius_drift.max(f64::EPSILON) / 1000.0
    );
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        h_hat.x, h_hat.y, h_hat.z,
    );
    let elements =
        OrbitalElements::from_state(&final_state, mu, OrbitalElementClassification::default())?;
    println!("{:?}", elements);

    Ok(())
}
