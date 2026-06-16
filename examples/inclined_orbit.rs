#![allow(missing_docs)]

use vast::{
    Angle, CentralBody, Column, Distance, Duration, Orbit, SimBuilder, TimeStep, TwoBodyGravity,
    VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = 600.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 10.0;
const PERIODS: f64 = 3.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let body = CentralBody::earth()?;
    let spec = Orbit::circular_inclined(
        Distance::new(ALTITUDE_KM * 1000.0)?,
        Angle::from_degrees(INCLINATION_DEG)?,
        Angle::from_degrees(RAAN_DEG)?,
    );
    let period = spec.to_elliptic(&body)?.period()?;
    let num_periods = Duration::new(period.seconds() * PERIODS).unwrap();

    // --- run simulation ---
    let result = SimBuilder::new(body)
        .orbit(spec)?
        .dynamics(TwoBodyGravity::from_body(&body))
        .integrator(VelocityVerletIntegrator)
        .timestep(TimeStep::new(STEP_SECONDS)?)
        .duration(num_periods)?;

    // --- inspect results ---
    let s = &result.series;
    let radius_drift = (s.column(Column::Radius)[1] - s.column(Column::Radius)[0]).abs();
    let energy_error =
        (s.column(Column::SpecificEnergy)[1] - s.column(Column::SpecificEnergy)[0]).abs();
    let h_error = (s.column(Column::SpecificAngularMomentum)[1]
        - s.column(Column::SpecificAngularMomentum)[0])
        .abs();
    let h = result
        .final_state
        .position
        .vector()
        .cross(&result.final_state.velocity.vector());
    let h_hat = h.normalize();

    // --- print report ---
    println!(
        "Inclined circular orbit: altitude {:.0} km | inclination {:.1}° | RAAN {:.1}°",
        ALTITUDE_KM, INCLINATION_DEG, RAAN_DEG,
    );
    println!(
        "propagated: {:.1} periods | elapsed: {:.1} s | steps: {}",
        PERIODS,
        result.elapsed.seconds(),
        result.steps,
    );
    println!(
        "radius drift: {:.6} km | energy error: {:.6} | h error: {:.6}",
        radius_drift.max(f64::EPSILON) / 1000.0,
        energy_error,
        h_error,
    );
    println!("eccentricity: {:.9}", s.column(Column::Eccentricity)[1],);
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        h_hat.x, h_hat.y, h_hat.z,
    );
    println!("{}", result.elements);

    Ok(())
}
