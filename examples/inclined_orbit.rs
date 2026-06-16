#![allow(missing_docs)]

use vast::{
    Angle, Distance, Duration, EllipticOrbit, Orbit, R_EARTH, SimBuilder, TimeStep,
    TwoBodyGravity, VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = (R_EARTH + 600_000.0) / 1000.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 10.0;
const PERIODS: f64 = 3.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let spec = Orbit::CircularInclined {
        altitude: Distance::from_kilometers(ALTITUDE_KM)?,
        inclination: Angle::from_degrees(INCLINATION_DEG)?,
        raan: Angle::from_degrees(RAAN_DEG)?,
    };
    let period = EllipticOrbit::try_from(spec)?.period()?;
    let num_periods = Duration::from_seconds(period.seconds() * PERIODS).unwrap();

    let result = SimBuilder::new()
        .orbit(spec)?
        .dynamics(TwoBodyGravity::earth())
        .integrator(VelocityVerletIntegrator)
        .timestep(TimeStep::from_seconds(STEP_SECONDS)?)
        .duration(num_periods)?;

    let radius_drift = result
        .diagnostics
        .radius()
        .km_diff(result.initial_diagnostics.radius())
        .abs();
    let energy_error = result
        .initial_diagnostics
        .specific_energy()
        .abs_diff(result.diagnostics.specific_energy());
    let h_error = result
        .initial_diagnostics
        .specific_angular_momentum()
        .abs_diff(result.diagnostics.specific_angular_momentum());
    let h_hat = result.diagnostics.specific_angular_momentum_vector().normalize();

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
        radius_drift,
        energy_error.square_meters_per_square_second(),
        h_error.square_meters_per_second(),
    );
    println!(
        "eccentricity: {:.9}",
        result.diagnostics.eccentricity().scalar(),
    );
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        h_hat.x, h_hat.y, h_hat.z,
    );
    println!("{}", result.elements);

    Ok(())
}
