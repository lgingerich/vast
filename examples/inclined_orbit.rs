#![allow(missing_docs)]

use vast::{
    Angle, Column, Distance, Duration, EllipticOrbit, Orbit, SimBuilder, TimeStep, TwoBodyGravity,
    VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = (vast::R_EARTH + 600_000.0) / 1000.0;
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

    let s = &result.series;
    let radius_drift = (s.column(Column::RadiusKm)[1] - s.column(Column::RadiusKm)[0]).abs();
    let energy_error =
        (s.column(Column::SpecificEnergyM2s2)[1] - s.column(Column::SpecificEnergyM2s2)[0]).abs();
    let h_error =
        (s.column(Column::SpecificAngularMomentumM2s)[1]
            - s.column(Column::SpecificAngularMomentumM2s)[0])
        .abs();
    let h = result.final_state.position.vector()
        .cross(&result.final_state.velocity.vector());
    let h_hat = h.normalize();

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
        radius_drift.max(f64::EPSILON),
        energy_error,
        h_error,
    );
    println!(
        "eccentricity: {:.9}",
        s.column(Column::Eccentricity)[1],
    );
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        h_hat.x, h_hat.y, h_hat.z,
    );
    println!("{}", result.elements);

    Ok(())
}
