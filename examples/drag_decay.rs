#![allow(missing_docs)]

use vast::{
    AngularRate, AreaPerMass, AtmosphericDrag, Column, CompositeDynamics, Distance, Duration,
    ExponentialAtmosphere, MassDensity, Orbit, R_EARTH, RungeKutta4Integrator, SimBuilder,
    TimeStep, TwoBodyGravity,
};

const ALTITUDE_KM: f64 = 250.0;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let atmosphere = ExponentialAtmosphere::new(
        Distance::new(R_EARTH)?,
        Distance::new(250.0 * 1000.0)?,
        MassDensity::new(1.0e-10)?,
        Distance::new(50.0 * 1000.0)?,
        AngularRate::new(7.292_115_9e-5)?,
    );
    let dynamics = CompositeDynamics::new(
        TwoBodyGravity::earth(),
        AtmosphericDrag::new(atmosphere, AreaPerMass::new(0.02)?),
    );
    let total_duration = Duration::new(PROPAGATION_HOURS * 3_600.0).unwrap();

    // --- run simulation ---
    let result = SimBuilder::new()
        .orbit(Orbit::CircularEquatorial {
            altitude: Distance::new(ALTITUDE_KM * 1000.0)?,
        })?
        .dynamics(dynamics)
        .integrator(RungeKutta4Integrator)
        .timestep(TimeStep::new(STEP_SECONDS)?)
        .duration(total_duration)?;

    // --- inspect results ---
    let s = &result.series;
    let initial_alt_km = (s.column(Column::Radius)[0] - R_EARTH) / 1000.0;
    let final_alt_km = (s.column(Column::Radius)[1] - R_EARTH) / 1000.0;
    let energy_delta = s.column(Column::SpecificEnergy)[1] - s.column(Column::SpecificEnergy)[0];
    let speed_delta = s.column(Column::Speed)[1] - s.column(Column::Speed)[0];

    // --- print report ---
    println!(
        "Drag decay demo: initial altitude {:.0} km | duration {:.1} h",
        initial_alt_km,
        result.elapsed.seconds() / 3_600.0,
    );
    println!(
        "final altitude: {:.3} km | altitude change: {:+.3} km",
        final_alt_km,
        final_alt_km - initial_alt_km,
    );
    println!(
        "specific energy change: {:+.6} | speed change: {:+.6} m/s",
        energy_delta, speed_delta,
    );
    println!("eccentricity: {:.9}", s.column(Column::Eccentricity)[1]);

    Ok(())
}
