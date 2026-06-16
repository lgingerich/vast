#![allow(missing_docs)]

use vast::{
    AngularRate, AreaPerMass, AtmosphericDrag, CompositeDynamics, Distance, Duration,
    ExponentialAtmosphere, MassDensity, Orbit, R_EARTH, RungeKutta4Integrator, SimBuilder,
    TimeStep, TwoBodyGravity,
};

const ALTITUDE_KM: f64 = (R_EARTH + 250_000.0) / 1000.0;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let atmosphere = ExponentialAtmosphere::new(
        Distance::from_kilometers(R_EARTH / 1000.0)?,
        Distance::from_kilometers(250.0)?,
        MassDensity::from_kilograms_per_cubic_meter(1.0e-10)?,
        Distance::from_kilometers(50.0)?,
        AngularRate::from_radians_per_second(7.292_115_9e-5)?,
    );
    let dynamics = CompositeDynamics::new(
        TwoBodyGravity::earth(),
        AtmosphericDrag::new(
            atmosphere,
            AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        ),
    );
    let total_duration = Duration::from_seconds(PROPAGATION_HOURS * 3_600.0).unwrap();

    let result = SimBuilder::new()
        .orbit(Orbit::CircularEquatorial {
            altitude: Distance::from_kilometers(ALTITUDE_KM)?,
        })?
        .dynamics(dynamics)
        .integrator(RungeKutta4Integrator)
        .timestep(TimeStep::from_seconds(STEP_SECONDS)?)
        .duration(total_duration)?;

    let s = &result.series;
    let initial_alt_km = s.radius_km()[0] - R_EARTH / 1000.0;
    let final_alt_km = s.radius_km()[1] - R_EARTH / 1000.0;
    let energy_delta = s.specific_energy_m2s2()[1] - s.specific_energy_m2s2()[0];
    let speed_delta = s.speed_mps()[1] - s.speed_mps()[0];

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
    println!("eccentricity: {:.9}", s.eccentricity()[1]);

    Ok(())
}
