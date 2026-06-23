#![allow(missing_docs)]

use vast::{
    AreaPerMass, AtmosphericDrag, CompositeDynamics, Distance, Duration, ExponentialAtmosphere,
    Frame, Orbit, OrbitalDiagnostics, R_EARTH, RungeKutta4Integrator, SimBuilder, TimeStep,
    TwoBodyGravity,
};

const ALTITUDE_KM: f64 = 250.0;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let frame = Frame::earth_inertial();
    let atmosphere = ExponentialAtmosphere::from_frame(&frame)?;
    let dynamics = CompositeDynamics::new(
        TwoBodyGravity::from_frame(&frame)?,
        AtmosphericDrag::new(atmosphere, AreaPerMass::new(0.02)),
    );
    let total_duration = Duration::new(PROPAGATION_HOURS * 3_600.0);
    let dt = TimeStep::new(STEP_SECONDS);
    let mu = frame.mu()?;

    // --- run simulation ---
    let mut sim = SimBuilder::new(frame)
        .orbit(Orbit::circular_equatorial(Distance::new(
            ALTITUDE_KM * 1000.0,
        )))?
        .dynamics(dynamics)
        .integrator(RungeKutta4Integrator)
        .build()?;
    let initial_state = sim.state()?;
    let initial_diag = OrbitalDiagnostics::from_state(&initial_state, mu)?;
    sim.advance(total_duration, dt)?;

    // --- inspect results ---
    let final_state = sim.state()?;
    let final_diag = OrbitalDiagnostics::from_state(&final_state, mu)?;

    let initial_alt_km = (initial_diag.radius().meters() - R_EARTH) / 1000.0;
    let final_alt_km = (final_diag.radius().meters() - R_EARTH) / 1000.0;
    let energy_delta = final_diag
        .specific_energy()
        .square_meters_per_square_second()
        - initial_diag
            .specific_energy()
            .square_meters_per_square_second();
    let speed_delta =
        final_diag.speed().meters_per_second() - initial_diag.speed().meters_per_second();

    // --- print report ---
    println!(
        "Drag decay demo: initial altitude {:.0} km | duration {:.1} h",
        initial_alt_km,
        sim.epoch().seconds() / 3_600.0,
    );
    println!(
        "final altitude: {:.3} km | altitude change: {:+.3} km",
        final_alt_km,
        final_alt_km - initial_alt_km,
    );
    println!(
        "specific energy change: {:+.6} m^2/s^2 | speed change: {:+.6} m/s",
        energy_delta, speed_delta,
    );
    println!("eccentricity: {:.9}", final_diag.eccentricity().scalar());

    Ok(())
}
