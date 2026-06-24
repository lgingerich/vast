#![allow(missing_docs)]

use vast::{
    AreaPerMass, AtmosphericDrag, CompositeDynamics, Distance, Duration, ExponentialAtmosphere,
    Frame, Orbit, R_EARTH, RungeKutta4Integrator, SimBuilder, TimeStep, TwoBodyGravity,
};

const ALTITUDE_KM: f64 = 250.0;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let frame = Frame::earth_inertial();
    let atmosphere = ExponentialAtmosphere::from_frame(&frame)?;
    let dynamics = CompositeDynamics::new(
        TwoBodyGravity::from_frame(&frame)?,
        AtmosphericDrag::new(atmosphere, AreaPerMass::new(0.02)),
    );
    let total_duration = Duration::new(PROPAGATION_HOURS * 3_600.0);
    let dt = TimeStep::new(STEP_SECONDS);

    let mut sim = SimBuilder::new(frame)
        .orbit(Orbit::circular_equatorial(Distance::new(
            ALTITUDE_KM * 1000.0,
        )))?
        .dynamics(dynamics)
        .integrator(RungeKutta4Integrator)
        .build()?;
    let initial_state = sim.state()?;
    sim.advance(total_duration, dt)?;

    let final_state = sim.state()?;
    let initial_alt_km = (initial_state.position.vector().norm() - R_EARTH) / 1000.0;
    let final_alt_km = (final_state.position.vector().norm() - R_EARTH) / 1000.0;
    let speed_delta = final_state.velocity.vector().norm() - initial_state.velocity.vector().norm();

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
    println!("speed change: {:+.6} m/s", speed_delta);

    Ok(())
}
