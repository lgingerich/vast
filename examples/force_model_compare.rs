#![allow(missing_docs)]

use vast::{
    Angle, AreaPerMass, AtmosphericDrag, CompositeDynamics, Distance, Duration,
    ExponentialAtmosphere, Frame, Orbit, OrbitalElementClassification, OrbitalElements, R_EARTH,
    RungeKutta4Integrator, SimBuilder, TimeStep, TwoBodyGravity,
};

const ALTITUDE_KM: f64 = 250.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const AREA_PER_MASS: f64 = 0.02;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

struct Summary {
    model: &'static str,
    final_altitude_km: f64,
    altitude_delta_km: f64,
    raan_drift_deg: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let frame = Frame::earth_inertial();
    let orbit = Orbit::circular_inclined(
        Distance::new(ALTITUDE_KM * 1000.0),
        Angle::from_degrees(INCLINATION_DEG),
        Angle::from_degrees(RAAN_DEG),
    );

    let two_body = run(
        "two-body",
        frame,
        orbit,
        TwoBodyGravity::from_frame(&frame)?,
    )?;
    let j2 = run("J2", frame, orbit, vast::J2Gravity::from_frame(&frame)?)?;
    let drag = run(
        "drag",
        frame,
        orbit,
        CompositeDynamics::new(TwoBodyGravity::from_frame(&frame)?, drag_model(&frame)?),
    )?;
    let j2_drag = run(
        "J2+drag",
        frame,
        orbit,
        CompositeDynamics::new(vast::J2Gravity::from_frame(&frame)?, drag_model(&frame)?),
    )?;

    println!(
        "Force model comparison: altitude {:.0} km | inclination {:.1} deg | duration {:.1} h",
        ALTITUDE_KM, INCLINATION_DEG, PROPAGATION_HOURS,
    );
    println!(
        "integrator: RK4 | step: {:.1} s | area/mass for drag: {:.3} m^2/kg",
        STEP_SECONDS, AREA_PER_MASS,
    );
    println!();
    println!(
        "{:>10} {:>16} {:>18} {:>16}",
        "model", "final alt(km)", "alt delta(km)", "RAAN drift(deg)",
    );

    for summary in [two_body, j2, drag, j2_drag] {
        println!(
            "{:>10} {:>16.3} {:>18.3} {:>16.6}",
            summary.model,
            summary.final_altitude_km,
            summary.altitude_delta_km,
            summary.raan_drift_deg,
        );
    }

    Ok(())
}

fn drag_model(frame: &Frame) -> Result<AtmosphericDrag, Box<dyn std::error::Error>> {
    Ok(AtmosphericDrag::new(
        ExponentialAtmosphere::from_frame(frame)?,
        AreaPerMass::new(AREA_PER_MASS),
    ))
}

fn run<D: vast::Dynamics>(
    model: &'static str,
    frame: Frame,
    orbit: Orbit,
    dynamics: D,
) -> Result<Summary, Box<dyn std::error::Error>> {
    let mu = frame.mu()?;
    let dt = TimeStep::new(STEP_SECONDS);

    let mut sim = SimBuilder::new(frame)
        .orbit(orbit)?
        .dynamics(dynamics)
        .integrator(RungeKutta4Integrator)
        .build()?;
    let initial_state = sim.state()?;
    sim.advance(Duration::new(PROPAGATION_HOURS * 3_600.0), dt)?;

    let final_state = sim.state()?;
    let elements =
        OrbitalElements::from_state(&final_state, mu, OrbitalElementClassification::default())?;
    let final_raan = elements
        .right_ascension_of_ascending_node()
        .map(|raan| raan.degrees())
        .ok_or("final RAAN is undefined")?;

    let initial_altitude_km = (initial_state.position.vector().norm() - R_EARTH) / 1000.0;
    let final_altitude_km = (final_state.position.vector().norm() - R_EARTH) / 1000.0;

    Ok(Summary {
        model,
        final_altitude_km,
        altitude_delta_km: final_altitude_km - initial_altitude_km,
        raan_drift_deg: angle_delta_degrees(final_raan, RAAN_DEG),
    })
}

fn angle_delta_degrees(final_degrees: f64, initial_degrees: f64) -> f64 {
    let delta = (final_degrees - initial_degrees).rem_euclid(360.0);
    if delta > 180.0 { delta - 360.0 } else { delta }
}
