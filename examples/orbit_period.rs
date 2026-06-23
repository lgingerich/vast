#![allow(missing_docs)]

use vast::{
    Distance, Duration, EulerIntegrator, Frame, GravitationalParameter, LeapfrogIntegrator, Orbit,
    OrbitalDiagnostics, Position, RungeKutta4Integrator, SimBuilder, Speed, TimeStep,
    TwoBodyGravity, VelocityVerletIntegrator,
};

const ALTITUDE_KM: f64 = 400.0;
const STEP_SECONDS: [f64; 6] = [60.0, 30.0, 10.0, 1.0, 0.1, 0.01];

struct Summary {
    method: &'static str,
    dt_s: f64,
    steps: usize,
    elapsed_s: f64,
    overshoot_s: f64,
    position_drift_km: f64,
    speed_error_mps: f64,
    energy_error: f64,
    relative_energy_error: f64,
    h_error: f64,
    relative_h_error: f64,
    eccentricity: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- configure simulation ---
    let frame = Frame::earth_inertial();
    let orbit_spec = Orbit::circular_equatorial(Distance::new(ALTITUDE_KM * 1000.0));
    let speed = orbit_spec.speed(&frame)?;
    let period = orbit_spec.period(&frame)?;
    let initial_position = orbit_spec.to_cartesian_state(&frame)?.position;
    let mu = frame.mu()?;

    // --- print header ---
    println!(
        "Circular Earth orbit at {:.0} km altitude",
        orbit_spec.semi_major_axis(&frame)?.meters() / 1000.0 - vast::R_EARTH / 1000.0,
    );
    println!(
        "radius: {:.3} km | speed: {:.3} km/s | period: {:.3} min",
        orbit_spec.semi_major_axis(&frame)?.meters() / 1000.0,
        speed.meters_per_second() / 1000.0,
        period.seconds() / 60.0,
    );
    println!();
    println!(
        "{:>8} {:>8} {:>8} {:>12} {:>13} {:>20} {:>18} {:>20} {:>12} {:>18} {:>12} {:>12}",
        "method",
        "dt(s)",
        "steps",
        "elapsed(s)",
        "overshoot(s)",
        "position drift(km)",
        "speed error(m/s)",
        "energy abs(m^2/s^2)",
        "energy rel",
        "h abs(m^2/s)",
        "h rel",
        "eccentricity",
    );

    // --- sweep timesteps ---
    for step_s in STEP_SECONDS {
        let dt = TimeStep::new(step_s);

        let euler = run(
            "Euler",
            frame,
            &orbit_spec,
            EulerIntegrator,
            dt,
            period,
            &initial_position,
            speed,
            mu,
        )?;
        let rk4 = run(
            "RK4",
            frame,
            &orbit_spec,
            RungeKutta4Integrator,
            dt,
            period,
            &initial_position,
            speed,
            mu,
        )?;
        let verlet = run(
            "Verlet",
            frame,
            &orbit_spec,
            VelocityVerletIntegrator,
            dt,
            period,
            &initial_position,
            speed,
            mu,
        )?;
        let leapfrog = run(
            "Leapfrog",
            frame,
            &orbit_spec,
            LeapfrogIntegrator,
            dt,
            period,
            &initial_position,
            speed,
            mu,
        )?;

        print_summary(&euler);
        print_summary(&rk4);
        print_summary(&verlet);
        print_summary(&leapfrog);
    }

    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn run<I: vast::Integrator>(
    method: &'static str,
    frame: Frame,
    orbit_spec: &Orbit,
    integrator: I,
    dt: TimeStep,
    period: Duration,
    initial_position: &Position,
    circular_speed: Speed,
    mu: GravitationalParameter,
) -> Result<Summary, Box<dyn std::error::Error>> {
    // --- run simulation ---
    let mut sim = SimBuilder::new(frame)
        .orbit(*orbit_spec)?
        .dynamics(TwoBodyGravity::from_frame(&frame)?)
        .integrator(integrator)
        .build()?;
    let initial_state = sim.state()?;
    let initial_diag = OrbitalDiagnostics::from_state(&initial_state, mu)?;
    sim.advance(period, dt)?;

    // --- inspect results ---
    let final_state = sim.state()?;
    let final_diag = OrbitalDiagnostics::from_state(&final_state, mu)?;

    let pos_drift = (final_state.position.vector() - initial_position.vector())
        .norm()
        .max(f64::EPSILON);
    let speed_error =
        (final_diag.speed().meters_per_second() - circular_speed.meters_per_second()).abs();
    let energy_error = (final_diag
        .specific_energy()
        .square_meters_per_square_second()
        - initial_diag
            .specific_energy()
            .square_meters_per_square_second())
    .abs();
    let relative_energy_error = energy_error
        / initial_diag
            .specific_energy()
            .square_meters_per_square_second()
            .abs();
    let h_error = (final_diag
        .specific_angular_momentum()
        .square_meters_per_second()
        - initial_diag
            .specific_angular_momentum()
            .square_meters_per_second())
    .abs();
    let relative_h_error = h_error
        / initial_diag
            .specific_angular_momentum()
            .square_meters_per_second();

    let elapsed = Duration::new(sim.epoch().seconds());
    Ok(Summary {
        method,
        dt_s: dt.seconds(),
        steps: sim.steps(),
        elapsed_s: elapsed.seconds(),
        overshoot_s: elapsed.seconds() - period.seconds(),
        position_drift_km: pos_drift / 1000.0,
        speed_error_mps: speed_error,
        energy_error,
        relative_energy_error,
        h_error,
        relative_h_error,
        eccentricity: final_diag.eccentricity().scalar(),
    })
}

fn print_summary(s: &Summary) {
    println!(
        "{:>8} {:>8.2} {:>8} {:>12.1} {:>13.3} {:>20.6} {:>18.9} {:>20.6} {:>12.3e} {:>18.6} {:>12.3e} {:>12.9}",
        s.method,
        s.dt_s,
        s.steps,
        s.elapsed_s,
        s.overshoot_s,
        s.position_drift_km,
        s.speed_error_mps,
        s.energy_error,
        s.relative_energy_error,
        s.h_error,
        s.relative_h_error,
        s.eccentricity,
    );
}
