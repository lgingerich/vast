#![allow(missing_docs)]

use vast::{
    Angle, CartesianState, CartesianStateBuffer, Distance, Duration, Eccentricity, EllipticOrbit,
    Epoch, EulerIntegrator, GravitationalParameter, Integrator, LeapfrogIntegrator, MU_EARTH,
    OrbitDirection, OrbitalDiagnostics, R_EARTH, RungeKutta4Integrator, SpecificAngularMomentum,
    SpecificEnergy, Speed, TimeStep, TwoBodyGravity, VelocityVerletIntegrator,
};

const ALTITUDE_M: f64 = 400_000.0;
const STEP_SECONDS: [f64; 6] = [60.0, 30.0, 10.0, 1.0, 0.1, 0.01];

struct PropagationSummary {
    dt: TimeStep,
    steps: usize,
    elapsed: Duration,
    overshoot: Duration,
    position_drift: Distance,
    speed_error: Speed,
    specific_energy_error: SpecificEnergy,
    specific_angular_momentum_error: SpecificAngularMomentum,
    eccentricity: f64,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
    let orbit = EllipticOrbit::new(
        Distance::from_meters(R_EARTH + ALTITUDE_M)?,
        Eccentricity::from_scalar(0.0)?,
        mu,
        Angle::ZERO,
        Angle::ZERO,
        Angle::ZERO,
        Angle::from_radians(0.0)?,
        OrbitDirection::Prograde,
    )?;

    let speed = orbit.speed()?;
    let period = orbit.period()?;
    let initial_state = orbit.to_cartesian_state()?;
    let gravity = TwoBodyGravity::new(mu);
    let euler = EulerIntegrator;
    let rk4 = RungeKutta4Integrator;
    let verlet = VelocityVerletIntegrator;
    let leapfrog = LeapfrogIntegrator;

    println!(
        "Circular Earth orbit at {:.0} km altitude",
        ALTITUDE_M / 1000.0
    );
    println!(
        "radius: {:.3} km | speed: {:.3} km/s | period: {:.3} min",
        orbit.semi_major_axis().meters() / 1000.0,
        speed.meters_per_second() / 1000.0,
        period.seconds() / 60.0,
    );
    println!();
    println!(
        "{:>8} {:>8} {:>8} {:>12} {:>13} {:>20} {:>18} {:>18} {:>16} {:>12}",
        "method",
        "dt(s)",
        "steps",
        "elapsed(s)",
        "overshoot(s)",
        "position drift(km)",
        "speed error(m/s)",
        "energy error",
        "h error",
        "eccentricity",
    );

    for step_seconds in STEP_SECONDS {
        let dt = TimeStep::from_seconds(step_seconds)?;
        let euler_summary =
            propagate_one_period(&euler, &gravity, &initial_state, speed, period, dt)?;
        let rk4_summary = propagate_one_period(&rk4, &gravity, &initial_state, speed, period, dt)?;
        let verlet_summary =
            propagate_one_period(&verlet, &gravity, &initial_state, speed, period, dt)?;
        let leapfrog_summary =
            propagate_one_period(&leapfrog, &gravity, &initial_state, speed, period, dt)?;

        print_summary("Euler", &euler_summary);
        print_summary("RK4", &rk4_summary);
        print_summary("Verlet", &verlet_summary);
        print_summary("Leapfrog", &leapfrog_summary);
    }

    Ok(())
}

fn propagate_one_period<I: Integrator>(
    integrator: &I,
    gravity: &TwoBodyGravity,
    initial_state: &CartesianState,
    circular_speed: Speed,
    period: Duration,
    dt: TimeStep,
) -> Result<PropagationSummary, Box<dyn std::error::Error>> {
    let initial_position = initial_state.position;
    let initial_diagnostics = OrbitalDiagnostics::from_state(initial_state, gravity.mu())?;
    let mut epoch = Epoch::from_seconds(0.0)?;
    let mut states = CartesianStateBuffer::<1>::new();
    states.write_from_state(0, initial_state)?;
    let mut steps = 0;

    while epoch.seconds() < period.seconds() {
        let remaining_seconds = period.seconds() - epoch.seconds();
        let step_seconds = dt.seconds().min(remaining_seconds);
        let step = TimeStep::from_seconds(step_seconds)?;

        integrator.step(gravity, &mut states.view_mut(), 0, epoch, step)?;
        epoch = epoch.advance_by(step)?;
        steps += 1;
    }

    let state = states.read_state(0)?;
    let position_drift = Distance::from_meters(
        (state.position.vector() - initial_position.vector())
            .norm()
            .max(f64::EPSILON),
    )?;
    let speed_error = Speed::from_meters_per_second(
        (state.velocity.vector().norm() - circular_speed.meters_per_second()).abs(),
    )?;
    let final_diagnostics = OrbitalDiagnostics::from_state(&state, gravity.mu())?;
    let specific_energy_error = SpecificEnergy::from_square_meters_per_second_squared(
        (final_diagnostics
            .specific_energy()
            .square_meters_per_square_second()
            - initial_diagnostics
                .specific_energy()
                .square_meters_per_square_second())
        .abs(),
    )?;
    let specific_angular_momentum_error = SpecificAngularMomentum::from_square_meters_per_second(
        (final_diagnostics
            .specific_angular_momentum()
            .square_meters_per_second()
            - initial_diagnostics
                .specific_angular_momentum()
                .square_meters_per_second())
        .abs(),
    )?;

    Ok(PropagationSummary {
        dt,
        steps,
        elapsed: Duration::from_seconds(epoch.seconds())?,
        overshoot: Duration::from_seconds(epoch.seconds() - period.seconds())?,
        position_drift,
        speed_error,
        specific_energy_error,
        specific_angular_momentum_error,
        eccentricity: final_diagnostics.eccentricity().scalar(),
    })
}

fn print_summary(method: &str, summary: &PropagationSummary) {
    println!(
        "{method:>8} {:>8.2} {:>8} {:>12.1} {:>13.3} {:>20.6} {:>18.9} {:>18.6} {:>16.6} {:>12.9}",
        summary.dt.seconds(),
        summary.steps,
        summary.elapsed.seconds(),
        summary.overshoot.seconds(),
        summary.position_drift.meters() / 1000.0,
        summary.speed_error.meters_per_second(),
        summary
            .specific_energy_error
            .square_meters_per_square_second(),
        summary
            .specific_angular_momentum_error
            .square_meters_per_second(),
        summary.eccentricity,
    );
}
