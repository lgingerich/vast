#![allow(missing_docs)]

use space_sim::{
    Angle, CartesianState, CircularOrbit, Distance, Duration, Epoch, EulerIntegrator,
    GravitationalParameter, Integrator, MU_EARTH, OrbitDirection, OrbitPlane, R_EARTH,
    RungeKutta4Integrator, Speed, TimeStep, TwoBodyGravity,
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
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
        .ok_or("invalid Earth gravitational parameter")?;
    let orbit = CircularOrbit {
        radius: Distance::from_meters(R_EARTH + ALTITUDE_M).ok_or("invalid orbital radius")?,
        mu,
        plane: OrbitPlane::Xy,
        phase: Angle::from_radians(0.0).ok_or("invalid initial phase")?,
        direction: OrbitDirection::Prograde,
    };

    let speed = orbit.speed().ok_or("invalid circular orbit speed")?;
    let period = orbit.period().ok_or("invalid circular orbit period")?;
    let initial_state = orbit
        .to_cartesian_state()
        .ok_or("invalid initial Cartesian state")?;
    let gravity = TwoBodyGravity { mu };
    let euler = EulerIntegrator;
    let rk4 = RungeKutta4Integrator;

    println!(
        "Circular Earth orbit at {:.0} km altitude",
        ALTITUDE_M / 1000.0
    );
    println!(
        "radius: {:.3} km | speed: {:.3} km/s | period: {:.3} min",
        orbit.radius.meters() / 1000.0,
        speed.meters_per_second() / 1000.0,
        period.seconds() / 60.0,
    );
    println!();
    println!(
        "{:>8} {:>8} {:>8} {:>12} {:>13} {:>20} {:>18}",
        "method",
        "dt(s)",
        "steps",
        "elapsed(s)",
        "overshoot(s)",
        "position drift(km)",
        "speed error(m/s)"
    );

    for step_seconds in STEP_SECONDS {
        let dt = TimeStep::from_seconds(step_seconds).ok_or("invalid time step")?;
        let euler_summary =
            propagate_one_period(&euler, &gravity, &initial_state, speed, period, dt)?;
        let rk4_summary = propagate_one_period(&rk4, &gravity, &initial_state, speed, period, dt)?;

        print_summary("Euler", &euler_summary);
        print_summary("RK4", &rk4_summary);
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
    let mut epoch = Epoch::from_seconds(0.0).ok_or("invalid initial epoch")?;
    let mut state = CartesianState::new(initial_state.position, initial_state.velocity);
    let mut steps = 0;

    while epoch.seconds() < period.seconds() {
        let remaining_seconds = period.seconds() - epoch.seconds();
        let step_seconds = dt.seconds().min(remaining_seconds);
        let step = TimeStep::from_seconds(step_seconds).ok_or("invalid final time step")?;

        state = integrator.step(gravity, &state, epoch, step)?;
        epoch = epoch.advance_by(step).ok_or("invalid next epoch")?;
        steps += 1;
    }

    let position_drift = Distance::from_meters(
        (state.position.vector() - initial_position.vector())
            .norm()
            .max(f64::EPSILON),
    )
    .ok_or("invalid position drift")?;
    let speed_error = Speed::from_meters_per_second(
        (state.velocity.vector().norm() - circular_speed.meters_per_second()).abs(),
    )
    .ok_or("invalid speed error")?;

    Ok(PropagationSummary {
        dt,
        steps,
        elapsed: Duration::from_seconds(epoch.seconds()).ok_or("invalid elapsed duration")?,
        overshoot: Duration::from_seconds(epoch.seconds() - period.seconds())
            .ok_or("invalid period overshoot")?,
        position_drift,
        speed_error,
    })
}

fn print_summary(method: &str, summary: &PropagationSummary) {
    println!(
        "{method:>8} {:>8.2} {:>8} {:>12.1} {:>13.3} {:>20.6} {:>18.9}",
        summary.dt.seconds(),
        summary.steps,
        summary.elapsed.seconds(),
        summary.overshoot.seconds(),
        summary.position_drift.meters() / 1000.0,
        summary.speed_error.meters_per_second(),
    );
}
