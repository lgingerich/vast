#![allow(missing_docs)]

use space_sim::{
    Angle, CartesianStateBuffer, CircularOrbit, CircularOrbitOrientation, Distance, Duration,
    Epoch, GravitationalParameter, Integrator, MU_EARTH, OrbitDirection, OrbitalDiagnostics,
    R_EARTH, SpecificAngularMomentum, SpecificEnergy, TimeStep, TwoBodyGravity,
    VelocityVerletIntegrator,
};

const ALTITUDE_M: f64 = 600_000.0;
const INCLINATION_DEG: f64 = 51.6;
const RAAN_DEG: f64 = 32.0;
const STEP_SECONDS: f64 = 10.0;
const PERIODS: usize = 3;

struct PropagationSummary {
    elapsed: Duration,
    steps: usize,
    radius_drift: Distance,
    specific_energy_error: SpecificEnergy,
    specific_angular_momentum_error: SpecificAngularMomentum,
    eccentricity: f64,
    angular_momentum_direction: (f64, f64, f64),
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
        .ok_or("invalid Earth gravitational parameter")?;
    let orbit = CircularOrbit {
        radius: Distance::from_meters(R_EARTH + ALTITUDE_M).ok_or("invalid orbital radius")?,
        mu,
        orientation: CircularOrbitOrientation::new(
            Angle::from_radians(INCLINATION_DEG.to_radians()).ok_or("invalid inclination")?,
            Angle::from_radians(RAAN_DEG.to_radians()).ok_or("invalid RAAN")?,
        ),
        phase: Angle::from_radians(0.0).ok_or("invalid phase")?,
        direction: OrbitDirection::Prograde,
    };
    let initial_state = orbit
        .to_cartesian_state()
        .ok_or("invalid initial Cartesian state")?;
    let initial_diagnostics =
        OrbitalDiagnostics::from_state(&initial_state, mu).ok_or("invalid initial diagnostics")?;
    let gravity = TwoBodyGravity { mu };
    let integrator = VelocityVerletIntegrator;
    let dt = TimeStep::from_seconds(STEP_SECONDS).ok_or("invalid time step")?;
    let period = orbit.period().ok_or("invalid orbital period")?;

    let mut states = CartesianStateBuffer::<1>::new();
    states.write_from_state(0, &initial_state)?;

    let summary = propagate(
        &integrator,
        &gravity,
        &mut states,
        initial_diagnostics,
        period,
        dt,
        PERIODS,
    )?;

    println!(
        "Inclined circular orbit: altitude {:.0} km | inclination {:.1} deg | RAAN {:.1} deg",
        ALTITUDE_M / 1000.0,
        INCLINATION_DEG,
        RAAN_DEG,
    );
    println!(
        "propagated: {:.1} periods | elapsed: {:.1} s | steps: {}",
        PERIODS,
        summary.elapsed.seconds(),
        summary.steps,
    );
    println!(
        "radius drift: {:.6} km | energy error: {:.6} | h error: {:.6}",
        summary.radius_drift.meters() / 1000.0,
        summary
            .specific_energy_error
            .square_meters_per_square_second(),
        summary
            .specific_angular_momentum_error
            .square_meters_per_second(),
    );
    println!("eccentricity: {:.9}", summary.eccentricity);
    println!(
        "h direction: [{:.6}, {:.6}, {:.6}]",
        summary.angular_momentum_direction.0,
        summary.angular_momentum_direction.1,
        summary.angular_momentum_direction.2,
    );

    Ok(())
}

fn propagate<I: Integrator>(
    integrator: &I,
    gravity: &TwoBodyGravity,
    states: &mut CartesianStateBuffer<1>,
    initial_diagnostics: OrbitalDiagnostics,
    period: Duration,
    dt: TimeStep,
    periods: usize,
) -> Result<PropagationSummary, Box<dyn std::error::Error>> {
    let target_seconds = period.seconds() * periods as f64;
    let mut epoch = Epoch::from_seconds(0.0).ok_or("invalid initial epoch")?;
    let mut steps = 0;

    while epoch.seconds() < target_seconds {
        let remaining_seconds = target_seconds - epoch.seconds();
        let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))
            .ok_or("invalid integration step")?;
        integrator.step(gravity, &mut states.view_mut(), 0, epoch, step)?;
        epoch = epoch.advance_by(step).ok_or("invalid next epoch")?;
        steps += 1;
    }

    let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, gravity.mu)
        .ok_or("invalid final diagnostics")?;
    let radius_drift = Distance::from_meters(
        (diagnostics.radius.meters() - initial_diagnostics.radius.meters())
            .abs()
            .max(f64::EPSILON),
    )
    .ok_or("invalid radius drift")?;
    let specific_energy_error = SpecificEnergy::from_square_meters_per_square_second(
        (diagnostics
            .specific_energy
            .square_meters_per_square_second()
            - initial_diagnostics
                .specific_energy
                .square_meters_per_square_second())
        .abs(),
    )
    .ok_or("invalid specific energy error")?;
    let specific_angular_momentum_error = SpecificAngularMomentum::from_square_meters_per_second(
        (diagnostics
            .specific_angular_momentum
            .square_meters_per_second()
            - initial_diagnostics
                .specific_angular_momentum
                .square_meters_per_second())
        .abs(),
    )
    .ok_or("invalid specific angular momentum error")?;
    let angular_momentum_direction = diagnostics.specific_angular_momentum_vector.normalize();

    Ok(PropagationSummary {
        elapsed: Duration::from_seconds(epoch.seconds()).ok_or("invalid elapsed duration")?,
        steps,
        radius_drift,
        specific_energy_error,
        specific_angular_momentum_error,
        eccentricity: diagnostics.eccentricity.scalar(),
        angular_momentum_direction: (
            angular_momentum_direction.x,
            angular_momentum_direction.y,
            angular_momentum_direction.z,
        ),
    })
}
