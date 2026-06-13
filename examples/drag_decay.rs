#![allow(missing_docs)]

use vast::{
    Angle, AngularRate, AreaPerMass, AtmosphericDrag, CartesianStateBuffer, CompositeDynamics,
    Distance, Duration, Eccentricity, EllipticOrbit, Epoch, ExponentialAtmosphere,
    GravitationalParameter, Integrator, MU_EARTH, MassDensity, OrbitDirection, OrbitalDiagnostics,
    R_EARTH, RungeKutta4Integrator, TimeStep, TwoBodyGravity,
};

const ALTITUDE_M: f64 = 250_000.0;
const STEP_SECONDS: f64 = 10.0;
const PROPAGATION_HOURS: f64 = 6.0;

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
    let initial_state = orbit.to_cartesian_state()?;
    let atmosphere = ExponentialAtmosphere::new(
        Distance::from_meters(R_EARTH)?,
        Distance::from_meters(ALTITUDE_M)?,
        MassDensity::from_kilograms_per_cubic_meter(1.0e-10)?,
        Distance::from_meters(50_000.0)?,
        AngularRate::from_radians_per_second(7.292_115_9e-5)?,
    );
    let dynamics = CompositeDynamics::new(
        TwoBodyGravity::new(mu),
        AtmosphericDrag::new(
            atmosphere,
            AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        ),
    );
    let integrator = RungeKutta4Integrator;
    let dt = TimeStep::from_seconds(STEP_SECONDS)?;
    let duration = Duration::from_seconds(PROPAGATION_HOURS * 3_600.0)?;
    let mut states = CartesianStateBuffer::<1>::new();
    states.write_from_state(0, &initial_state)?;

    let initial_diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
    propagate(&integrator, &dynamics, &mut states, duration, dt)?;
    let final_diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;

    let initial_altitude_m = initial_diagnostics.radius().meters() - R_EARTH;
    let final_altitude_m = final_diagnostics.radius().meters() - R_EARTH;
    let energy_delta = final_diagnostics
        .specific_energy()
        .square_meters_per_square_second()
        - initial_diagnostics
            .specific_energy()
            .square_meters_per_square_second();
    let speed_delta =
        final_diagnostics.speed().meters_per_second() - initial_diagnostics.speed().meters_per_second();

    println!(
        "Drag decay demo: initial altitude {:.0} km | duration {:.1} h",
        initial_altitude_m / 1000.0,
        duration.seconds() / 3_600.0,
    );
    println!(
        "final altitude: {:.3} km | altitude change: {:+.3} km",
        final_altitude_m / 1000.0,
        (final_altitude_m - initial_altitude_m) / 1000.0,
    );
    println!(
        "specific energy change: {:+.6} | speed change: {:+.6} m/s",
        energy_delta, speed_delta,
    );
    println!(
        "eccentricity: {:.9}",
        final_diagnostics.eccentricity().scalar(),
    );

    Ok(())
}

fn propagate<I: Integrator, D: vast::Dynamics>(
    integrator: &I,
    dynamics: &D,
    states: &mut CartesianStateBuffer<1>,
    duration: Duration,
    dt: TimeStep,
) -> Result<(), Box<dyn std::error::Error>> {
    let mut epoch = Epoch::from_seconds(0.0)?;

    while epoch.seconds() < duration.seconds() {
        let remaining_seconds = duration.seconds() - epoch.seconds();
        let step = TimeStep::from_seconds(dt.seconds().min(remaining_seconds))?;
        integrator.step(dynamics, &mut states.view_mut(), 0, epoch, step)?;
        epoch = epoch.advance_by(step)?;
    }

    Ok(())
}
