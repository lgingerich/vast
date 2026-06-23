#![allow(missing_docs)]

use std::path::Path;

use vast::{
    GravitationalParameter, LoadedScenario, MU_EARTH, OrbitalElementClassification,
    OrbitalElements, R_EARTH, run_mission, stop_reason_name,
};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- load mission scenario ---
    let scenario_path = Path::new("scenarios/hohmann_transfer.toml");
    let LoadedScenario::Mission(config) = LoadedScenario::from_file(scenario_path)? else {
        return Err(format!("{} is not a mission scenario", scenario_path.display()).into());
    };

    // --- run mission ---
    let result = run_mission(&config)?;
    let mu = GravitationalParameter::new(MU_EARTH);
    let final_elements = OrbitalElements::from_state(
        &result.final_state,
        mu,
        OrbitalElementClassification::default(),
    )?;
    let final_radius = result.final_state.position.vector().norm();

    // --- print report ---
    println!("Mission transfer demo: {}", config.name);
    println!("scenario: {}", scenario_path.display());
    println!(
        "initial burns: {} | segments: {}",
        config.initial_burns.len(),
        result.segments.len(),
    );
    println!();
    println!(
        "{:>7} {:>14} {:>10} {:>12} {:>14} {:>8}",
        "segment", "stop", "steps", "elapsed(h)", "samples", "burns",
    );

    for segment in &result.segments {
        println!(
            "{:>7} {:>14} {:>10} {:>12.3} {:>14} {:>8}",
            segment.segment_index,
            stop_reason_name(segment.propagation.stop_reason),
            segment.propagation.steps,
            segment.propagation.final_epoch.seconds() / 3_600.0,
            segment.propagation.samples.len(),
            segment.burns_applied,
        );
    }

    println!();
    println!(
        "final epoch: {:.3} h | final altitude: {:.3} km",
        result.final_epoch.seconds() / 3_600.0,
        final_radius / 1000.0 - R_EARTH / 1000.0,
    );
    println!(
        "final orbit: a = {:.3} km | e = {:.9} | i = {:.3} deg",
        final_elements.semi_major_axis().meters() / 1000.0,
        final_elements.eccentricity().scalar(),
        final_elements.inclination().degrees(),
    );

    Ok(())
}
