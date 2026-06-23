#![allow(missing_docs)]

use vast::{Action, Environment, LeoCircularEnv, ScenarioConfig, stop_reason_name};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // --- load rollout scenario ---
    let config = ScenarioConfig::from_toml(
        r#"
        name = "leo_env_rollout"
        body = "earth"
        altitude_m = 400_000.0
        dt_s = 10.0
        duration_s = 300.0
        integrator = "leapfrog"
        force_model = { kind = "two_body" }
        sample_every_steps = 10
        steps_per_env_step = 20
        "#,
    )?;
    let mut env = LeoCircularEnv::from_config(&config)?;

    // --- reset environment ---
    let seed = 7;
    let initial = env.reset(seed)?;
    println!("Environment rollout demo: {}", config.name);
    println!(
        "seed: {} | dt: {:.1} s | integrator steps per env step: {}",
        seed, config.dt_s, config.steps_per_env_step,
    );
    println!(
        "initial altitude: {:.3} km | speed: {:.3} km/s",
        initial.altitude_m / 1000.0,
        initial.speed_mps / 1000.0,
    );
    println!("action: inertial acceleration [0.0, 2.0e-4, 0.0] m/s^2 | reward: MVP zero reward",);
    println!();
    println!(
        "{:>8} {:>8} {:>14} {:>14} {:>12} {:>16}",
        "env step", "intg", "altitude(km)", "speed(km/s)", "reward", "done",
    );

    // --- step until done ---
    let action = Action::inertial([0.0, 2.0e-4, 0.0])?;
    let mut env_steps = 0_usize;
    let mut integrator_steps = 0_usize;
    let mut total_reward = 0.0;
    let stop_reason = loop {
        let step = env.step(action)?;
        env_steps += 1;
        integrator_steps += step.steps;
        total_reward += step.reward;

        if env_steps == 1 || env_steps.is_multiple_of(5) || step.done {
            println!(
                "{:>8} {:>8} {:>14.3} {:>14.6} {:>12.3} {:>16}",
                env_steps,
                integrator_steps,
                step.observation.altitude_m / 1000.0,
                step.observation.speed_mps / 1000.0,
                step.reward,
                step.done,
            );
        }

        if step.done {
            break step.stop_reason;
        }
    };

    println!();
    println!(
        "finished after {} env steps and {} integrator steps",
        env_steps, integrator_steps,
    );
    println!("total reward: {:.3}", total_reward);
    if let Some(reason) = stop_reason {
        println!("stop reason: {}", stop_reason_name(reason));
    }

    Ok(())
}
