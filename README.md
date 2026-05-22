# Space Sim

Space Sim is a Rust orbital simulation platform built primarily as a
**reinforcement-learning training environment**: run large numbers of
deterministic, loose- to medium-fidelity physics rollouts in parallel, train
control policies, then hand promising trajectories to high-fidelity tools such as
GMAT for detailed mission analysis.

The long-term product is not a GMAT replacement or a visualization demo. It is
**fast, composable RL infrastructure for space** — scenarios, propagators,
actions, observations, rewards, and termination conditions that engineers can
scale to massive batch training and later validate precisely elsewhere.

## Direction

- **RL-first product**: The core API target is `reset` / `step(action)` over
  orbital environments, not one-off propagation scripts.
- **Train at scale, validate elsewhere**: Prefer fixed-step, batch-friendly
  physics that is good enough to learn from; defer mission-analysis fidelity
  (adaptive integrators, high-degree gravity, ephemerides) to external tools.
- **Readable physics core**: Keep equations visible so engineers trust rollouts
  enough to drill down in GMAT or similar when a policy looks promising.
- **Scenario-driven design**: Compose initial conditions, forces, controls,
  rewards, and stopping rules as reusable environment definitions.
- **Performance-aware foundation**: SoA state, allocation-free stepping, and
  batched backends to support many parallel environments.

## Documentation

- **[Architecture](docs/architecture.md)** — layered simulator model, physics vs simulation boundaries, and design targets
- **[Roadmap](ROADMAP.md)** — prioritized unimplemented work

The physics core (state, dynamics, integrators) is implemented today. The
simulation and environment layers (scenario, propagator, termination, low-thrust
control, observations, rewards) are the active product path — see the
architecture doc and roadmap.

## Running

```bash
cargo run --example orbit_viz
```

## License

[TBD]
