# Space Sim

Space Sim is becoming a small, modern orbital simulation platform: a place to build, run, and inspect spacecraft behavior from first principles.

The long-term goal is not just to draw an orbit. It is to grow a simulation core that can support interactive exploration, repeatable scenarios, automated experiments, and eventually more demanding control loops.

## Direction

- **Readable physics core**: Keep the equations visible and the abstractions small enough to reason about.
- **Real simulation concepts**: Use SI units, explicit dynamics models, and swappable integrators from the beginning.
- **Scenario-driven design**: Make initial conditions, controls, rewards, and termination conditions easy to compose.
- **Growing fidelity**: Start with two-body motion, then add better numerics, thrust, contacts, multiple bodies, and environment tooling.
- **Performance-aware foundation**: Keep the design compatible with batched stepping, parallel execution, and efficient data layout.

## Documentation

- **[Architecture](docs/architecture.md)** — layered simulator model, physics vs simulation boundaries, and design targets
- **[Roadmap](ROADMAP.md)** — prioritized unimplemented work

The physics core (state, dynamics, integrators) is implemented today. The
simulation layer (scenario, propagator, termination, maneuvers, events,
environments) is planned — see the architecture doc and roadmap.

## Running

```bash
cargo run --example orbit_viz
```

## License

[TBD]
