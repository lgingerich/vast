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

## Near-Term Milestones

### Tier 1: Make Orbits Inspectable

- Add pause, reset, time-scale, and step controls.
- Draw velocity and acceleration vectors.
- Display radius, altitude, speed, and orbital period.
- Add tests for circular orbit setup and gravity direction.

### Tier 2: Teach Numerical Methods

- Add RK4 integration.
- Track orbital energy and angular momentum.
- Compare Euler, semi-implicit Euler, and RK4 visually.
- Make time step error obvious in the UI.

### Tier 3: Add Spacecraft Behavior

- Add thrust acceleration.
- Add finite burns and impulsive burns.
- Add action interfaces for maneuver experiments.
- Add collision or surface impact detection.

### Tier 4: Shape Environments

- Add resettable scenarios.
- Add observations, actions, rewards, and termination conditions.
- Add deterministic seeds.
- Add trajectory history and export.

### Tier 5: Expand the World

- Add multiple bodies.
- Add configurable central bodies.
- Add rendezvous and landing scenarios.
- Add configurable environment parameters.

### Tier 6: Build Toward Scale

- Separate simulation stepping from rendering.
- Introduce data layouts suitable for many bodies.
- Explore batched and parallel stepping.
- Add benchmarks for integrators and force models.

## Open Questions

- Should the first-class public API focus on spacecraft environments or general body simulation?
- Should 2D become a first-class type, or should the simulator keep using 3D vectors with `z = 0` for now?
- Should visualization stay native with `macroquad`, or eventually split into a separate frontend?

## License

[TBD]
