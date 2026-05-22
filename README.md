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

The physics core and an **RL Ops MVP** are implemented: scenario TOML,
propagator, termination, `LeoCircularEnv`, run artifacts, the `runner` CLI/API,
and a React replay UI. Low-thrust control, multi-step horizons, and batch
training are next — see the architecture doc and roadmap.

## Running

### Visualization example

```bash
cargo run --example orbit_viz
```

### RL Ops MVP (rollouts + replay UI)

```bash
# 1. Run headless episodes → writes runs/<run_id>/
cargo run --bin runner -- run --scenario scenarios/leo_circular.toml --episodes 3

# 2. Serve run artifacts to the UI
cargo run --bin runner -- serve --runs runs --port 3001

# 3. Start the web UI (proxies /api to the runner)
cd ui && npm install && npm run dev
```

Open the Vite dev server URL (default `http://localhost:5173`). The UI falls back
to bundled sample data under `ui/public/sample-run/` when the API is offline.

## License

[TBD]
