# Architecture

Project-specific invariants live here; universal Rust workflow lives in `AGENTS.md`. This file is longer than the template's one-page target because it holds the repo's former project-specific agent rules. Layer detail, data flow, and open design questions live in the living reference `docs/architecture.md`; planned work lives in `ROADMAP.md`.

## Purpose

VAST (Vectorized Astrodynamics Simulation Toolkit) is an idiomatic Rust astrodynamics simulator whose **primary product goal is an RL training environment** for space: massively parallel, deterministic, loose- to medium-fidelity orbital rollouts with a `reset` / `step(action)` contract. Engineers train and screen control policies at scale, then move promising results to high-fidelity mission-analysis tools such as GMAT for validation. The crate grows from a readable physics core into reusable simulation APIs (scenarios, propagators, controls, observations, rewards, termination, batch stepping); examples and visualization are thin consumers.

It does NOT try to replace GMAT-class tools: VAST trains and filters, external tools confirm.

Product direction:
- **RL env is the north star.** Prefer work that completes the environment loop: scenario setup, propagation step, action application, observation/reward, `done` signals.
- **Loose fidelity on purpose.** Two-body, J2, drag, and simple low thrust are in scope. High-fidelity gravity, adaptive integrators, ephemeris files, and mission-script tooling are for offline validation or later, not blockers for RL rollouts.
- **Batch and determinism matter.** Fixed timesteps, SoA layout, bounded work, and future `step_many` paths exist to run huge numbers of independent envs reliably.
- **Design durable contracts early, not speculatively.** Observations, actions, rewards, termination, events, diagnostics, and scenario config should become durable data contracts (not ad hoc UI strings) once the environment API is in active development, and not before the propagator path exists.

## Module Boundaries

Layers, lowest first. Lower layers must not depend on higher ones (see `docs/architecture.md`).

- `src/physics/` (`pub` modules, re-exported from `src/lib.rs`): the physics core. Domain types (`types.rs`), frames, Cartesian state storage (`state.rs`), `Dynamics` models, integrators, orbits and elements, thrust, and the `Sim` builder. Depends on nothing above it (no `crate::sim`); only on `crate::errors`.
- `src/sim/` (`pub(crate)` modules, public items re-exported from `src/lib.rs`): simulation orchestration. TOML config DTOs (`config.rs`), validated runtime plans (`plan.rs`), the propagator (`engine.rs`), termination (`termination.rs`), `Environment` / `LeoCircularEnv` (`environment.rs`), and the mission runner (`mission.rs`). Uses physics through typed state, dynamics, integrators, and orbit construction.
- `src/recorder.rs`: run artifacts (manifests, per-episode metrics, trajectory JSONL, read helpers). Depends on `sim` config/engine/termination types.
- `src/errors.rs`: typed `thiserror` enums shared by all layers.
- `src/bin/runner.rs`: `runner` CLI (`run`, `serve`); consumes only the public `vast` API. `axum`/`tokio`/`clap` live here, not in the library modules.
- `examples/`, `ui/` (React/R3F replay UI), `scenarios/`: thin consumers of the library.

Architecture principles:
- Keep the crate library-first. Examples, visualizations, CLIs, and future tools are thin consumers of reusable simulation APIs.
- Own domain-facing types instead of exposing rendering, storage, control, or external crate details across core boundaries.
- Keep adapters behind clear boundaries. Visualization, scenario loading, export formats, coordinate conversion, and future control policies must not leak into stepping and dynamics.
- Prefer durable domain abstractions over throwaway helpers when the near-term roadmap already depends on the concept (e.g. if orbit construction, inspection, and conversion are active work, model an orbit explicitly instead of one-off circular-orbit helpers that will be replaced).
- Add abstractions only when they protect a real boundary, represent a core domain concept, or remove meaningful duplication. No speculative infrastructure for concepts not yet in active development.
- Prioritize RL-relevant boundaries: low-thrust actions belong above `Dynamics`; impulsive burns and mission-segment scripting are secondary to the environment step loop unless a task explicitly targets them.

State storage and stepping:
- `CartesianStateBuffer`, `CartesianStates`, and `CartesianStatesMut` are the core dense Cartesian state storage path. The internal layout is SoA for batched stepping; public names stay domain-oriented.
- Integrators implement indexed stepping over `CartesianStatesMut` and remain allocation-free.
- State storage enforces validity when data enters the buffer, then exposes one internal read path that trusts stored values. Do not stack read-time validation, "trusted" variants, and scalar bypass methods for the same data.
- Do not claim mutual N-body or coupled dynamics readiness until stepping uses a read-old/write-new state split or another deterministic update strategy.

## Core Invariants

- **Performance is a primary design constraint.** Prefer bounded work, predictable memory, cache-friendly layout, and allocation-free stepping. Do not choose avoidable allocation or unbounded work for convenience in core simulation paths.
- **Allocation and bounded work.** The core stepping path (integrators, dynamics models) does not allocate without a strong reason. For future history, trajectory, event, or batch APIs, prefer caller-provided buffers or fixed-capacity storage over hidden `Vec` growth. Put explicit bounds on loops that process simulation steps, histories, events, or render trails; if a loop is intentionally unbounded, make that clear in the surrounding structure. Do not add general storage abstractions before the core has a real use; example-only buffers stay in examples.
- **Readable physics.** Equations stay visible; abstractions must represent real simulator concepts.
- **Invariants at boundaries.** Enforce them in public constructors, domain-type arithmetic, and before returning new simulation state. Internal physics values are valid by construction: move the invariant boundary into the existing constructor/accessor or write path and update all internal call sites rather than adding "trusted"/"unchecked" variants. Once a value has crossed a construction or write boundary, hot-path physics code trusts it; avoid repeated finite/range checks on stored state or typed values unless a new floating-point result is being created.
- **Do not re-assert what the type guarantees.** If a constructor enforces `> 0.0`, its `Add` impl must not repeat `debug_assert!(result > 0.0)`. Reserve `debug_assert!` for what types cannot prove: float overflow (Rust does not check float arithmetic; `f64::MAX + f64::MAX` is silently `+Inf`, so guard `Add`/`Sub` impls with `debug_assert!(result.is_finite())`) and preconditions of truly exceptional internal shortcuts (add one only when the existing method cannot own the invariant, and document why a second method is unavoidable; replacing the existing method is usually preferred).
- **Units and frames.** SI units internally unless a module explicitly says otherwise. Use scalar domain types (`Distance`, `Speed`, `GravitationalParameter`, ...) instead of unit-suffixed raw numbers; keep units in type docs. Do not expose unit-bearing scalar fields as bare `f64`. Unit-suffixed locals are acceptable only for short-lived derived values inside an implementation where a new domain type would not clarify ownership or invariants, never for public fields, function parameters, or reusable domain APIs. Do not hide coordinate conversions inside unrelated algorithms; make frame and unit changes visible at the call site or in clearly named conversion methods.
- **Typed domains.** Prefer enums over strings for known constrained domains (coordinate systems, operation kinds, environment states, integrator choices).
- **Executable invariants.** Make invariants executable with assertions and tests. Tests cover valid behavior and invalid boundaries (negative-space tests for impossible states, non-finite values, bad radii, invalid timesteps, coordinate-system edge cases). Protect behavior that would damage trust if wrong: conservation properties, coordinate conversions, integrator error, termination conditions, deterministic resets, invalid physical states. Prefer invariant, boundary, and contract tests over implementation mirrors; use table-driven tests when cases share a structure.
- **Comments and naming.** Comments explain why a design is shaped a certain way, not what the code does. Use precise simulation-domain nouns and verbs (state, dynamics, integrator, acceleration, epoch, duration, scenario, environment); avoid abbreviations unless standard in orbital mechanics or units; prefer names that compose as the simulator grows.
- **Function size.** Keep functions short enough to understand on one screen; split helpers when they clarify ownership of control flow or state changes.
- **Docs.** Keep public APIs documented; treat docs as part of the design for public types and traits.

## Forbidden Patterns / Dependencies

- No `unwrap()`, `expect()`, `panic!()`, or `assert!()` in library code (`unwrap_used` / `expect_used` are linted in `Cargo.toml`). `debug_assert!` is allowed for the float-overflow and exceptional-precondition cases above. (The former exception for `_unchecked` constructors no longer applies: `AGENTS.md` forbids `*_unchecked` helpers, and none exist in `src/`.)
- No `*_trusted` / `*_unchecked` method variants to route around validation.
- No visualization, rendering, or HTTP types (`macroquad`, `axum`, `tower-http`, `tokio`) in public simulation APIs or in `src/physics` / `src/sim`. `macroquad` is a dev-dependency used by examples only; `axum`/`tokio`/`clap` are used by `src/bin/runner.rs`.
- No `src/physics` dependency on `src/sim` or `src/recorder`.
- No scenario loading, export formats, or control policies leaking into stepping and dynamics.
- Treat each new dependency as a design decision (correctness, API stability, compile time, maintenance, whether it belongs in core or only in examples/tools). Default to the standard library and existing dependencies.
- Reserve broad error handling (`anyhow`-style) for executable/setup boundaries such as `runner`; use `thiserror` enums in the library (`src/errors.rs`).

## Agent Workflow
`docs/architecture.md` is the living architecture reference (layer boundaries, simulation vs physics concepts, data flow, open design questions); `ROADMAP.md` is the living plan for recommended, not-yet-done work.

### Architecture Documentation
Before significant work:
- Read `docs/architecture.md` when the task touches scenarios, propagators, termination, events, controls, environments, or other simulation-layer APIs.
- Read `ROADMAP.md` for prioritized unimplemented work in the same area.

After major work, keep `docs/architecture.md` in sync in the same change set when work:
- adds or changes public simulation-layer types or module boundaries
- implements items from the Simulation Layer roadmap category
- resolves an open design question listed in the architecture doc
- shifts what is "implemented today" vs planned

Updates are concise: adjust the layered model, "what exists today", planned concepts, or open questions, not a changelog of every commit. Treat a stale architecture doc as a bug when the crate's public shape or layer boundaries no longer match it. Mention architecture doc updates in the task summary.

### Roadmap Maintenance
Keep `ROADMAP.md` current in the same change set as the code or decision it reflects.

Before significant work:
- Read `ROADMAP.md` to see whether the task is already planned, deprioritized, or scoped with explicit deferrals.
- If the work is not on the roadmap but is a durable simulator capability, add it before or as part of the implementation PR unless the user asked for a one-off experiment.

When adding items:
- Place the item under the best-fit category. Create a new category only when the work does not fit an existing one.
- Assign exactly one priority: **P1** (next), **P2** (soon after), or **P3** (later). No finer-grained priority numbers.
- Include **What**, **Why**, **How it helps**, **First version scope**, and **Defer** when the item is non-trivial.
- Run the design checklist at the bottom of `ROADMAP.md` before adding force models or other hot-path features.
- Do not add speculative items the project has not decided to pursue. If something belongs in "avoid for now", leave it off the roadmap.

When completing work:
- Remove the finished item from its category section.
- Add a concise bullet to the **Implemented baseline** list at the top of `ROADMAP.md`.
- If completion only covers the first-version scope, add follow-on work as new P2/P3 items under the same or a related category rather than leaving a stale entry.
- If priorities shift because something else is now more important, rebalance P1/P2/P3 within the affected category.
- If the finished work changes what is implemented or planned at the layer level, update `docs/architecture.md`.

Treat a stale roadmap as a bug: finished features must not remain listed as unimplemented, and planned work must not be missing after a substantive merge. Mention roadmap updates in the task summary.

### Review Checklist
- Is the ownership and borrowing model minimal and correct?
- Are there unnecessary clones, allocations, temporary collections, or panics?
- Are core APIs ergonomic, with borrows where possible and ownership where appropriate?
- Are invariants checked at the right boundaries?
- Is this the simplest correct solution for the current repo, not imagined future infrastructure?
- Would `cargo fmt`, `cargo test`, or strict Clippy complain?
- If this change completes or introduces planned work, is `ROADMAP.md` updated?
- If this change affects layers, boundaries, or resolved design questions, is `docs/architecture.md` updated?
