# Agent Guidance

VAST is an idiomatic Rust astrodynamics simulator whose **primary product
goal is an RL training environment** for space: massively parallel,
deterministic, loose- to medium-fidelity orbital rollouts with a
`reset` / `step(action)` environment contract. Engineers use it to train and
screen control policies at scale, then move promising results to high-fidelity
mission-analysis tools such as GMAT for detailed validation.

The codebase grows from a readable physics core into reusable simulation APIs —
scenarios, propagators, controls, observations, rewards, termination, and batch
stepping — with examples and visualization as thin consumers.

These instructions describe how to work in this repository. They are influenced
by Tiger Style, but adapted to this project: keep work bounded, make invalid
states hard to ignore, and design for performance without hiding correctness. Do
not copy TigerBeetle-specific constraints blindly.

## Product Direction

- **RL env is the north star.** When choosing between features, prefer work that
  completes the environment loop: scenario setup, propagation step, action
  application, observation/reward, and `done` signals.
- **Loose fidelity on purpose.** Two-body, J2, drag, and simple low thrust are
  in scope for training. High-fidelity gravity, adaptive integrators, ephemeris
  files, and mission-script tooling are for offline validation or later — not
  blockers for RL rollouts.
- **Batch and determinism matter.** Fixed timesteps, SoA layout, bounded work,
  and future `step_many` paths exist to run huge numbers of independent envs
  reliably.
- **GMAT-class tools are downstream.** VAST trains and filters; external
  tools confirm. Do not optimize the core roadmap to replace mission-analysis
  software.
- **Design durable contracts early** for observations, actions, rewards,
  termination, and scenario config once the environment API is in active
  development. Do not introduce these speculatively before the propagator path
  exists.

## Core Behavior

- Ask before assuming when requirements, architecture, or intent are unclear.
- Prefer the simplest solution that satisfies the request.
- Think before writing code for architecture, debugging, or non-trivial feature work. Surface the approach and tradeoffs first.
- Stay in scope. Only modify files and code directly related to the current task.
- Do not refactor, rename, reformat, or reorganize unrelated code. Mention worthwhile follow-ups instead of making them.
- Ask before big changes that restructure existing code, remove behavior, change public APIs, or alter project conventions.
- Do not preserve backwards compatibility for internal Rust APIs unless the user explicitly asks. When an internal API needs
  different semantics, update the existing method/type directly and migrate call sites.
- Do not add parallel helper methods such as `*_trusted`, `*_unchecked`, or alternate constructors merely to avoid changing 
  existing internal callers. Prefer one clear method with the correct invariant boundary.
- Admit uncertainty before it costs the user. If a fact, version, API, or approach is uncertain, say so before relying on it.
- Before destructive or irreversible actions, explain the impact and wait for explicit confirmation.

## Core Priorities

- Performance is a primary design constraint. Prefer bounded work, predictable
  memory use, cache-friendly data layout, and allocation-free stepping paths.
- Use idiomatic Rust in service of performance and correctness. Prefer simple,
  clear code, but do not choose avoidable allocation or unbounded work for
  convenience in core simulation paths.
- Keep the physics readable. Equations should remain visible, and abstractions
  should earn their place by representing real simulator concepts.
- Default to the standard library and existing dependencies before adding new
  crates.
- Use explicit domain types instead of raw numbers when units or coordinate
  systems matter. Do not expose unit-bearing scalar fields as bare `f64`.
- Make invariants executable with assertions and tests.
- Keep work bounded where practical, especially in simulation loops,
  visualization histories, queues, and future batch APIs.
- Avoid adding dependencies unless they clearly improve safety, correctness, or
  maintainability.

## Architecture Principles

- Keep the crate library-first. Examples, visualizations, CLIs, and future
  tools should be thin consumers of reusable simulation APIs.
- Own domain-facing types instead of exposing rendering, storage, control, or
  external crate details across core boundaries.
- Keep adapters behind clear boundaries. Visualization, scenario loading,
  export formats, coordinate conversion, and future control policies should not
  leak into stepping and dynamics.
- Prefer durable domain abstractions over throwaway helpers when the near-term
  roadmap already depends on the concept. For example, if orbit construction,
  inspection, and conversion are active work, model an orbit explicitly instead
  of adding one-off circular-orbit helper functions that will immediately be
  replaced.
- Add abstractions when they protect a real boundary, represent a core domain
  concept, or remove meaningful duplication. Do not add speculative
  infrastructure for concepts that are not yet part of active development.
- When concrete scenario or environment APIs need them, design trajectories,
  observations, rewards, events, and diagnostics as durable data contracts rather
  than ad hoc UI strings. Do not introduce these contracts speculatively.
- Prioritize RL-relevant boundaries: low-thrust actions belong above `Dynamics`;
  impulsive burns and mission-segment scripting are secondary to the environment
  step loop unless a task explicitly targets them.

## Invariants And Assertions

- Enforce invariants at boundaries: public constructors, domain-type arithmetic,
  and before returning new simulation state.
- Internal physics values should be valid by construction. Do not introduce
  extra "trusted" or "unchecked" method variants to route around validation.
  Instead, move the invariant boundary into the existing constructor/accessor or
  write path and update all internal call sites to use the revised contract.
- Once a value has crossed a construction or write boundary, hot-path physics
  code should trust that invariant. Avoid repeated finite/range checks on stored
  state or typed domain values unless a new floating-point result is being
  created.
- **Do not re-assert domain constraints already guaranteed by the type.**
  When a type's constructor enforces `> 0.0`, its `Add` impl should not repeat
  `debug_assert!(result > 0.0)` — the sum of two positives is provably positive.
  Reserve `debug_assert!` for checks type invariants cannot prove:
  - **Float overflow** in arithmetic on domain types.
    Rust does not check float arithmetic; `f64::MAX + f64::MAX` produces `+Inf`
    silently. Guard `Add` and `Sub` impls with `debug_assert!(result.is_finite())`.
  - **Preconditions for truly exceptional internal shortcuts.**
    Add a shortcut only when the existing method cannot own the invariant
    cleanly, and document why a second method is unavoidable. This should be
    rare; replacing the existing method is usually preferred.
- Do not use `unwrap()`, `expect()`, `panic!()`, or `assert!()` in library code.
  `debug_assert!` in conjunction with `_unchecked` constructors is the only
  exception.
- Tests should cover valid behavior and invalid boundaries. Add negative-space
  tests for impossible states, non-finite values, bad radii, invalid timesteps,
  and coordinate-system edge cases.

## Units And Coordinate Systems

- Use SI units internally unless a module explicitly says otherwise.
- Prefer scalar domain types such as `Distance`, `Speed`, and
  `GravitationalParameter` over unit-suffixed raw numbers. Keep units in type
  documentation and use clear domain names at call sites.
- Unit-suffixed local variables are acceptable only for short-lived derived
  values inside an implementation where introducing a new domain type would not
  clarify ownership or invariants. Do not use unit suffixes for public fields,
  function parameters, or reusable domain APIs.
- Do not hide coordinate conversions inside unrelated algorithms. Make frame and
  unit changes visible at the call site or in clearly named conversion methods.

## Allocation And Bounded Work

- The core stepping path should not allocate. Keep integrators and dynamics
  models allocation-free unless there is a strong reason.
- Do not introduce general storage abstractions before the core has a real use
  for them. Example-only buffers should stay in examples.
- For future history, trajectory, event, or batch APIs, prefer caller-provided
  buffers or fixed-capacity storage over hidden `Vec` growth.
- Put explicit bounds on loops that process simulation steps, histories, events,
  or render trails. If a loop is intentionally unbounded, make that clear in the
  surrounding structure.

## State Storage And Stepping

- Use `CartesianStateBuffer`, `CartesianStates`, and `CartesianStatesMut` as the
  core dense Cartesian state storage path. The internal layout is SoA for batched
  stepping, but public names should remain domain-oriented.
- Integrators should implement indexed stepping over `CartesianStatesMut` and
  remain allocation-free.
- State storage should enforce validity when data enters the buffer, then expose
  one internal read path that trusts stored values. Do not stack read-time
  validation, "trusted" variants, and scalar bypass methods for the same data.
- Do not claim mutual N-body or coupled dynamics readiness until stepping uses a
  read-old/write-new state split or another deterministic update strategy.

## Rust Style

- Use `cargo` for Rust workflows.
- Prefer clear, direct Rust over clever abstractions. Use ownership, borrowing,
  `Result`, `Option`, iterators, pattern matching, and modules idiomatically.
- Prefer borrowing over owning when a function does not need ownership: `&str`
  over `String`, `&[T]` over `Vec<T>`, and references over clones.
- Avoid unnecessary `.clone()`, `.to_string()`, intermediate `Vec`s, and
  repeated passes over data. If a clone is non-obvious, explain why it is
  needed.
- Use early returns and `?` for error propagation. Prefer `match` when it is
  clearer than nested `if let`.
- Prefer enums over strings for known constrained domains such as coordinate
  systems, operation kinds, environment states, or integrator choices.
- Keep APIs simple. Use generics or `impl Trait` when they reduce duplication
  without hiding intent.
- Do not use `unwrap()` or `expect()` in library code. Preserve the existing lint
  stance in `Cargo.toml`.
- Keep public APIs documented. `missing_docs` is currently a warning, but treat
  docs as part of the design for public types and traits.
- Keep functions short enough to understand on one screen. Split helpers when
  they clarify ownership of control flow or state changes.
- Avoid `unsafe` unless there is a measurable need. If used, keep the unsafe
  block minimal, document the invariants that make it safe, and test boundary
  conditions.

## Rust Project Standards

- Use Cargo for all Rust workflows.
- Lock the tech stack to Rust, Cargo, rustfmt, Clippy, rustdoc, and cargo-nextest unless the user asks for alternatives.
- Preserve the pinned Rust toolchain in `rust-toolchain.toml` unless asked to change it.
- Match the existing strict lint style in `Cargo.toml`.
- Prefer clear public APIs, useful documentation, and focused tests.
- Do not weaken lint levels, release profiles, or CI checks without calling it out first.

## Comments And Naming

- Comments should explain why a design is shaped a certain way, not restate the
  code.
- Use precise nouns and verbs from the simulation domain: state, dynamics,
  integrator, acceleration, epoch, duration, scenario, environment.
- Avoid abbreviations unless they are standard in orbital mechanics or units.
- Prefer names that compose cleanly as the simulator grows.

## Dependencies

- Treat each dependency as a design decision. Consider correctness, API
  stability, compile time, maintenance cost, and whether it belongs in core or
  only in examples/tools.
- Visualization dependencies may live in examples or tools, but core modules
  must not expose them in public simulation APIs.
- Use `thiserror` for typed library errors when error enums become useful.
  Reserve broad error handling for executable/setup boundaries.

## Testing Principles

- Add unit tests for non-trivial logic and edge cases.
- Prefer invariant, boundary, and contract tests over tests that mirror
  implementation details.
- Use table-driven tests when many cases share the same structure.
- Protect behavior that would damage trust if wrong: conservation properties,
  coordinate conversions, integrator error, termination conditions,
  deterministic resets, and invalid physical states.
- Avoid tests that rely on timing or flaky behavior.

## Architecture Documentation

`docs/architecture.md` is the living architecture reference: layer boundaries,
simulation vs physics concepts, data flow, and open design questions.

### Before Starting Significant Work

- Read `docs/architecture.md` when the task touches scenarios, propagators,
  termination, events, controls, environments, or other simulation-layer APIs.
- Read `ROADMAP.md` for prioritized unimplemented work in the same area.

### After Major Work

Keep `docs/architecture.md` in sync in the same change set when work:

- adds or changes public simulation-layer types or module boundaries
- implements items from the Simulation Layer roadmap category
- resolves an open design question listed in the architecture doc
- shifts what is "implemented today" vs planned

Updates should be concise: adjust the layered model, "what exists today",
planned concepts, or open questions — not a changelog of every commit.

### Review Expectations

- Treat a stale architecture doc as a bug when the crate's public shape or
  layer boundaries no longer match the document.
- Mention architecture doc updates in the task summary when they were part of
  the change.

## Roadmap Maintenance

`ROADMAP.md` is the living plan for recommended, not-yet-done work. Keep it
current in the same change set as the code or decision it reflects.

### Before Starting Significant Work

- Read `ROADMAP.md` to see whether the task is already planned, deprioritized,
  or scoped with explicit deferrals.
- If the work is not on the roadmap but is a durable simulator capability,
  add it before or as part of the implementation PR unless the user asked for a
  one-off experiment.

### When Adding New Items

- Place the item under the best-fit category. Create a new category section only
  when the work does not fit an existing one.
- Assign exactly one priority: **P1** (next), **P2** (soon after), or **P3**
  (later). Do not use finer-grained priority numbers.
- Include **What**, **Why**, **How it helps**, **First version scope**, and
  **Defer** when the item is non-trivial.
- Run the design checklist at the bottom of `ROADMAP.md` before adding force
  models or other hot-path features.
- Do not add speculative items the project has not decided to pursue. If
  something belongs in "avoid for now", leave it off the roadmap.

### When Completing Work

- Remove the finished item from its category section.
- Add a concise bullet to the **Implemented baseline** list at the top of
  `ROADMAP.md`.
- If completion only covers the first-version scope, add follow-on work as new
  P2/P3 items under the same or a related category rather than leaving a stale
  entry.
- If priorities shift because something else is now more important, rebalance
  P1/P2/P3 within the affected category.
- If the finished work changes what is implemented or planned at the layer level,
  update `docs/architecture.md` accordingly.

### Review Expectations

- Treat a stale roadmap as a bug. Finished features should not remain listed as
  unimplemented; planned work should not be missing after a substantive merge.
- Mention roadmap updates in the task summary when they were part of the change.

## Verification

After substantive code changes, run the relevant checks:

```bash
cargo fmt --check
cargo clippy --all-targets --all-features -- -D warnings
cargo doc --no-deps --all-features
cargo test
```

If `cargo nextest` is installed, also run:

```bash
cargo nextest run --all-targets --all-features
```

## Communication

- Never open with filler phrases like "Great question", "Of course", or "Certainly". Start with the answer or action.
- Match length to the task. Be concise for small tasks and more detailed for design/debugging work.
- Summarize what changed and how it was verified.
- Mention useful follow-ups, but do not implement them without being asked.

## Review Checklist

- Is the ownership and borrowing model minimal and correct?
- Are there unnecessary clones, allocations, temporary collections, or panics?
- Are core APIs ergonomic, with borrows where possible and ownership where
  appropriate?
- Are invariants checked at the right boundaries?
- Is this the simplest correct solution for the current repo, not imagined
  future infrastructure?
- Would `cargo fmt`, `cargo test`, or strict Clippy complain?
- If this change completes or introduces planned work, is `ROADMAP.md` updated?
- If this change affects layers, boundaries, or resolved design questions, is
  `docs/architecture.md` updated?
