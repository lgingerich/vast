# Agent Guidance

Space Sim is an idiomatic Rust astrodynamics simulator. It is growing from a
small, readable physics core into reusable simulation APIs for scenarios,
environments, visualization, and automated experiments.

These instructions describe how to work in this repository. They are influenced
by Tiger Style, but adapted to this project: keep work bounded, make invalid
states hard to ignore, and design for performance without hiding correctness. Do
not copy TigerBeetle-specific constraints blindly.

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

## Invariants And Assertions

- Put invariants on the type that owns the meaning. Use the existing
  `debug_assert_valid()` pattern on `Epoch`, `Duration`, `Position`, `Velocity`,
  `Acceleration`, and state types instead of repeating component checks in every
  algorithm.
- Call invariant checks at boundaries: before using caller-provided state, after
  computing derived values, and before returning new simulation state.
- Split unrelated assertions. Prefer:

  ```rust
  debug_assert!(radius_m.is_finite());
  debug_assert!(radius_m > 0.0);
  ```

  over one compound assertion.
- Assert positive space directly when possible. For example, prefer
  `radius_m > 0.0` over negated forms.
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
- Keep coordinate systems explicit in type names and docs. Today the core uses
  `CartesianState`; related representations such as Keplerian elements,
  circular orbits, geodetic coordinates, body-fixed frames, or inertial frames
  should own their own invariants and conversions when they become active
  concepts.
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
- Run these checks after substantive changes:

  ```bash
  cargo fmt
  cargo test
  cargo clippy --all-targets --all-features
  ```

- Do not use `unwrap()` or `expect()` in library code. Preserve the existing lint
  stance in `Cargo.toml`.
- Keep public APIs documented. `missing_docs` is currently a warning, but treat
  docs as part of the design for public types and traits.
- Keep functions short enough to understand on one screen. Split helpers when
  they clarify ownership of control flow or state changes.
- Avoid `unsafe` unless there is a measurable need. If used, keep the unsafe
  block minimal, document the invariants that make it safe, and test boundary
  conditions.

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

## Review Checklist

- Is the ownership and borrowing model minimal and correct?
- Are there unnecessary clones, allocations, temporary collections, or panics?
- Are core APIs ergonomic, with borrows where possible and ownership where
  appropriate?
- Are invariants checked at the right boundaries?
- Is this the simplest correct solution for the current repo, not imagined
  future infrastructure?
- Would `cargo fmt`, `cargo test`, or strict Clippy complain?
