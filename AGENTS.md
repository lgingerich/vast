# AGENTS.md

Read `ARCHITECTURE.md` before non-trivial changes.

## Workflow & Behavior
- Ask before assuming when requirements, architecture, or intent are unclear.
- Think before coding on design or debugging; surface tradeoffs first.
- Stay strictly in scope: only edit files required by the task. Mention unrelated follow-ups instead of making them.
- Ask before restructuring code, changing public APIs, weakening lints/CI/profiles, adding dependencies, or running destructive actions.
- Admit uncertainty instead of guessing.
- No filler openers ("Great question", "Certainly"). Start with the answer or action; end with what changed and how it was verified.

## Minimum Code That Works
Understand the problem and trace the real flow first, then stop at the first rung that holds:
1. Does this need to be built at all? (YAGNI)
2. Does a helper or pattern already exist in this repo? Reuse it.
3. Does `std`, the platform, or an existing dependency cover it? Use it.
4. Can it be one line? Make it one line.
5. Otherwise, write the minimum code that works.

- Deletion over addition; boring over clever; fewest files possible.
- Fix root causes, not symptoms: grep all callers of any shared function you touch and fix it once.
- Question overbuilt requests ("Do you need X, or does Y cover it?").
- When two stdlib options are the same size, pick the edge-case-correct one.
- Mark deliberate simplifications that cut a corner with a known ceiling (e.g., global lock, O(n^2) scan) with a comment naming the ceiling and upgrade path.
- Never cut corners on trust-boundary validation, data-loss prevention, security, or explicitly requested behavior. Non-trivial logic must leave one minimal runnable check behind.

## Rust & API Rules
`Cargo.toml` and `clippy.toml` enforce formatting, `unsafe` bans, docs, panics, complexity, and common perf lints. Beyond those:
- **No internal back-compat**: when internal semantics change, update the method/type and migrate callers. Never add parallel `*_trusted`/`*_unchecked` helpers or duplicate constructors.
- **Conversions on the type**: implement `Display`/`FromStr`, `From`/`TryFrom`/`AsRef`/`Borrow` rather than one-off `to_*`/`from_*` free functions.
- **Public APIs**: private fields with `pub fn` accessors; accept borrowed inputs (`&str`, `&[T]`) and return owned values unless zero-copy is explicit; keep external/wire types at crate boundaries; use identical accessor names across sibling types.
- **Errors**: typed `thiserror` in libraries, `anyhow` in binaries/tests/examples, no `Box<dyn Error>`. Layer bottom-up with `#[from]` (`leaf -> domain -> boundary`); leaf errors name the bad input/constraint. Avoid `map_err` unless injecting boundary-only context or crossing orphan-rule limits.
- **Performance**: measure a release-with-symbols build before optimizing, and keep the benchmark justifying any `#[inline]`, `#[cold]`, or custom hasher. Keep hot paths zero-allocation, bounded, contiguous, and statically dispatched (`dyn Trait`/`Arc`/`Box` at boundaries). Never add `unsafe` for speed without measurements and sign-off.

## Tests
- Group integration tests under `tests/integration/main.rs` (plus `tests/live/main.rs` only for network/credentials) with submodules. Never place `.rs` files directly in `tests/` and never set `autotests = false`. Build fixture paths from `env!("CARGO_MANIFEST_DIR")`.
- Test observable contracts, invariants, edge cases, and failure isolation—not implementation details or incidental formatting.
- Prefer real fixtures for formats/parsers and small local fakes over mocks. Write a behavior-named regression test before fixing a reproducible bug; use property tests for parsers and round-trips.
- No arbitrary sleeps in async tests; use explicit synchronization or time control.

## Verification
After code changes, run the CI-equivalent checks (prefix with `RUSTC_WRAPPER=""` if `kache` is not installed):

```bash
cargo fmt --check
CARGO_BUILD_WARNINGS=deny cargo clippy --all-targets --all-features -- -D warnings
RUSTDOCFLAGS="-D warnings" cargo doc --no-deps --all-features
cargo test --doc
cargo nextest run --all-targets --all-features --no-tests pass # or: cargo test --all-targets --all-features
```

## Project-Specific
Project direction, layers, invariants, units, allocation, and dependency rules live in `ARCHITECTURE.md`. `docs/architecture.md` is the living architecture reference (layer boundaries, simulation vs physics concepts, data flow, open design questions); `ROADMAP.md` is the living plan for recommended, not-yet-done work.

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
