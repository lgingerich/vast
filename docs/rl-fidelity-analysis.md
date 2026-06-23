# RL Fidelity Analysis

Reference analysis of simulator fidelity choices for RL training in space
operations. Triggered by discussion of
[bsk_rl](https://github.com/AVSLab/bsk_rl) (Basilisk-based RL) and the case for
a medium-fidelity, highly-parallelizable sim.

**This is an analysis document.** Open questions are flagged explicitly.

---

## Context

Two external prose pieces were used as discussion prompts. This document
preserves the key arguments and maps them to vast's current state and
roadmap, without committing to architectural direction.

The core tension:
- High-fidelity sims (Basilisk) capture all dynamics but limit parallelism.
- Medium-fidelity sims sacrifice precision for training throughput.
- The question is which fidelity dimensions matter for policy transfer.

---

## The Fidelity Spectrum

| Simulator | Orbit | Attitude | Power | Thermal | Data | Parallelism |
|-----------|-------|----------|-------|---------|------|-------------|
| GMAT | SPICE + harmonics | Full rigid body | — | — | — | Single-threaded |
| Basilisk | SPICE + harmonics | Reaction wheels, desat, MRP | Battery, solar, eclipse | Modeled | Buffer, downlink | ~100 envs |
| Space-sim (today) | Two-body + J2 + drag | None (3DOF) | None | None | None | Designed for 10K+ |
| Hypothetical Warp sim | Two-body + J2 | Simplified slew + wheels | Linear battery | None | Linear buffer | 10K+ GPU |
| Kepler two-body | Kepler only | None | None | None | None | Infinite (analytic) |

---

## Category 1: Orbit Mechanics (Easily Simplifiable)

**Argument:** Keplerian two-body + J2 captures 99.9% of timing for LEO.
Policies trained on simplified gravity transfer to high-fidelity with
negligible degradation. The RL agent cares about "target will be in view in
~47 seconds," not J4 coefficients.

**Space-sim status:** Already at this level. Two-body + J2 + exponential
atmospheric drag, four fixed-step integrators, SoA state layout. This is the
right fidelity for RL training — and already beyond what the prose argues is
necessary.

**Roadmap alignment:** P3 EGM2008 and P3 J4 are correctly positioned as
offline validation tools, not training-path upgrades. P2 third-body (Sun/Moon)
is the next orbital fidelity step with training relevance.

---

## Category 2: Power, Thermal, Data (Partially Simplifiable)

**Argument:** Linear battery models, binary eclipse, constant power draws
per mode, and linear data buffers transfer well to nonlinear counterparts.
Policies learn conservative behavior; safety margins cover the fidelity gap.

**Space-sim status:** None of these are modeled. This is deliberate — the
product direction targets loose-fidelity orbit-level RL. Power/thermal/data
are mission-specific and belong at the scenario/environment layer, not in the
physics core.

**Open question:** Does the RL training pipeline need *any* subsystem
modeling, or is it sufficient to train on orbital dynamics alone and let
subsystem constraints be handled by downstream tools?

---

## Category 3: Attitude Dynamics (Hardest to Simplify)

**Argument:** Reaction wheel momentum buildup is path-dependent. A simplified
sim that lacks momentum concepts cannot represent the constraint, and policies
trained without it will learn infeasible behaviors (rapid back-and-forth
slewing that accumulates momentum faster than desaturation can handle).

**Proposed Phase 2 model:** `slew_time = f(angular_distance)`,
`desat_cost = fixed_cost`, no full reaction wheel integration. Stateful
momentum resource that accumulates with slew angle and depletes with
desaturation burns.

**Space-sim status:** Not modeled. 3DOF point mass only. No attitude state,
no slew constraints, no reaction wheels, no desaturation.

**Open question:** Is a simplified attitude model necessary for policies that
will transfer to higher-fidelity tools (Basilisk, GMAT), or can the orbital
dynamics alone produce transferable behavior? If needed, at what priority
level does it belong relative to the current P1 (low thrust) and P2 (rewards,
multi-step horizon)?

---

## GPU Parallelism: Warp vs Hand-Written CUDA vs Rust GPU

Three approaches to GPU-accelerated batch simulation were discussed:

### NVIDIA Warp

- Python framework that JIT-compiles to CUDA kernels
- Built-in vector/quaternion types
- Automatic differentiation (`wp.Tape`)
- `wp.launch(kernel, dim=N)` for parallel execution
- Mature and maintained by NVIDIA
- Constraint: Python interpreter overhead, JIT compilation step, CPU↔GPU copies
  across the Python/Rust boundary

### Hand-Written CUDA (Rust FFI)

- Maximum control over memory layout and kernel scheduling
- Rust-side SoA buffers map directly to device memory
- No Python dependency, no JIT overhead
- Constraint: kernel code is in CUDA C++, not Rust; two-language maintenance
- Space-sim roadmap item: P3 CUDA backend prototype

### Rust GPU Toolchain (cuda-oxide / rust-cuda)

- Write kernels in Rust, compile to PTX
- Single-language codebase, Rust safety in kernels
- Constraint: experimental toolchain, API instability, smaller ecosystem
- Space-sim roadmap item: P3 Rust GPU toolchain evaluation

**Current vast position:** Designed for `step_many` over SoA buffers. The P2
batched backend abstraction creates the boundary where any of these three
approaches can be plugged in. The P1 priority is landing the CPU batch path
first.

**Open question:** Which GPU approach is optimal? Trade-offs involve
development speed (Warp wins), performance ceiling (hand-written CUDA wins),
and codebase coherence (Rust GPU wins). This decision is not urgent — the CPU
batch path and RL training loop come first.

---

## Differentiability

**Argument for:** A differentiable sim enables analytic policy gradients,
model-based RL, and direct trajectory optimization. Instead of millions of
black-box PPO rollouts, thousands of gradient-based trajectories could suffice.

**Argument against (for spacecraft tasking specifically):** The tasking
problem has a fundamentally discrete decision space ("image target 3 now, or
target 7, or charge?"). Gradients through the dynamics don't help with
discrete choices — the policy gradient is estimated from returns, not dynamics
gradients. Differentiability would help more for continuous-control problems
like low-thrust trajectory optimization.

**Space-sim status:** No differentiable path. Rust's AD ecosystem is immature
(experimental LLVM-based tools like `enzyme` exist but aren't production-ready).

**Open question:** Does the benefit of differentiability justify the
complexity for our problem domain? For continuous control (low thrust,
station-keeping) it's compelling. For discrete tasking, PPO-style policy
gradients are likely sufficient.

---

## Action Spaces: Continuous vs Discrete

**Basilisk/bsk_rl approach:** The RL agent selects a flight software mode
(CHARGE, IMAGE, DESAT, DOWNLINK) — a discrete action space.

**Space-sim current approach:** `Action = [f64; 3]` — continuous inertial
thrust acceleration. Appropriate for low-thrust trajectory control. Issue:
for satellite tasking, the natural action is "which target do I image?" or
"which mode do I enter?" — both discrete.

**Open question:** Should the `Environment` trait support both continuous
and discrete action spaces (generic `Action` associated type), or should
discrete tasking be modeled as a scenario-layer concern that maps discrete
decisions to continuous thrust commands?

---

## What Space-Sim Already Has vs What the Prose Proposes

### Aligned (already implemented)

- Two-body + J2 gravity (the prose's Category 1 target)
- Compositional force models (`CompositeDynamics`)
- Fixed-step, allocation-free integrators (4 variants)
- SoA state layout (designed for GPU coalescing)
- `Environment` trait with `reset(seed)` / `step(action)`
- TOML scenario configuration
- Run recording (JSONL trajectory + metrics)
- Web replay UI (React + R3F)
- Mission segment infrastructure (propagate + burn sequences)

### Missing (relative to the medium-fidelity sim described in the prose)

- Controlled low thrust (P1 on roadmap)
- Multi-step RL horizon (P2 on roadmap)
- Reward functions (P2 on roadmap)
- Seeded IC perturbations (P2 on roadmap)
- Simplified attitude model (not on roadmap — **open question**)
- Target access geometry (not on roadmap — scenario-layer concern)
- Sun position model (needed for P2 SRP; not yet needed for training)

### Deliberately out of scope (for loose-fidelity training)

- Power/thermal subsystem models
- Data buffer and downlink models
- Full reaction wheel integration
- High-fidelity gravity (EGM2008)
- SPICE ephemeris

---

## Sanity Check: Is Space-Sim's Architecture Right?

The prose argues for a three-phase fidelity curriculum:

1. Kepler orbits, no attitude → rapid iteration
2. Add simplified attitude → refine for feasibility
3. Validate against high-fidelity sim

Space-sim's layered architecture maps cleanly onto this:
- **Phase 1** = today + P1 low thrust + P2 rewards/horizon/perturbations
- **Phase 2** = add simplified attitude model at the environment layer
- **Phase 3** = validate policies against external tools (GMAT/Basilisk)

No architectural changes are needed to support this progression. The fidelity
is a configurable dial, not a binary choice.

---

## Open Questions

These are explicitly unresolved and should not be treated as settled:

1. **GPU approach:** Warp vs hand-written CUDA vs Rust GPU toolchain.
   Decision deferred until the CPU batch backend (P2) is stable.

2. **Attitude model priority:** Does a simplified attitude model belong at P2
   (alongside rewards and multi-step horizon) or P3 (after batch backend)?

3. **Action space type:** Should the `Environment` trait use an associated
   `Action` type (supporting both continuous thrust vectors and discrete mode
   selections), or keep `[f64; 3]` and handle discrete modes at a higher layer?

4. **Subsystem modeling:** Does RL training need *any* power/data/thermal
   modeling, or is orbital dynamics alone sufficient for the training loop?

5. **Differentiability:** Is it worth pursuing (P3 at most) given the discrete
   nature of the spacecraft tasking problem and Rust's immature AD ecosystem?

6. **Validation strategy:** How do we determine whether a vast-trained
   policy transfers to higher-fidelity tools? What metrics constitute "close
   enough"?

---

## Related Documents

- [ROADMAP.md](../ROADMAP.md) — prioritized unimplemented work
- [architecture.md](architecture.md) — layered simulator model
