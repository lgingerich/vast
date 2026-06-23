use crate::errors::PropagationError;
use crate::physics::body::CentralBody;
use crate::physics::diagnostics::{DiagnosticSeries, OrbitalDiagnostics};
use crate::physics::dynamics::Dynamics;
use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};
use crate::physics::frame::Frame;
use crate::physics::integrators::Integrator;
use crate::physics::orbits::Orbit;
use crate::physics::state::{CartesianState, CartesianStateBuffer};
use crate::physics::types::{Duration, Epoch, GravitationalParameter, TimeStep};

// new
#[expect(
    dead_code,
    reason = "Simulation is being staged; not yet wired to public builders."
)]
pub struct Simulation<D: Dynamics, I: Integrator> {
    states: CartesianStateBuffer<1>,
    frame: Frame,
    dynamics: D,
    integrator: I,
    epoch: Epoch,
    steps: usize,
}

/*

Orbit + Frame  --(Orbit::to_cartesian_state)-->  CartesianState  --(builder)-->  CartesianStateBuffer<1>  -->  Simulation

Simulation
 ├── Initial State
 ├── Frame
 |   ├── Origin
 |   └── Axes
 ├── Force Models 
 └── Integrator

 */

/// Propagates a single-body state for a fixed duration with a given
/// dynamics/integrator pair.
///
/// `Sim` owns the state buffer and a while-loop step control flow.
/// It answers "where is this state after N seconds?" without knowing
/// about termination conditions, sampling cadence, or artifact recording.
///
/// The crate-private `Propagator` builds on the same
/// [`Integrator::step`] trait method but adds stop-condition evaluation,
/// per-segment sampling, and burn application.
pub struct Sim<D: Dynamics, I: Integrator> {
    states: CartesianStateBuffer<1>,
    epoch: Epoch,
    dynamics: D,
    integrator: I,
    dt: TimeStep,
    mu: GravitationalParameter,
    start_epoch: Epoch,
    steps: usize,
    initial_diagnostics: OrbitalDiagnostics,
    initial_elements: OrbitalElements,
}

/// Builder for [`Sim`] where every configuration is a chainable method.
///
/// # Example
///
/// ```
/// # use vast::*;
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let body = CentralBody::earth()?;
/// let orbit = Orbit::circular_inclined(
///     Distance::new((6678.0) * 1000.0),
///     Angle::from_degrees(51.6),
///     Angle::from_degrees(32.0),
/// );
/// let period = orbit.period(&body)?;
/// let result = SimBuilder::new(body)
///     .orbit(Orbit::circular_inclined(
///         Distance::new((6678.0) * 1000.0),
///         Angle::from_degrees(51.6),
///         Angle::from_degrees(32.0),
///     ))?
///     .dynamics(TwoBodyGravity::from_body(&body))
///     .integrator(VelocityVerletIntegrator)
///     .timestep(TimeStep::new(10.0))
///     .duration(Duration::new(period.seconds() * 3.0))?;
/// # Ok(())
/// # }
/// ```
pub struct SimBuilder<D = (), I = ()> {
    body: CentralBody,
    state: Option<CartesianState>,
    dynamics: Option<D>,
    integrator: Option<I>,
    dt: Option<TimeStep>,
}

/// Snapshot produced after propagation or taken on demand from a [`Sim`].
#[derive(Debug, Clone, PartialEq)]
pub struct SimResult {
    /// State after the final step.
    pub final_state: CartesianState,
    /// Epoch after the final step.
    pub final_epoch: Epoch,
    /// Number of integrator steps taken.
    pub steps: usize,
    /// Diagnostic scalars across the trajectory (index 0 = initial, index 1 = final).
    pub series: DiagnosticSeries,
    /// Bound orbital elements at the start of propagation.
    pub initial_elements: OrbitalElements,
    /// Bound orbital elements computed from the final state.
    pub elements: OrbitalElements,
    /// Elapsed simulation time since the builder epoch.
    pub elapsed: Duration,
}

impl SimBuilder {
    /// Creates a new builder configured for the given central body.
    ///
    /// The body supplies the gravitational parameter used for diagnostics and
    /// orbital-element extraction, and the equatorial radius used when
    /// converting [`Orbit`] altitude specs into Cartesian state.
    pub fn new(body: CentralBody) -> Self {
        Self {
            body,
            state: None,
            dynamics: None,
            integrator: None,
            dt: None,
        }
    }
}

impl<D, I> SimBuilder<D, I> {
    /// Sets the initial orbit from an [`Orbit`] specification.
    ///
    /// Converts the spec into the initial [`CartesianState`] around the
    /// builder's central body.
    pub fn orbit(mut self, spec: Orbit) -> Result<Self, PropagationError> {
        let state = spec.to_cartesian_state(&self.body)?;
        self.state = Some(state);
        Ok(self)
    }

    /// Sets the dynamics model.
    pub fn dynamics<NewD: Dynamics>(self, dynamics: NewD) -> SimBuilder<NewD, I> {
        SimBuilder {
            body: self.body,
            state: self.state,
            dynamics: Some(dynamics),
            integrator: self.integrator,
            dt: self.dt,
        }
    }

    /// Sets the time integrator.
    pub fn integrator<NewI: Integrator>(self, integrator: NewI) -> SimBuilder<D, NewI> {
        SimBuilder {
            body: self.body,
            state: self.state,
            dynamics: self.dynamics,
            integrator: Some(integrator),
            dt: self.dt,
        }
    }

    /// Sets the fixed integration time step.
    pub fn timestep(mut self, dt: TimeStep) -> Self {
        self.dt = Some(dt);
        self
    }
}

impl<D: Dynamics, I: Integrator> SimBuilder<D, I> {
    /// Writes the initial state into a buffer and returns a ready [`Sim`].
    pub fn build(mut self) -> Result<Sim<D, I>, PropagationError> {
        let state = self.state.take().ok_or(PropagationError::BuildIncomplete)?;
        let dt = self.dt.ok_or(PropagationError::BuildIncomplete)?;
        let dynamics = self
            .dynamics
            .take()
            .ok_or(PropagationError::BuildIncomplete)?;
        let integrator = self
            .integrator
            .take()
            .ok_or(PropagationError::BuildIncomplete)?;
        let start_epoch = Epoch::ZERO;
        let mu = self.body.mu();

        let initial_diagnostics = OrbitalDiagnostics::from_state(&state, mu)?;
        let initial_elements =
            OrbitalElements::from_state(&state, mu, OrbitalElementClassification::default())?;

        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        Ok(Sim {
            states,
            epoch: start_epoch,
            dynamics,
            integrator,
            dt,
            mu,
            start_epoch,
            steps: 0,
            initial_diagnostics,
            initial_elements,
        })
    }

    /// Convenience: builds the [`Sim`], advances for `duration`, and returns
    /// a [`SimResult`].
    ///
    /// # Errors
    ///
    /// Returns [`PropagationError`] if the builder is incomplete, the
    /// initial state is invalid, or propagation fails.
    pub fn duration(self, duration: Duration) -> Result<SimResult, PropagationError> {
        let mut sim = self.build()?;
        sim.advance(duration)?;
        sim.snapshot()
    }
}

impl<D: Dynamics, I: Integrator> Sim<D, I> {
    /// Advances the simulation forward by `duration`.
    ///
    /// The final step may be shorter than the configured time step
    /// to land exactly on the requested duration.
    pub fn advance(&mut self, duration: Duration) -> Result<&mut Self, PropagationError> {
        let target = self.epoch.seconds() + duration.seconds();
        while self.epoch.seconds() < target {
            let remaining = target - self.epoch.seconds();
            let step_s = self.dt.seconds().min(remaining);
            let step = TimeStep::new(step_s);
            self.integrator.step(
                &self.dynamics,
                &mut self.states.view_mut(),
                0,
                self.epoch,
                step,
            )?;
            self.epoch = self.epoch.advance_by(step);
            self.steps += 1;
        }
        Ok(self)
    }

    /// Advances the simulation by a single step of the configured time step `dt`.
    ///
    /// Returns the new state after the step.
    pub fn step(&mut self) -> Result<CartesianState, PropagationError> {
        self.integrator.step(
            &self.dynamics,
            &mut self.states.view_mut(),
            0,
            self.epoch,
            self.dt,
        )?;
        self.epoch = self.epoch.advance_by(self.dt);
        self.steps += 1;
        self.state()
    }

    /// Computes and returns the orbital diagnostics for the current state.
    pub fn diagnostics(&self) -> Result<OrbitalDiagnostics, PropagationError> {
        Ok(OrbitalDiagnostics::from_state(&self.state()?, self.mu)?)
    }

    /// Reads the current body state at index 0.
    pub fn state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.view().read_state(0)?)
    }

    /// Returns the current simulation epoch.
    pub const fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// Returns the number of integrator steps taken so far.
    pub const fn steps(&self) -> usize {
        self.steps
    }

    /// Snapshot of the current state, diagnostics, and orbital elements.
    pub fn snapshot(&self) -> Result<SimResult, PropagationError> {
        let final_state = self.state()?;
        let final_diagnostics = OrbitalDiagnostics::from_state(&final_state, self.mu)?;
        let elements = OrbitalElements::from_state(
            &final_state,
            self.mu,
            OrbitalElementClassification::default(),
        )?;
        let elapsed = self.epoch.duration_since(self.start_epoch);
        let series = DiagnosticSeries::from_pair(&self.initial_diagnostics, &final_diagnostics);
        Ok(SimResult {
            final_state,
            final_epoch: self.epoch,
            steps: self.steps,
            series,
            initial_elements: self.initial_elements,
            elements,
            elapsed,
        })
    }
}
