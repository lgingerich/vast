use crate::errors::PropagationError;
use crate::physics::diagnostics::OrbitalDiagnostics;
use crate::physics::dynamics::Dynamics;
use crate::physics::elements::{OrbitalElementClassification, OrbitalElements};
use crate::physics::integrators::Integrator;
use crate::physics::orbits::Orbit;
use crate::physics::state::{CartesianState, CartesianStateBuffer};
use crate::physics::types::{Duration, Epoch, GravitationalParameter, TimeStep};

/// Propagates a single-body state for a fixed duration with a given
/// dynamics/integrator pair.
///
/// `Sim` owns the state buffer and a while-loop step control flow.
/// It answers "where is this state after N seconds?" without knowing
/// about termination conditions, sampling cadence, or artifact recording.
///
/// The crate-private [`Propagator`](crate::sim) builds on the same
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
/// let orbit: EllipticOrbit = Orbit::CircularInclined {
///     altitude: Distance::from_kilometers(6678.0)?,
///     inclination: Angle::from_degrees(51.6)?,
///     raan: Angle::from_degrees(32.0)?,
/// }.try_into()?;
/// let period = orbit.period()?;
/// let result = SimBuilder::new()
///     .orbit(Orbit::CircularInclined {
///         altitude: Distance::from_kilometers(6678.0)?,
///         inclination: Angle::from_degrees(51.6)?,
///         raan: Angle::from_degrees(32.0)?,
///     })?
///     .dynamics(TwoBodyGravity::earth())
///     .integrator(VelocityVerletIntegrator)
///     .timestep(TimeStep::from_seconds(10.0)?)
///     .duration(Duration::from_seconds(period.seconds() * 3.0).unwrap())?;
/// # Ok(())
/// # }
/// ```
pub struct SimBuilder<D = (), I = ()> {
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
    /// Two-body diagnostics at the start of propagation.
    pub initial_diagnostics: OrbitalDiagnostics,
    /// Two-body diagnostics computed from the final state.
    pub diagnostics: OrbitalDiagnostics,
    /// Bound orbital elements at the start of propagation.
    pub initial_elements: OrbitalElements,
    /// Bound orbital elements computed from the final state.
    pub elements: OrbitalElements,
    /// Elapsed simulation time since the builder epoch.
    pub elapsed: Duration,
}

impl SimBuilder {
    /// Creates a new builder with nothing configured yet.
    pub fn new() -> Self {
        Self {
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
    /// Converts the spec into a validated [`EllipticOrbit`], then into
    /// the initial [`CartesianState`].
    pub fn orbit(mut self, spec: Orbit) -> Result<Self, PropagationError> {
        let elliptic = crate::physics::orbits::EllipticOrbit::try_from(spec)?;
        let state = elliptic.to_cartesian_state()?;
        self.state = Some(state);
        Ok(self)
    }

    /// Sets the dynamics model.
    pub fn dynamics<NewD: Dynamics>(self, dynamics: NewD) -> SimBuilder<NewD, I> {
        SimBuilder {
            state: self.state,
            dynamics: Some(dynamics),
            integrator: self.integrator,
            dt: self.dt,
        }
    }

    /// Sets the time integrator.
    pub fn integrator<NewI: Integrator>(self, integrator: NewI) -> SimBuilder<D, NewI> {
        SimBuilder {
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
        let mu = GravitationalParameter::earth();

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
            let step = TimeStep::from_seconds(step_s)?;
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

    /// Reads the current body state at index 0.
    pub fn state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.view().read_state(0)?)
    }

    /// Computes two-body diagnostics from the current state.
    pub fn diagnostics(&self) -> Result<OrbitalDiagnostics, crate::errors::StateError> {
        OrbitalDiagnostics::from_states(&self.states.view(), 0, self.mu)
    }

    /// Returns the current simulation epoch.
    pub const fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// Returns the number of integrator steps taken so far.
    pub const fn steps(&self) -> usize {
        self.steps
    }

    /// Returns the two-body diagnostics computed at the start of propagation.
    pub const fn initial_diagnostics(&self) -> &OrbitalDiagnostics {
        &self.initial_diagnostics
    }

    /// Snapshot of the current state, diagnostics, and orbital elements.
    pub fn snapshot(&self) -> Result<SimResult, PropagationError> {
        let final_state = self.state()?;
        let diagnostics = OrbitalDiagnostics::from_state(&final_state, self.mu)?;
        let elements = OrbitalElements::from_state(
            &final_state,
            self.mu,
            OrbitalElementClassification::default(),
        )?;
        let elapsed = self.epoch.duration_since(self.start_epoch);
        Ok(SimResult {
            final_state,
            final_epoch: self.epoch,
            steps: self.steps,
            initial_diagnostics: self.initial_diagnostics,
            diagnostics,
            initial_elements: self.initial_elements,
            elements,
            elapsed,
        })
    }
}

impl Default for SimBuilder {
    fn default() -> Self {
        Self::new()
    }
}
