use crate::errors::PropagationError;
use crate::physics::dynamics::Dynamics;
use crate::physics::frame::Frame;
use crate::physics::integrators::Integrator;
use crate::physics::orbits::Orbit;
use crate::physics::state::{CartesianState, CartesianStateBuffer};
use crate::physics::types::{Duration, Epoch, TimeStep};

/// Propagates a single-body state with a given dynamics/integrator pair.
///
/// `Sim` owns the state buffer and a while-loop step control flow. It answers
/// "where is this state after N seconds?" without knowing about termination
/// conditions, sampling cadence, or artifact recording. Analysis (orbital
/// elements) is computed by the caller via
/// [`crate::physics::elements::OrbitalElements::from_state`].
///
/// The crate-private `Propagator` builds on the same
/// [`Integrator::step`] trait method but adds stop-condition evaluation,
/// per-segment sampling, and burn application.
pub struct Sim<D: Dynamics, I: Integrator> {
    states: CartesianStateBuffer<1>,
    frame: Frame,
    dynamics: D,
    integrator: I,
    epoch: Epoch,
    steps: usize,
}

/// Builder for [`Sim`] where every configuration is a chainable method.
///
/// # Example
///
/// ```
/// # use vast::*;
/// # fn main() -> Result<(), Box<dyn std::error::Error>> {
/// let frame = Frame::earth_inertial();
/// let orbit = Orbit::circular_inclined(
///     Distance::new((6678.0) * 1000.0),
///     Angle::from_degrees(51.6),
///     Angle::from_degrees(32.0),
/// );
/// let mut sim = SimBuilder::new(frame)
///     .orbit(Orbit::circular_inclined(
///         Distance::new((6678.0) * 1000.0),
///         Angle::from_degrees(51.6),
///         Angle::from_degrees(32.0),
///     ))?
///     .dynamics(TwoBodyGravity::from_frame(&frame)?)
///     .integrator(VelocityVerletIntegrator)
///     .build()?;
/// sim.advance(Duration::new(60.0), TimeStep::new(10.0))?;
/// # Ok(())
/// # }
/// ```
pub struct SimBuilder<D = (), I = ()> {
    frame: Frame,
    state: Option<CartesianState>,
    dynamics: Option<D>,
    integrator: Option<I>,
}

impl SimBuilder {
    /// Creates a new builder configured for the given reference frame.
    ///
    /// The frame supplies the central body used when converting [`Orbit`]
    /// altitude specs into Cartesian state.
    pub fn new(frame: Frame) -> Self {
        Self {
            frame,
            state: None,
            dynamics: None,
            integrator: None,
        }
    }
}

impl<D, I> SimBuilder<D, I> {
    /// Sets the initial orbit from an [`Orbit`] specification.
    ///
    /// Converts the spec into the initial [`CartesianState`] around the
    /// builder's frame.
    pub fn orbit(mut self, spec: Orbit) -> Result<Self, PropagationError> {
        self.state = Some(spec.to_cartesian_state(&self.frame)?);
        Ok(self)
    }

    /// Sets the initial Cartesian state directly.
    pub fn state(mut self, state: CartesianState) -> Self {
        self.state = Some(state);
        self
    }

    /// Sets the dynamics model.
    pub fn dynamics<NewD: Dynamics>(self, dynamics: NewD) -> SimBuilder<NewD, I> {
        SimBuilder {
            frame: self.frame,
            state: self.state,
            dynamics: Some(dynamics),
            integrator: self.integrator,
        }
    }

    /// Sets the time integrator.
    pub fn integrator<NewI: Integrator>(self, integrator: NewI) -> SimBuilder<D, NewI> {
        SimBuilder {
            frame: self.frame,
            state: self.state,
            dynamics: self.dynamics,
            integrator: Some(integrator),
        }
    }
}

impl<D: Dynamics, I: Integrator> SimBuilder<D, I> {
    /// Writes the initial state into a buffer and returns a ready [`Sim`].
    pub fn build(mut self) -> Result<Sim<D, I>, PropagationError> {
        let state = self.state.take().ok_or(PropagationError::BuildIncomplete)?;
        let frame = self.frame;
        let dynamics = self
            .dynamics
            .take()
            .ok_or(PropagationError::BuildIncomplete)?;
        let integrator = self
            .integrator
            .take()
            .ok_or(PropagationError::BuildIncomplete)?;

        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        Ok(Sim {
            states,
            frame,
            dynamics,
            integrator,
            epoch: Epoch::ZERO,
            steps: 0,
        })
    }
}

impl<D: Dynamics, I: Integrator> Sim<D, I> {
    // ---------------- Stepping ----------------

    /// Advances the simulation forward by `duration` using a fixed time step `dt`.
    ///
    /// The final step may be shorter than `dt` to land exactly on the
    /// requested duration.
    pub fn advance(&mut self, duration: Duration, dt: TimeStep) -> Result<(), PropagationError> {
        let target = self.epoch.seconds() + duration.seconds();
        while self.epoch.seconds() < target {
            let remaining = target - self.epoch.seconds();
            let step_s = dt.seconds().min(remaining);
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
        Ok(())
    }

    /// Advances the simulation by a single step of the given time step `dt`.
    pub fn step(&mut self, dt: TimeStep) -> Result<(), PropagationError> {
        self.integrator.step(
            &self.dynamics,
            &mut self.states.view_mut(),
            0,
            self.epoch,
            dt,
        )?;
        self.epoch = self.epoch.advance_by(dt);
        self.steps += 1;
        Ok(())
    }

    // ---------------- Queries ----------------

    /// Reads the current body state at index 0.
    pub fn state(&self) -> Result<CartesianState, PropagationError> {
        Ok(self.states.view().read_state(0)?)
    }

    /// Returns the current simulation epoch.
    pub const fn epoch(&self) -> Epoch {
        self.epoch
    }

    /// Returns the frame in which the simulation state is expressed.
    pub const fn frame(&self) -> Frame {
        self.frame
    }

    /// Returns the number of integrator steps taken so far.
    pub const fn steps(&self) -> usize {
        self.steps
    }
}
