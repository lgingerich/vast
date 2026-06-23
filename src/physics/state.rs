use nalgebra::Vector3;

use super::types::{Position, Velocity};
use crate::errors::StateError;

/// Position in meters (SI), Cartesian frame.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CartesianState {
    /// Position in meters (SI), Cartesian frame.
    pub position: Position,
    /// Velocity in meters per second (SI), Cartesian frame.
    pub velocity: Velocity,
}

impl CartesianState {
    /// Creates a Cartesian state from typed position and velocity.
    #[inline]
    pub const fn new(position: Position, velocity: Velocity) -> Self {
        Self { position, velocity }
    }
}

/// Position and velocity components for one Cartesian state.
///
/// Flat scalar storage lets dynamics models and integrators read components
/// directly in the hot path. Validation lives at the typed [`Position`] and
/// [`Velocity`] domain-type boundaries.
#[derive(Debug, Clone, Copy)]
pub struct CartesianStateComponents {
    px_m: f64,
    py_m: f64,
    pz_m: f64,
    vx_mps: f64,
    vy_mps: f64,
    vz_mps: f64,
}

impl CartesianStateComponents {
    pub(crate) fn from_vectors(position: Vector3<f64>, velocity: Vector3<f64>) -> Self {
        Self {
            px_m: position.x,
            py_m: position.y,
            pz_m: position.z,
            vx_mps: velocity.x,
            vy_mps: velocity.y,
            vz_mps: velocity.z,
        }
    }

    /// Returns the x position component in meters.
    #[inline]
    pub const fn px_meters(&self) -> f64 {
        self.px_m
    }

    /// Returns the y position component in meters.
    #[inline]
    pub const fn py_meters(&self) -> f64 {
        self.py_m
    }

    /// Returns the z position component in meters.
    #[inline]
    pub const fn pz_meters(&self) -> f64 {
        self.pz_m
    }

    /// Returns the x velocity component in meters per second.
    #[inline]
    pub const fn vx_meters_per_second(&self) -> f64 {
        self.vx_mps
    }

    /// Returns the y velocity component in meters per second.
    #[inline]
    pub const fn vy_meters_per_second(&self) -> f64 {
        self.vy_mps
    }

    /// Returns the z velocity component in meters per second.
    #[inline]
    pub const fn vz_meters_per_second(&self) -> f64 {
        self.vz_mps
    }

    /// Returns the position as a nalgebra vector.
    #[inline]
    pub fn position_vector(&self) -> Vector3<f64> {
        Vector3::new(self.px_m, self.py_m, self.pz_m)
    }

    /// Returns the velocity as a nalgebra vector.
    #[inline]
    pub fn velocity_vector(&self) -> Vector3<f64> {
        Vector3::new(self.vx_mps, self.vy_mps, self.vz_mps)
    }
}

impl From<&CartesianState> for CartesianStateComponents {
    fn from(state: &CartesianState) -> Self {
        Self::from_vectors(state.position.vector(), state.velocity.vector())
    }
}

impl From<(Position, Velocity)> for CartesianStateComponents {
    fn from((position, velocity): (Position, Velocity)) -> Self {
        Self::from_vectors(position.vector(), velocity.vector())
    }
}

impl From<CartesianStateComponents> for CartesianState {
    fn from(components: CartesianStateComponents) -> Self {
        Self::new(
            Position::from_vector(components.position_vector()),
            Velocity::from_vector(components.velocity_vector()),
        )
    }
}

/// Fixed-capacity storage for many Cartesian states.
///
/// Internally this uses a structure-of-arrays layout for cache-friendly batched
/// stepping while keeping typed state conversions at API boundaries.
#[derive(Debug)]
pub struct CartesianStateBuffer<const N: usize> {
    pos_x: [f64; N],
    pos_y: [f64; N],
    pos_z: [f64; N],
    vel_x: [f64; N],
    vel_y: [f64; N],
    vel_z: [f64; N],
    len: usize,
}

impl<const N: usize> CartesianStateBuffer<N> {
    // ---------------- Construction & Capacity ----------------

    /// Creates a buffer with no active bodies.
    #[inline]
    pub const fn new() -> Self {
        Self {
            pos_x: [0.0; N],
            pos_y: [0.0; N],
            pos_z: [0.0; N],
            vel_x: [0.0; N],
            vel_y: [0.0; N],
            vel_z: [0.0; N],
            len: 0,
        }
    }

    /// Creates a buffer with `len` active finite zero-initialized bodies.
    pub fn with_len(len: usize) -> Result<Self, StateError> {
        let mut buffer = Self::new();
        buffer.set_len(len)?;
        Ok(buffer)
    }

    /// Returns active body count.
    #[inline]
    pub const fn len(&self) -> usize {
        self.len
    }

    /// Returns true when no bodies are active.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// Returns maximum body capacity.
    #[inline]
    pub const fn capacity(&self) -> usize {
        N
    }

    /// Sets the active body count.
    pub fn set_len(&mut self, len: usize) -> Result<(), StateError> {
        if len > N {
            return Err(StateError::InvalidLength);
        }

        self.len = len;
        Ok(())
    }

    // ---------------- Views ----------------

    /// Returns an immutable borrowed state view.
    pub fn view(&self) -> CartesianStates<'_> {
        CartesianStates {
            pos_x: &self.pos_x[..self.len],
            pos_y: &self.pos_y[..self.len],
            pos_z: &self.pos_z[..self.len],
            vel_x: &self.vel_x[..self.len],
            vel_y: &self.vel_y[..self.len],
            vel_z: &self.vel_z[..self.len],
        }
    }

    /// Returns a mutable borrowed state view.
    pub fn view_mut(&mut self) -> CartesianStatesMut<'_> {
        CartesianStatesMut {
            pos_x: &mut self.pos_x[..self.len],
            pos_y: &mut self.pos_y[..self.len],
            pos_z: &mut self.pos_z[..self.len],
            vel_x: &mut self.vel_x[..self.len],
            vel_y: &mut self.vel_y[..self.len],
            vel_z: &mut self.vel_z[..self.len],
        }
    }

    // ---------------- Typed I/O ----------------

    /// Writes a typed Cartesian state into the active body at `index`.
    pub fn write_from_state(
        &mut self,
        index: usize,
        state: &CartesianState,
    ) -> Result<(), StateError> {
        self.view_mut()
            .write_components(index, CartesianStateComponents::from(state))
    }

    /// Reads the active body at `index` into a typed Cartesian state.
    pub fn read_state(&self, index: usize) -> Result<CartesianState, StateError> {
        self.view().read_state(index)
    }
}

impl<const N: usize> Default for CartesianStateBuffer<N> {
    fn default() -> Self {
        Self::new()
    }
}

/// Immutable borrowed view over many Cartesian states.
pub struct CartesianStates<'a> {
    pos_x: &'a [f64],
    pos_y: &'a [f64],
    pos_z: &'a [f64],
    vel_x: &'a [f64],
    vel_y: &'a [f64],
    vel_z: &'a [f64],
}

impl<'a> CartesianStates<'a> {
    /// Returns active body count.
    #[inline]
    pub const fn len(&self) -> usize {
        self.pos_x.len()
    }

    /// Returns true when this view contains no active bodies.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.pos_x.is_empty()
    }

    /// Reads the active body at `index` into a typed Cartesian state.
    pub fn read_state(&self, index: usize) -> Result<CartesianState, StateError> {
        Ok(CartesianState::from(self.components(index)?))
    }

    pub(crate) fn components(&self, index: usize) -> Result<CartesianStateComponents, StateError> {
        self.validate_index(index)?;
        Ok(CartesianStateComponents {
            px_m: self.pos_x[index],
            py_m: self.pos_y[index],
            pz_m: self.pos_z[index],
            vx_mps: self.vel_x[index],
            vy_mps: self.vel_y[index],
            vz_mps: self.vel_z[index],
        })
    }

    fn validate_index(&self, index: usize) -> Result<(), StateError> {
        if index >= self.len() {
            return Err(StateError::InvalidBodyIndex);
        }

        Ok(())
    }
}

/// Mutable borrowed view over many Cartesian states.
pub struct CartesianStatesMut<'a> {
    pos_x: &'a mut [f64],
    pos_y: &'a mut [f64],
    pos_z: &'a mut [f64],
    vel_x: &'a mut [f64],
    vel_y: &'a mut [f64],
    vel_z: &'a mut [f64],
}

impl<'a> CartesianStatesMut<'a> {
    /// Returns active body count.
    #[inline]
    pub const fn len(&self) -> usize {
        self.pos_x.len()
    }

    /// Returns true when this view contains no active bodies.
    #[inline]
    pub const fn is_empty(&self) -> bool {
        self.pos_x.is_empty()
    }

    /// Returns an immutable view over the same active bodies.
    pub fn as_view(&self) -> CartesianStates<'_> {
        CartesianStates {
            pos_x: self.pos_x,
            pos_y: self.pos_y,
            pos_z: self.pos_z,
            vel_x: self.vel_x,
            vel_y: self.vel_y,
            vel_z: self.vel_z,
        }
    }

    pub(crate) fn write_components(
        &mut self,
        index: usize,
        components: CartesianStateComponents,
    ) -> Result<(), StateError> {
        self.validate_index(index)?;
        self.pos_x[index] = components.px_m;
        self.pos_y[index] = components.py_m;
        self.pos_z[index] = components.pz_m;
        self.vel_x[index] = components.vx_mps;
        self.vel_y[index] = components.vy_mps;
        self.vel_z[index] = components.vz_mps;
        Ok(())
    }

    fn validate_index(&self, index: usize) -> Result<(), StateError> {
        if index >= self.len() {
            return Err(StateError::InvalidBodyIndex);
        }

        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    fn state() -> Result<CartesianState, Box<dyn std::error::Error>> {
        Ok(CartesianState::new(
            Position::from_vector(Vector3::new(1.0, 2.0, 3.0)),
            Velocity::from_vector(Vector3::new(4.0, 5.0, 6.0)),
        ))
    }

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() <= f64::EPSILON,
            "expected {actual} to equal {expected}",
        );
    }

    #[test]
    fn state_buffer_roundtrips_cartesian_state() -> TestResult {
        let mut buffer = CartesianStateBuffer::<1>::with_len(1)?;
        let state = state()?;

        buffer.write_from_state(0, &state)?;
        let read = buffer.read_state(0)?;

        assert_close(read.position.vector().x, 1.0);
        assert_close(read.position.vector().y, 2.0);
        assert_close(read.position.vector().z, 3.0);
        assert_close(read.velocity.vector().x, 4.0);
        assert_close(read.velocity.vector().y, 5.0);
        assert_close(read.velocity.vector().z, 6.0);
        Ok(())
    }

    #[test]
    fn state_buffer_rejects_invalid_length() {
        assert!(matches!(
            CartesianStateBuffer::<1>::with_len(2),
            Err(StateError::InvalidLength),
        ));
    }

    #[test]
    fn state_buffer_rejects_invalid_index() -> TestResult {
        let mut buffer = CartesianStateBuffer::<1>::with_len(1)?;

        assert!(matches!(
            buffer.write_from_state(1, &state()?),
            Err(StateError::InvalidBodyIndex),
        ));
        assert!(matches!(
            buffer.read_state(1),
            Err(StateError::InvalidBodyIndex),
        ));
        Ok(())
    }

    #[test]
    fn cartesian_state_new_holds_values() -> TestResult {
        let state = CartesianState::new(
            Position::from_vector(Vector3::new(1.0, 2.0, 3.0)),
            Velocity::from_vector(Vector3::new(4.0, 5.0, 6.0)),
        );

        assert_close(state.position.vector().x, 1.0);
        assert_close(state.position.vector().y, 2.0);
        assert_close(state.position.vector().z, 3.0);
        assert_close(state.velocity.vector().x, 4.0);
        assert_close(state.velocity.vector().y, 5.0);
        assert_close(state.velocity.vector().z, 6.0);
        Ok(())
    }
}
