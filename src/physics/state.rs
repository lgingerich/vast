use nalgebra::Vector3;

pub use super::types::{Position, Velocity};
use crate::errors::{NumericalError, StateError};

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

/// Position components in an inertial Cartesian frame, measured in meters.
#[derive(Debug, Clone, Copy)]
pub struct CartesianPositionComponents {
    x_m: f64,
    y_m: f64,
    z_m: f64,
}

impl CartesianPositionComponents {
    /// Creates finite Cartesian position components from meters.
    pub fn from_meters(x_m: f64, y_m: f64, z_m: f64) -> Result<Self, NumericalError> {
        if components_are_finite(x_m, y_m, z_m) {
            Ok(Self { x_m, y_m, z_m })
        } else {
            Err(NumericalError::NotFinite {
                field: "position components",
            })
        }
    }

    fn from_position(position: Position) -> Self {
        let v = position.vector();
        debug_assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
        Self {
            x_m: v.x,
            y_m: v.y,
            z_m: v.z,
        }
    }

    /// Returns the x component in meters.
    #[inline]
    pub const fn x_meters(self) -> f64 {
        self.x_m
    }

    /// Returns the y component in meters.
    #[inline]
    pub const fn y_meters(self) -> f64 {
        self.y_m
    }

    /// Returns the z component in meters.
    #[inline]
    pub const fn z_meters(self) -> f64 {
        self.z_m
    }

    pub(crate) fn vector(self) -> Vector3<f64> {
        Vector3::new(self.x_m, self.y_m, self.z_m)
    }

    fn to_position(self) -> Result<Position, StateError> {
        Ok(Position::from_vector(self.vector())?)
    }
}

/// Velocity components in an inertial Cartesian frame, measured in meters per second.
#[derive(Debug, Clone, Copy)]
pub struct CartesianVelocityComponents {
    x_mps: f64,
    y_mps: f64,
    z_mps: f64,
}

impl CartesianVelocityComponents {
    /// Creates finite Cartesian velocity components from meters per second.
    pub fn from_meters_per_second(
        x_mps: f64,
        y_mps: f64,
        z_mps: f64,
    ) -> Result<Self, NumericalError> {
        if components_are_finite(x_mps, y_mps, z_mps) {
            Ok(Self {
                x_mps,
                y_mps,
                z_mps,
            })
        } else {
            Err(NumericalError::NotFinite {
                field: "velocity components",
            })
        }
    }

    fn from_velocity(velocity: Velocity) -> Self {
        let v = velocity.vector();
        debug_assert!(v.x.is_finite() && v.y.is_finite() && v.z.is_finite());
        Self {
            x_mps: v.x,
            y_mps: v.y,
            z_mps: v.z,
        }
    }

    /// Returns the x component in meters per second.
    #[inline]
    pub const fn x_meters_per_second(self) -> f64 {
        self.x_mps
    }

    /// Returns the y component in meters per second.
    #[inline]
    pub const fn y_meters_per_second(self) -> f64 {
        self.y_mps
    }

    /// Returns the z component in meters per second.
    #[inline]
    pub const fn z_meters_per_second(self) -> f64 {
        self.z_mps
    }

    pub(crate) fn vector(self) -> Vector3<f64> {
        Vector3::new(self.x_mps, self.y_mps, self.z_mps)
    }

    fn to_velocity(self) -> Result<Velocity, StateError> {
        Ok(Velocity::from_vector(self.vector())?)
    }
}

/// Acceleration components in an inertial Cartesian frame, measured in meters per second squared.
#[derive(Debug, Clone, Copy)]
pub struct CartesianAccelerationComponents {
    x_mps2: f64,
    y_mps2: f64,
    z_mps2: f64,
}

impl CartesianAccelerationComponents {
    /// Creates finite Cartesian acceleration components from meters per second squared.
    pub fn from_meters_per_second_squared(
        x_mps2: f64,
        y_mps2: f64,
        z_mps2: f64,
    ) -> Result<Self, NumericalError> {
        if components_are_finite(x_mps2, y_mps2, z_mps2) {
            Ok(Self {
                x_mps2,
                y_mps2,
                z_mps2,
            })
        } else {
            Err(NumericalError::NotFinite {
                field: "acceleration components",
            })
        }
    }

    /// Returns the x component in meters per second squared.
    #[inline]
    pub const fn x_meters_per_second_squared(self) -> f64 {
        self.x_mps2
    }

    /// Returns the y component in meters per second squared.
    #[inline]
    pub const fn y_meters_per_second_squared(self) -> f64 {
        self.y_mps2
    }

    /// Returns the z component in meters per second squared.
    #[inline]
    pub const fn z_meters_per_second_squared(self) -> f64 {
        self.z_mps2
    }

    pub(crate) fn vector(self) -> Vector3<f64> {
        Vector3::new(self.x_mps2, self.y_mps2, self.z_mps2)
    }
}

/// Position and velocity components for one Cartesian state.
#[derive(Debug, Clone, Copy)]
pub struct CartesianStateComponents {
    /// Position components.
    pub position: CartesianPositionComponents,
    /// Velocity components.
    pub velocity: CartesianVelocityComponents,
}

impl CartesianStateComponents {
    /// Creates Cartesian state components from typed position and velocity.
    #[inline]
    pub fn from_state(state: &CartesianState) -> Self {
        Self {
            position: CartesianPositionComponents::from_position(state.position),
            velocity: CartesianVelocityComponents::from_velocity(state.velocity),
        }
    }
}

/// Fixed-capacity storage for many Cartesian states.
///
/// Internally this uses a structure-of-arrays layout for cache-friendly batched
/// stepping while keeping typed state conversions at API boundaries.
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
    /// Creates a buffer with all capacity active and initialized to finite zero components.
    #[inline]
    pub const fn new() -> Self {
        Self {
            pos_x: [0.0; N],
            pos_y: [0.0; N],
            pos_z: [0.0; N],
            vel_x: [0.0; N],
            vel_y: [0.0; N],
            vel_z: [0.0; N],
            len: N,
        }
    }

    /// Creates a buffer with no active bodies.
    #[inline]
    pub const fn empty() -> Self {
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
        let mut buffer = Self::empty();
        buffer.set_len(len)?;
        Ok(buffer)
    }

    /// Returns active body count.
    #[inline]
    pub const fn body_count(&self) -> usize {
        self.len
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

    /// Writes a typed Cartesian state into the active body at `index`.
    pub fn write_from_state(
        &mut self,
        index: usize,
        state: &CartesianState,
    ) -> Result<(), StateError> {
        self.view_mut()
            .write_components(index, CartesianStateComponents::from_state(state))
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
    /// Creates an immutable view from equal-length component slices.
    pub fn from_slices(
        pos_x: &'a [f64],
        pos_y: &'a [f64],
        pos_z: &'a [f64],
        vel_x: &'a [f64],
        vel_y: &'a [f64],
        vel_z: &'a [f64],
    ) -> Result<Self, StateError> {
        if !slice_lengths_match(
            pos_x.len(),
            pos_y.len(),
            pos_z.len(),
            vel_x.len(),
            vel_y.len(),
            vel_z.len(),
        ) {
            return Err(StateError::MismatchedSliceLengths);
        }

        Ok(Self {
            pos_x,
            pos_y,
            pos_z,
            vel_x,
            vel_y,
            vel_z,
        })
    }

    /// Returns active body count.
    #[inline]
    pub const fn body_count(&self) -> usize {
        self.pos_x.len()
    }

    /// Reads position for the active body at `index`.
    pub fn position(&self, index: usize) -> Result<Position, StateError> {
        self.components(index)?.position.to_position()
    }

    /// Reads velocity for the active body at `index`.
    pub fn velocity(&self, index: usize) -> Result<Velocity, StateError> {
        self.components(index)?.velocity.to_velocity()
    }

    /// Reads the active body at `index` into a typed Cartesian state.
    pub fn read_state(&self, index: usize) -> Result<CartesianState, StateError> {
        let components = self.components(index)?;
        Ok(CartesianState::new(
            components.position.to_position()?,
            components.velocity.to_velocity()?,
        ))
    }

    pub(crate) fn components(&self, index: usize) -> Result<CartesianStateComponents, StateError> {
        self.validate_index(index)?;
        Ok(CartesianStateComponents {
            position: CartesianPositionComponents::from_meters(
                self.pos_x[index],
                self.pos_y[index],
                self.pos_z[index],
            )?,
            velocity: CartesianVelocityComponents::from_meters_per_second(
                self.vel_x[index],
                self.vel_y[index],
                self.vel_z[index],
            )?,
        })
    }

    fn validate_index(&self, index: usize) -> Result<(), StateError> {
        if index >= self.body_count() {
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
    /// Creates a mutable view from equal-length component slices.
    pub fn from_slices(
        pos_x: &'a mut [f64],
        pos_y: &'a mut [f64],
        pos_z: &'a mut [f64],
        vel_x: &'a mut [f64],
        vel_y: &'a mut [f64],
        vel_z: &'a mut [f64],
    ) -> Result<Self, StateError> {
        if !slice_lengths_match(
            pos_x.len(),
            pos_y.len(),
            pos_z.len(),
            vel_x.len(),
            vel_y.len(),
            vel_z.len(),
        ) {
            return Err(StateError::MismatchedSliceLengths);
        }

        Ok(Self {
            pos_x,
            pos_y,
            pos_z,
            vel_x,
            vel_y,
            vel_z,
        })
    }

    /// Returns active body count.
    #[inline]
    pub const fn body_count(&self) -> usize {
        self.pos_x.len()
    }

    /// Reads the active body at `index` into a typed Cartesian state.
    pub fn read_state(&self, index: usize) -> Result<CartesianState, StateError> {
        self.as_view().read_state(index)
    }

    /// Writes a typed Cartesian state into the active body at `index`.
    pub fn write_from_state(
        &mut self,
        index: usize,
        state: &CartesianState,
    ) -> Result<(), StateError> {
        self.write_components(index, CartesianStateComponents::from_state(state))
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

    pub(crate) fn components(&self, index: usize) -> Result<CartesianStateComponents, StateError> {
        self.as_view().components(index)
    }

    pub(crate) fn write_components(
        &mut self,
        index: usize,
        components: CartesianStateComponents,
    ) -> Result<(), StateError> {
        self.validate_index(index)?;
        let position = components.position;
        let velocity = components.velocity;

        if !components_are_finite(position.x_m, position.y_m, position.z_m) {
            return Err(NumericalError::NotFinite { field: "position" }.into());
        }
        if !components_are_finite(velocity.x_mps, velocity.y_mps, velocity.z_mps) {
            return Err(NumericalError::NotFinite { field: "velocity" }.into());
        }

        self.pos_x[index] = position.x_m;
        self.pos_y[index] = position.y_m;
        self.pos_z[index] = position.z_m;
        self.vel_x[index] = velocity.x_mps;
        self.vel_y[index] = velocity.y_mps;
        self.vel_z[index] = velocity.z_mps;
        Ok(())
    }

    fn validate_index(&self, index: usize) -> Result<(), StateError> {
        if index >= self.body_count() {
            return Err(StateError::InvalidBodyIndex);
        }

        Ok(())
    }
}

fn components_are_finite(x: f64, y: f64, z: f64) -> bool {
    x.is_finite() && y.is_finite() && z.is_finite()
}

fn slice_lengths_match(
    pos_x_len: usize,
    pos_y_len: usize,
    pos_z_len: usize,
    vel_x_len: usize,
    vel_y_len: usize,
    vel_z_len: usize,
) -> bool {
    pos_x_len == pos_y_len
        && pos_x_len == pos_z_len
        && pos_x_len == vel_x_len
        && pos_x_len == vel_y_len
        && pos_x_len == vel_z_len
}

#[cfg(test)]
mod tests {
    use super::*;

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    fn state() -> Result<CartesianState, Box<dyn std::error::Error>> {
        Ok(CartesianState::new(
            Position::from_vector(Vector3::new(1.0, 2.0, 3.0))?,
            Velocity::from_vector(Vector3::new(4.0, 5.0, 6.0))?,
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
        let mut buffer = CartesianStateBuffer::<1>::new();
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
    fn state_views_reject_mismatched_slices() {
        let pos_x = [0.0, 1.0];
        let pos_y = [0.0];
        let pos_z = [0.0, 1.0];
        let vel_x = [0.0, 1.0];
        let vel_y = [0.0, 1.0];
        let vel_z = [0.0, 1.0];

        assert!(matches!(
            CartesianStates::from_slices(&pos_x, &pos_y, &pos_z, &vel_x, &vel_y, &vel_z),
            Err(StateError::MismatchedSliceLengths),
        ));
    }

    #[test]
    fn state_buffer_rejects_invalid_index() -> TestResult {
        let mut buffer = CartesianStateBuffer::<1>::new();

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
}
