pub use super::state::CartesianStateComponents;
pub use super::types::{
    Acceleration, AngularRate, AreaPerMass, Distance, Epoch, GravitationalParameter, J2Coefficient,
    MassDensity,
};
use crate::errors::{DynamicsError, NumericalError, StateError};
use crate::physics::constants::{
    EARTH_ATMOSPHERE_REFERENCE_ALTITUDE_M, EARTH_ATMOSPHERE_REFERENCE_DENSITY_KG_M3,
    EARTH_ATMOSPHERE_SCALE_HEIGHT_M,
};
use crate::physics::frame::{BodyId, Frame};
use nalgebra::Vector3;

/// Point-mass gravity model for a single central body.
#[derive(Debug)]
pub struct TwoBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    mu: GravitationalParameter,
}

impl TwoBodyGravity {
    /// Creates a point-mass gravity model.
    #[inline]
    pub const fn new(mu: GravitationalParameter) -> Self {
        Self { mu }
    }

    /// Creates a point-mass gravity model from a frame's central body.
    pub fn from_frame(frame: &Frame) -> Result<Self, StateError> {
        frame.require_inertial_axes()?;
        Ok(Self { mu: frame.mu()? })
    }

    /// Returns the central body's gravitational parameter.
    #[inline]
    pub const fn mu(&self) -> GravitationalParameter {
        self.mu
    }
}

/// Central gravity model with analytical J2 perturbation.
#[derive(Debug)]
pub struct J2Gravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    mu: GravitationalParameter,
    /// Equatorial reference radius in meters.
    equatorial_radius: Distance,
    /// Dimensionless second zonal harmonic coefficient.
    j2: J2Coefficient,
}

impl J2Gravity {
    /// Creates a J2 gravity model.
    #[inline]
    pub const fn new(
        mu: GravitationalParameter,
        equatorial_radius: Distance,
        j2: J2Coefficient,
    ) -> Self {
        Self {
            mu,
            equatorial_radius,
            j2,
        }
    }

    /// Creates a J2 gravity model from a frame's central body.
    ///
    /// Returns [`NumericalError::ConstraintViolation`] if the frame's body has no J2
    /// coefficient (point-mass only).
    pub fn from_frame(frame: &Frame) -> Result<Self, StateError> {
        frame.require_inertial_axes()?;
        let Some(j2) = frame.j2()? else {
            return Err(StateError::Numerical(NumericalError::ConstraintViolation {
                field: "j2",
                reason: "frame central body has no J2 coefficient",
            }));
        };
        Ok(Self::new(
            frame.mu()?,
            Distance::new(frame.equatorial_radius()?.meters()),
            j2,
        ))
    }

    /// Returns the central body's gravitational parameter.
    #[inline]
    pub const fn mu(&self) -> GravitationalParameter {
        self.mu
    }

    /// Returns the equatorial reference radius.
    #[inline]
    pub const fn equatorial_radius(&self) -> Distance {
        self.equatorial_radius
    }

    /// Returns the J2 coefficient.
    #[inline]
    pub const fn j2(&self) -> J2Coefficient {
        self.j2
    }
}

/// Spherical exponential atmosphere rotating about the inertial z axis.
#[derive(Debug)]
pub struct ExponentialAtmosphere {
    /// Planet reference radius in meters.
    reference_radius: Distance,
    /// Reference altitude above `reference_radius`.
    reference_altitude: Distance,
    /// Density at `reference_altitude`.
    reference_density: MassDensity,
    /// Exponential scale height.
    scale_height: Distance,
    /// Planet angular rate about the inertial z axis.
    angular_rate: AngularRate,
}

impl ExponentialAtmosphere {
    /// Creates an exponential atmosphere.
    #[inline]
    pub const fn new(
        reference_radius: Distance,
        reference_altitude: Distance,
        reference_density: MassDensity,
        scale_height: Distance,
        angular_rate: AngularRate,
    ) -> Self {
        Self {
            reference_radius,
            reference_altitude,
            reference_density,
            scale_height,
            angular_rate,
        }
    }

    /// Creates an exponential atmosphere from a frame's central body.
    ///
    /// Returns [`NumericalError::ConstraintViolation`] if the frame's body has no
    /// atmosphere.
    pub fn from_frame(frame: &Frame) -> Result<Self, StateError> {
        frame.require_inertial_axes()?;
        let Some(body) = frame.central_body() else {
            return Err(StateError::InvalidFrame);
        };
        match body {
            BodyId::Earth => Ok(Self::new(
                Distance::new(frame.equatorial_radius()?.meters()),
                Distance::new(EARTH_ATMOSPHERE_REFERENCE_ALTITUDE_M),
                MassDensity::new(EARTH_ATMOSPHERE_REFERENCE_DENSITY_KG_M3),
                Distance::new(EARTH_ATMOSPHERE_SCALE_HEIGHT_M),
                AngularRate::new(frame.rotation_rate()?.radians_per_second()),
            )),
            BodyId::Moon => Err(StateError::Numerical(NumericalError::ConstraintViolation {
                field: "atmosphere",
                reason: "frame central body has no atmosphere",
            })),
        }
    }

    /// Returns the planet reference radius.
    #[inline]
    pub const fn reference_radius(&self) -> Distance {
        self.reference_radius
    }

    /// Returns the reference altitude.
    #[inline]
    pub const fn reference_altitude(&self) -> Distance {
        self.reference_altitude
    }

    /// Returns the atmospheric density at the reference altitude.
    #[inline]
    pub const fn reference_density(&self) -> MassDensity {
        self.reference_density
    }

    /// Returns the exponential scale height.
    #[inline]
    pub const fn scale_height(&self) -> Distance {
        self.scale_height
    }

    /// Returns the planet angular rate.
    #[inline]
    pub const fn angular_rate(&self) -> AngularRate {
        self.angular_rate
    }
}

/// Cannonball atmospheric drag model.
#[derive(Debug)]
pub struct AtmosphericDrag {
    /// Atmosphere model.
    atmosphere: ExponentialAtmosphere,
    /// Drag coefficient times reference area divided by mass.
    area_per_mass: AreaPerMass,
}

impl AtmosphericDrag {
    /// Creates a cannonball atmospheric drag model.
    #[inline]
    pub const fn new(atmosphere: ExponentialAtmosphere, area_per_mass: AreaPerMass) -> Self {
        Self {
            atmosphere,
            area_per_mass,
        }
    }

    /// Returns the atmosphere model.
    pub const fn atmosphere(&self) -> &ExponentialAtmosphere {
        &self.atmosphere
    }

    /// Returns the drag coefficient times reference area divided by mass.
    #[inline]
    pub const fn area_per_mass(&self) -> AreaPerMass {
        self.area_per_mass
    }
}

/// Allocation-free binary composition of two dynamics models.
///
/// Binary composition keeps the stepping path allocation-free. For three or
/// more models, nest: `CompositeDynamics<A, CompositeDynamics<B, C>>`.
#[derive(Debug)]
pub struct CompositeDynamics<A, B> {
    first: A,
    second: B,
}

impl<A, B> CompositeDynamics<A, B> {
    /// Creates a composite from two dynamics models.
    #[inline]
    pub const fn new(first: A, second: B) -> Self {
        Self { first, second }
    }

    /// Returns a reference to the first dynamics model.
    pub const fn first(&self) -> &A {
        &self.first
    }

    /// Returns a reference to the second dynamics model.
    pub const fn second(&self) -> &B {
        &self.second
    }
}

/// Closed set of validated force models used by the propagator.
///
/// Each variant owns a fully-validated dynamics model. The enum itself
/// implements [`Dynamics`] so the propagator step path dispatches without
/// trait objects and stays allocation-free.
#[derive(Debug)]
pub(crate) enum RuntimeDynamics {
    /// Point-mass central gravity.
    TwoBody(TwoBodyGravity),
    /// Central gravity with J2 perturbation.
    J2(J2Gravity),
    /// Two-body gravity composed with exponential atmospheric drag.
    TwoBodyDrag(CompositeDynamics<TwoBodyGravity, AtmosphericDrag>),
    /// J2 gravity composed with exponential atmospheric drag.
    J2Drag(CompositeDynamics<J2Gravity, AtmosphericDrag>),
}

/// Computes acceleration for a state at a given simulation time.
///
/// The `t` parameter supports time-dependent models (e.g. low-thrust
/// profiles). Time-independent implementations ignore it.
pub trait Dynamics {
    /// Returns the acceleration for `state` at epoch `t`.
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        t: Epoch,
    ) -> Result<Acceleration, DynamicsError>;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let px = state.px_meters();
        let py = state.py_meters();
        let pz = state.pz_meters();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();
        let mu = self.mu.meters_cubed_per_second_squared();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let scale = -mu / (radius_sq_m * radius_m);
        Ok(Acceleration::from_vector(Vector3::new(
            scale * px,
            scale * py,
            scale * pz,
        )))
    }
}

impl Dynamics for J2Gravity {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let px = state.px_meters();
        let py = state.py_meters();
        let pz = state.pz_meters();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();
        let mu = self.mu.meters_cubed_per_second_squared();
        let equatorial_radius = self.equatorial_radius.meters();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let central_scale = -mu / (radius_sq_m * radius_m);
        let z_sq_over_r_sq = pz * pz / radius_sq_m;
        let j2_factor =
            1.5 * self.j2.scalar() * mu * equatorial_radius * equatorial_radius / radius_m.powi(5);
        let xy_j2_scale = j2_factor * (5.0 * z_sq_over_r_sq - 1.0);
        let z_j2_scale = j2_factor * (5.0 * z_sq_over_r_sq - 3.0);

        Ok(Acceleration::from_vector(Vector3::new(
            central_scale * px + xy_j2_scale * px,
            central_scale * py + xy_j2_scale * py,
            central_scale * pz + z_j2_scale * pz,
        )))
    }
}

impl ExponentialAtmosphere {
    /// Computes atmospheric density at `radius`.
    ///
    /// Returns an error if the computed density is non-finite.
    #[inline]
    pub fn density_at_radius(&self, radius: Distance) -> Result<MassDensity, NumericalError> {
        let radius_m = radius.meters();
        let altitude_m = radius_m - self.reference_radius.meters();
        let reference_altitude_m = self.reference_altitude.meters();
        let density = self.reference_density.kilograms_per_cubic_meter()
            * (-(altitude_m - reference_altitude_m) / self.scale_height.meters()).exp();

        Ok(MassDensity::new(density))
    }
}

impl Dynamics for AtmosphericDrag {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let px = state.px_meters();
        let py = state.py_meters();
        let pz = state.pz_meters();
        let vx = state.vx_meters_per_second();
        let vy = state.vy_meters_per_second();
        let vz = state.vz_meters_per_second();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let density = self
            .atmosphere
            .density_at_radius(Distance::new(radius_m))?
            .kilograms_per_cubic_meter();
        let omega = self.atmosphere.angular_rate.radians_per_second();
        let atmosphere_vx = -omega * py;
        let atmosphere_vy = omega * px;
        let relative_vx = vx - atmosphere_vx;
        let relative_vy = vy - atmosphere_vy;
        let relative_vz = vz;
        let relative_speed =
            (relative_vx * relative_vx + relative_vy * relative_vy + relative_vz * relative_vz)
                .sqrt();
        let scale =
            -0.5 * density * self.area_per_mass.square_meters_per_kilogram() * relative_speed;

        Ok(Acceleration::from_vector(Vector3::new(
            scale * relative_vx,
            scale * relative_vy,
            scale * relative_vz,
        )))
    }
}

impl<A: Dynamics, B: Dynamics> Dynamics for CompositeDynamics<A, B> {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let first = self.first.acceleration_at(state, t)?;
        let second = self.second.acceleration_at(state, t)?;
        let fv = first.vector();
        let sv = second.vector();

        Ok(Acceleration::from_vector(Vector3::new(
            fv.x + sv.x,
            fv.y + sv.y,
            fv.z + sv.z,
        )))
    }
}

impl Dynamics for RuntimeDynamics {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        t: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        match self {
            Self::TwoBody(d) => d.acceleration_at(state, t),
            Self::J2(d) => d.acceleration_at(state, t),
            Self::TwoBodyDrag(d) => d.acceleration_at(state, t),
            Self::J2Drag(d) => d.acceleration_at(state, t),
        }
    }
}

/// Stack-allocated wrapper that adds thrust acceleration above a base
/// dynamics model.
///
/// Constructed per integrator step from borrows of the natural dynamics and
/// the current [`ThrustMode`][crate::physics::thrust::ThrustMode]. Guidance
/// is resolved at each integrator substep against the actual state, so
/// prograde direction is always current. Zero allocation, zero lifetime
/// infection of the stored dynamics composite.
#[derive(Debug)]
pub struct ThrustWrapper<'a, D: Dynamics + ?Sized> {
    /// The natural dynamics model (gravity, drag, etc.).
    pub base: &'a D,
    /// Current thrust guidance mode.
    pub mode: &'a crate::physics::thrust::ThrustMode,
}

impl<D: Dynamics + ?Sized> Dynamics for ThrustWrapper<'_, D> {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        epoch: Epoch,
    ) -> Result<Acceleration, DynamicsError> {
        let natural = self.base.acceleration_at(state, epoch)?;
        let thrust = self.mode.resolve_acceleration(state)?;
        Ok(natural + thrust)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::physics::frame::Frame;
    use crate::physics::integrators::{Integrator, RungeKutta4Integrator};
    use crate::physics::orbits::Orbit;
    use crate::physics::state::CartesianStateBuffer;
    use crate::physics::types::TimeStep;
    use nalgebra::Vector3;

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    fn earth_gravity() -> Result<TwoBodyGravity, Box<dyn std::error::Error>> {
        Ok(TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH)))
    }

    fn earth_j2_gravity(j2: f64) -> Result<J2Gravity, Box<dyn std::error::Error>> {
        Ok(J2Gravity::new(
            GravitationalParameter::new(MU_EARTH),
            Distance::new(R_EARTH),
            J2Coefficient::new(j2),
        ))
    }

    fn atmosphere() -> Result<ExponentialAtmosphere, Box<dyn std::error::Error>> {
        Ok(ExponentialAtmosphere::new(
            Distance::new(R_EARTH),
            Distance::new(250_000.0),
            MassDensity::new(1.0e-10),
            Distance::new(50_000.0),
            AngularRate::new(7.292_115_9e-5),
        ))
    }

    fn drag_model(area_per_mass: f64) -> Result<AtmosphericDrag, Box<dyn std::error::Error>> {
        Ok(AtmosphericDrag::new(
            atmosphere()?,
            AreaPerMass::new(area_per_mass),
        ))
    }

    #[test]
    fn frame_based_gravity_rejects_body_fixed_axes() {
        assert_eq!(
            TwoBodyGravity::from_frame(&Frame::earth_body_fixed()).err(),
            Some(StateError::InvalidAxes),
        );
        assert_eq!(
            J2Gravity::from_frame(&Frame::earth_body_fixed()).err(),
            Some(StateError::InvalidAxes),
        );
    }

    #[test]
    fn frame_based_atmosphere_rejects_body_fixed_axes() {
        assert_eq!(
            ExponentialAtmosphere::from_frame(&Frame::earth_body_fixed()).err(),
            Some(StateError::InvalidAxes),
        );
    }

    fn state_from_position(px_m: f64, py_m: f64, pz_m: f64) -> CartesianStateComponents {
        CartesianStateComponents::from_vectors(Vector3::new(px_m, py_m, pz_m), Vector3::zeros())
    }

    fn state_from_vectors(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> CartesianStateComponents {
        CartesianStateComponents::from_vectors(position, velocity)
    }

    #[test]
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH));
        let radius = R_EARTH + 400_000.0;
        let acceleration =
            gravity.acceleration_at(state_from_position(radius, 0.0, 0.0), Epoch::ZERO)?;

        assert!(acceleration.vector().x < 0.0);
        assert_close(acceleration.vector().y, 0.0, EPS);
        assert_close(acceleration.vector().z, 0.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_has_inverse_square_magnitude() -> TestResult {
        let gravity = TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH));
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(state_from_position(radius, 0.0, 0.0), Epoch::ZERO)?
            .vector();

        assert_close(acceleration.norm(), MU_EARTH / radius.powi(2), EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_matches_off_axis_vector_formula() -> TestResult {
        let gravity = TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH));
        let radius = R_EARTH + 400_000.0;
        let position = Vector3::new(3.0 * radius, 4.0 * radius, 0.0);
        let expected = -MU_EARTH * position / position.norm().powi(3);

        let acceleration = gravity
            .acceleration_at(
                state_from_position(position.x, position.y, position.z),
                Epoch::ZERO,
            )?
            .vector();

        assert_close(acceleration.x, expected.x, EPS);
        assert_close(acceleration.y, expected.y, EPS);
        assert_close(acceleration.z, expected.z, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_falls_off_with_inverse_square_distance() -> TestResult {
        let gravity = TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH));
        let radius = R_EARTH + 400_000.0;
        let epoch = Epoch::ZERO;
        let near_acceleration = gravity
            .acceleration_at(state_from_position(radius, 0.0, 0.0), epoch)?
            .vector()
            .norm();
        let far_acceleration = gravity
            .acceleration_at(state_from_position(2.0 * radius, 0.0, 0.0), epoch)?
            .vector()
            .norm();

        assert_close(far_acceleration, near_acceleration / 4.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = TwoBodyGravity::new(GravitationalParameter::new(MU_EARTH));
        assert!(matches!(
            gravity.acceleration_at(state_from_position(0.0, 0.0, 0.0), Epoch::ZERO),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn j2_gravity_with_zero_j2_matches_two_body_gravity() -> TestResult {
        let two_body = earth_gravity()?;
        let j2_gravity = earth_j2_gravity(0.0)?;
        let px = R_EARTH + 400_000.0;
        let epoch = Epoch::ZERO;

        let two_body_acceleration = two_body
            .acceleration_at(state_from_position(px, 200_000.0, 300_000.0), epoch)?
            .vector();
        let j2_acceleration = j2_gravity
            .acceleration_at(state_from_position(px, 200_000.0, 300_000.0), epoch)?
            .vector();

        assert_close(j2_acceleration.x, two_body_acceleration.x, EPS);
        assert_close(j2_acceleration.y, two_body_acceleration.y, EPS);
        assert_close(j2_acceleration.z, two_body_acceleration.z, EPS);
        Ok(())
    }

    #[test]
    fn j2_gravity_is_finite_for_inclined_leo_position() -> TestResult {
        let gravity = earth_j2_gravity(J2_EARTH)?;

        let acceleration = gravity
            .acceleration_at(
                state_from_position(R_EARTH + 400_000.0, 500_000.0, 1_000_000.0),
                Epoch::ZERO,
            )?
            .vector();

        assert!(acceleration.x.is_finite());
        assert!(acceleration.y.is_finite());
        assert!(acceleration.z.is_finite());
        Ok(())
    }

    #[test]
    fn j2_gravity_rejects_zero_radius() -> TestResult {
        let gravity = earth_j2_gravity(J2_EARTH)?;

        assert!(matches!(
            gravity.acceleration_at(state_from_position(0.0, 0.0, 0.0), Epoch::ZERO,),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn j2_gravity_stores_j2_coefficient() -> TestResult {
        let j2 = J2Coefficient::new(J2_EARTH);
        let gravity = J2Gravity::new(
            GravitationalParameter::new(MU_EARTH),
            Distance::new(R_EARTH),
            j2,
        );

        assert_close(gravity.j2.scalar(), J2_EARTH, EPS);
        Ok(())
    }

    #[test]
    fn j2_gravity_equatorial_j2_increases_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let j2_gravity = earth_j2_gravity(J2_EARTH)?;
        let epoch = Epoch::ZERO;

        let two_body_x = two_body
            .acceleration_at(state_from_position(R_EARTH + 400_000.0, 0.0, 0.0), epoch)?
            .vector()
            .x;
        let j2_x = j2_gravity
            .acceleration_at(state_from_position(R_EARTH + 400_000.0, 0.0, 0.0), epoch)?
            .vector()
            .x;

        assert!(two_body_x < 0.0);
        assert!(j2_x < two_body_x);
        Ok(())
    }

    #[test]
    fn j2_gravity_polar_axis_j2_reduces_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let j2_gravity = earth_j2_gravity(J2_EARTH)?;
        let epoch = Epoch::ZERO;

        let two_body_z = two_body
            .acceleration_at(state_from_position(0.0, 0.0, R_EARTH + 400_000.0), epoch)?
            .vector()
            .z;
        let j2_z = j2_gravity
            .acceleration_at(state_from_position(0.0, 0.0, R_EARTH + 400_000.0), epoch)?
            .vector()
            .z;

        assert!(two_body_z < 0.0);
        assert!(j2_z > two_body_z);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_density_decreases_with_altitude() -> TestResult {
        let atmosphere = atmosphere()?;
        let low = atmosphere
            .density_at_radius(Distance::new(R_EARTH + 250_000.0))?
            .kilograms_per_cubic_meter();
        let high = atmosphere
            .density_at_radius(Distance::new(R_EARTH + 350_000.0))?
            .kilograms_per_cubic_meter();

        assert!(high < low);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_matches_reference_density() -> TestResult {
        let atmosphere = atmosphere()?;
        let density = atmosphere.density_at_radius(Distance::new(R_EARTH + 250_000.0))?;

        assert_close(density.kilograms_per_cubic_meter(), 1.0e-10, EPS);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_opposes_relative_velocity() -> TestResult {
        let base = atmosphere()?;
        let drag = AtmosphericDrag::new(
            ExponentialAtmosphere::new(
                base.reference_radius,
                base.reference_altitude,
                base.reference_density,
                base.scale_height,
                AngularRate::new(0.0),
            ),
            AreaPerMass::new(0.02),
        );
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let acceleration = drag.acceleration_at(state, Epoch::ZERO)?.vector();

        assert!(acceleration.dot(&state.velocity_vector()) < 0.0);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_is_zero_when_relative_velocity_is_zero() -> TestResult {
        let atmosphere = atmosphere()?;
        let radius = R_EARTH + 250_000.0;
        let omega = atmosphere.angular_rate.radians_per_second();
        let drag = AtmosphericDrag::new(atmosphere, AreaPerMass::new(0.02));
        let state = state_from_vectors(
            Vector3::new(radius, 0.0, 0.0),
            Vector3::new(0.0, omega * radius, 0.0),
        );

        let acceleration = drag.acceleration_at(state, Epoch::ZERO)?.vector();

        assert_close(acceleration.norm(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn composite_dynamics_sums_component_accelerations() -> TestResult {
        let gravity = earth_gravity()?;
        let drag = drag_model(0.02)?;
        let composite = CompositeDynamics::new(gravity, drag);
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );
        let epoch = Epoch::ZERO;

        let gravity_acceleration = composite.first.acceleration_at(state, epoch)?;
        let drag_acceleration = composite.second.acceleration_at(state, epoch)?;
        let total = composite.acceleration_at(state, epoch)?;

        assert_close(
            total.vector().x,
            gravity_acceleration.vector().x + drag_acceleration.vector().x,
            EPS,
        );
        assert_close(
            total.vector().y,
            gravity_acceleration.vector().y + drag_acceleration.vector().y,
            EPS,
        );
        assert_close(
            total.vector().z,
            gravity_acceleration.vector().z + drag_acceleration.vector().z,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn drag_decay_reduces_altitude() -> TestResult {
        let frame = Frame::earth_inertial();
        let altitude_m = 250_000.0;
        let orbit = Orbit::circular_equatorial(Distance::new(altitude_m));
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &orbit.to_cartesian_state(&frame)?)?;
        let initial_altitude_m = states.view().read_state(0)?.position.vector().norm() - R_EARTH;
        let dynamics =
            CompositeDynamics::new(TwoBodyGravity::from_frame(&frame)?, drag_model(0.1)?);
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::new(10.0);
        let mut epoch = Epoch::ZERO;
        let period_s = orbit.period(&Frame::earth_inertial())?.seconds();

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::new(dt.seconds().min(remaining_s));
            integrator.step(&dynamics, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        let final_altitude_m = states.view().read_state(0)?.position.vector().norm() - R_EARTH;
        assert!(
            final_altitude_m < initial_altitude_m,
            "drag should reduce altitude: initial {initial_altitude_m} m, final {final_altitude_m} m",
        );
        Ok(())
    }
}
