pub use super::state::{CartesianAccelerationComponents, CartesianStateComponents};
pub use super::types::{
    AngularRate, AreaPerMass, Distance, Epoch, GravitationalParameter, J2Coefficient, MassDensity,
};
use crate::errors::{DynamicsError, NumericalError};

/// Point-mass gravity model for a single central body.
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

    /// Returns the central body's gravitational parameter.
    #[inline]
    pub const fn mu(&self) -> GravitationalParameter {
        self.mu
    }
}

/// Gravity model for an oblate central body with J2 perturbation.
pub struct OblateBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    mu: GravitationalParameter,
    /// Equatorial reference radius in meters.
    equatorial_radius: Distance,
    /// Dimensionless second zonal harmonic coefficient.
    j2: J2Coefficient,
}

impl OblateBodyGravity {
    /// Creates an oblate-body gravity model.
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
    #[inline]
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
    #[inline]
    pub const fn first(&self) -> &A {
        &self.first
    }

    /// Returns a reference to the second dynamics model.
    #[inline]
    pub const fn second(&self) -> &B {
        &self.second
    }
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
    ) -> Result<CartesianAccelerationComponents, DynamicsError>;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let position = state.position;
        let px = position.x_meters();
        let py = position.y_meters();
        let pz = position.z_meters();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();
        let mu = self.mu.meters_cubed_per_second_squared();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let scale = -mu / (radius_sq_m * radius_m);
        Ok(
            CartesianAccelerationComponents::from_meters_per_second_squared(
                scale * px,
                scale * py,
                scale * pz,
            )?,
        )
    }
}

impl Dynamics for OblateBodyGravity {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let position = state.position;
        let px = position.x_meters();
        let py = position.y_meters();
        let pz = position.z_meters();
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

        Ok(
            CartesianAccelerationComponents::from_meters_per_second_squared(
                central_scale * px + xy_j2_scale * px,
                central_scale * py + xy_j2_scale * py,
                central_scale * pz + z_j2_scale * pz,
            )?,
        )
    }
}

impl ExponentialAtmosphere {
    /// Computes atmospheric density at `radius`.
    ///
    /// Returns an error if the computed density is non-finite.
    pub fn density_at_radius(&self, radius: Distance) -> Result<MassDensity, NumericalError> {
        self.density_at_radius_m(radius.meters())
    }

    fn density_at_radius_m(&self, radius_m: f64) -> Result<MassDensity, NumericalError> {
        let altitude_m = radius_m - self.reference_radius.meters();
        let reference_altitude_m = self.reference_altitude.meters();
        let density = self.reference_density.kilograms_per_cubic_meter()
            * (-(altitude_m - reference_altitude_m) / self.scale_height.meters()).exp();

        MassDensity::from_kilograms_per_cubic_meter(density)
    }
}

impl Dynamics for AtmosphericDrag {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let position = state.position;
        let velocity = state.velocity;
        let px = position.x_meters();
        let py = position.y_meters();
        let pz = position.z_meters();
        let vx = velocity.x_meters_per_second();
        let vy = velocity.y_meters_per_second();
        let vz = velocity.z_meters_per_second();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let density = self
            .atmosphere
            .density_at_radius_m(radius_m)?
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

        Ok(
            CartesianAccelerationComponents::from_meters_per_second_squared(
                scale * relative_vx,
                scale * relative_vy,
                scale * relative_vz,
            )?,
        )
    }
}

impl<A: Dynamics, B: Dynamics> Dynamics for CompositeDynamics<A, B> {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let first = self.first.acceleration_at(state, t)?;
        let second = self.second.acceleration_at(state, t)?;

        Ok(
            CartesianAccelerationComponents::from_meters_per_second_squared(
                first.x_meters_per_second_squared() + second.x_meters_per_second_squared(),
                first.y_meters_per_second_squared() + second.y_meters_per_second_squared(),
                first.z_meters_per_second_squared() + second.z_meters_per_second_squared(),
            )?,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::physics::diagnostics::OrbitalDiagnostics;
    use crate::physics::integrators::{Integrator, RungeKutta4Integrator};
    use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
    use crate::physics::state::{
        CartesianPositionComponents, CartesianStateBuffer, CartesianVelocityComponents,
    };
    use crate::physics::types::{Angle, Eccentricity, Position, TimeStep};
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
        Ok(TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        ))
    }

    fn earth_oblate_gravity(j2: f64) -> Result<OblateBodyGravity, Box<dyn std::error::Error>> {
        Ok(OblateBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            Distance::from_meters(R_EARTH)?,
            J2Coefficient::from_scalar(j2)?,
        ))
    }

    fn atmosphere() -> Result<ExponentialAtmosphere, Box<dyn std::error::Error>> {
        Ok(ExponentialAtmosphere::new(
            Distance::from_meters(R_EARTH)?,
            Distance::from_meters(250_000.0)?,
            MassDensity::from_kilograms_per_cubic_meter(1.0e-10)?,
            Distance::from_meters(50_000.0)?,
            AngularRate::from_radians_per_second(7.292_115_9e-5)?,
        ))
    }

    fn drag_model(area_per_mass: f64) -> Result<AtmosphericDrag, Box<dyn std::error::Error>> {
        Ok(AtmosphericDrag::new(
            atmosphere()?,
            AreaPerMass::from_square_meters_per_kilogram(area_per_mass)?,
        ))
    }

    fn state_from_position(position: CartesianPositionComponents) -> CartesianStateComponents {
        CartesianStateComponents {
            position,
            velocity: CartesianVelocityComponents::from_meters_per_second(0.0, 0.0, 0.0).unwrap(),
        }
    }

    fn state_from_vectors(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> Result<CartesianStateComponents, Box<dyn std::error::Error>> {
        Ok(CartesianStateComponents {
            position: CartesianPositionComponents::from_meters(position.x, position.y, position.z)?,
            velocity: CartesianVelocityComponents::from_meters_per_second(
                velocity.x, velocity.y, velocity.z,
            )?,
        })
    }

    #[test]
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity.acceleration_at(
            state_from_position(CartesianPositionComponents::from_meters(radius, 0.0, 0.0)?),
            Epoch::from_seconds(0.0)?,
        )?;

        assert!(acceleration.x_meters_per_second_squared() < 0.0);
        assert_close(acceleration.y_meters_per_second_squared(), 0.0, EPS);
        assert_close(acceleration.z_meters_per_second_squared(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_has_inverse_square_magnitude() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(radius, 0.0, 0.0)?),
                Epoch::from_seconds(0.0)?,
            )?
            .vector();

        assert_close(acceleration.norm(), MU_EARTH / radius.powi(2), EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_matches_off_axis_vector_formula() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        let radius = R_EARTH + 400_000.0;
        let position = Vector3::new(3.0 * radius, 4.0 * radius, 0.0);
        let expected = -MU_EARTH * position / position.norm().powi(3);

        let acceleration = gravity
            .acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(
                    position.x, position.y, position.z,
                )?),
                Epoch::from_seconds(0.0)?,
            )?
            .vector();

        assert_close(acceleration.x, expected.x, EPS);
        assert_close(acceleration.y, expected.y, EPS);
        assert_close(acceleration.z, expected.z, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_falls_off_with_inverse_square_distance() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        let radius = R_EARTH + 400_000.0;
        let epoch = Epoch::from_seconds(0.0)?;
        let near_acceleration = gravity
            .acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(radius, 0.0, 0.0)?),
                epoch,
            )?
            .vector()
            .norm();
        let far_acceleration = gravity
            .acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(
                    2.0 * radius,
                    0.0,
                    0.0,
                )?),
                epoch,
            )?
            .vector()
            .norm();

        assert_close(far_acceleration, near_acceleration / 4.0, EPS);
        Ok(())
    }

    #[test]
    fn position_rejects_nan_components() {
        assert!(Position::from_vector(Vector3::new(f64::NAN, 0.0, 0.0)).is_err());
    }

    #[test]
    fn gravitational_parameter_rejects_zero() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(0.0).is_err());
    }

    #[test]
    fn gravitational_parameter_rejects_nan() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(f64::NAN).is_err());
    }

    #[test]
    fn two_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)?,),
                Epoch::from_seconds(0.0)?
            ),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn two_body_gravity_rejects_nonfinite_acceleration() -> TestResult {
        let gravity = TwoBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        );
        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(
                    1.0e-150, 0.0, 0.0
                )?,),
                Epoch::from_seconds(0.0)?
            ),
            Err(DynamicsError::Numerical(NumericalError::NotFinite {
                field: "acceleration components"
            })),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_with_zero_j2_matches_two_body_gravity() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(0.0)?;
        let position =
            CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 200_000.0, 300_000.0)?;
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_acceleration = two_body
            .acceleration_at(state_from_position(position), epoch)?
            .vector();
        let oblate_acceleration = oblate
            .acceleration_at(state_from_position(position), epoch)?
            .vector();

        assert_close(oblate_acceleration.x, two_body_acceleration.x, EPS);
        assert_close(oblate_acceleration.y, two_body_acceleration.y, EPS);
        assert_close(oblate_acceleration.z, two_body_acceleration.z, EPS);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_is_finite_for_inclined_leo_position() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH)?;
        let position =
            CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 500_000.0, 1_000_000.0)?;

        let acceleration = gravity
            .acceleration_at(state_from_position(position), Epoch::from_seconds(0.0)?)?
            .vector();

        assert!(acceleration.x.is_finite());
        assert!(acceleration.y.is_finite());
        assert!(acceleration.z.is_finite());
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH)?;

        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)?,),
                Epoch::from_seconds(0.0)?,
            ),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_rejects_nonfinite_j2_at_construction() -> TestResult {
        assert!(matches!(
            J2Coefficient::from_scalar(f64::NAN),
            Err(NumericalError::NotFinite { field: "j2" }),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_stores_j2_coefficient() -> TestResult {
        let j2 = J2Coefficient::from_scalar(J2_EARTH)?;
        let gravity = OblateBodyGravity::new(
            GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            Distance::from_meters(R_EARTH)?,
            j2,
        );

        assert_close(gravity.j2.scalar(), J2_EARTH, EPS);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_equatorial_j2_increases_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(J2_EARTH)?;
        let position = CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 0.0, 0.0)?;
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_x = two_body
            .acceleration_at(state_from_position(position), epoch)?
            .x_meters_per_second_squared();
        let oblate_x = oblate
            .acceleration_at(state_from_position(position), epoch)?
            .x_meters_per_second_squared();

        assert!(two_body_x < 0.0);
        assert!(oblate_x < two_body_x);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_polar_axis_j2_reduces_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(J2_EARTH)?;
        let position = CartesianPositionComponents::from_meters(0.0, 0.0, R_EARTH + 400_000.0)?;
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_z = two_body
            .acceleration_at(state_from_position(position), epoch)?
            .z_meters_per_second_squared();
        let oblate_z = oblate
            .acceleration_at(state_from_position(position), epoch)?
            .z_meters_per_second_squared();

        assert!(two_body_z < 0.0);
        assert!(oblate_z > two_body_z);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_density_decreases_with_altitude() -> TestResult {
        let atmosphere = atmosphere()?;
        let low = atmosphere
            .density_at_radius(Distance::from_meters(R_EARTH + 250_000.0)?)?
            .kilograms_per_cubic_meter();
        let high = atmosphere
            .density_at_radius(Distance::from_meters(R_EARTH + 350_000.0)?)?
            .kilograms_per_cubic_meter();

        assert!(high < low);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_matches_reference_density() -> TestResult {
        let atmosphere = atmosphere()?;
        let density = atmosphere.density_at_radius(Distance::from_meters(R_EARTH + 250_000.0)?)?;

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
                AngularRate::from_radians_per_second(0.0)?,
            ),
            AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        );
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        )?;

        let acceleration = drag
            .acceleration_at(state, Epoch::from_seconds(0.0)?)?
            .vector();

        assert!(acceleration.dot(&state.velocity.vector()) < 0.0);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_is_zero_when_relative_velocity_is_zero() -> TestResult {
        let atmosphere = atmosphere()?;
        let radius = R_EARTH + 250_000.0;
        let omega = atmosphere.angular_rate.radians_per_second();
        let drag = AtmosphericDrag::new(
            atmosphere,
            AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        );
        let state = state_from_vectors(
            Vector3::new(radius, 0.0, 0.0),
            Vector3::new(0.0, omega * radius, 0.0),
        )?;

        let acceleration = drag
            .acceleration_at(state, Epoch::from_seconds(0.0)?)?
            .vector();

        assert_close(acceleration.norm(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_rejects_nonfinite_acceleration() -> TestResult {
        let base = atmosphere()?;
        let drag = AtmosphericDrag::new(
            ExponentialAtmosphere::new(
                base.reference_radius,
                base.reference_altitude,
                MassDensity::from_kilograms_per_cubic_meter(f64::MAX)?,
                base.scale_height,
                base.angular_rate,
            ),
            AreaPerMass::from_square_meters_per_kilogram(f64::MAX)?,
        );
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, f64::MAX, 0.0),
        )?;

        assert!(matches!(
            drag.acceleration_at(state, Epoch::from_seconds(0.0)?),
            Err(DynamicsError::Numerical(NumericalError::NotFinite {
                field: "acceleration components"
            })),
        ));
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
        )?;
        let epoch = Epoch::from_seconds(0.0)?;

        let gravity_acceleration = composite.first.acceleration_at(state, epoch)?;
        let drag_acceleration = composite.second.acceleration_at(state, epoch)?;
        let total = composite.acceleration_at(state, epoch)?;

        assert_close(
            total.x_meters_per_second_squared(),
            gravity_acceleration.x_meters_per_second_squared()
                + drag_acceleration.x_meters_per_second_squared(),
            EPS,
        );
        assert_close(
            total.y_meters_per_second_squared(),
            gravity_acceleration.y_meters_per_second_squared()
                + drag_acceleration.y_meters_per_second_squared(),
            EPS,
        );
        assert_close(
            total.z_meters_per_second_squared(),
            gravity_acceleration.z_meters_per_second_squared()
                + drag_acceleration.z_meters_per_second_squared(),
            EPS,
        );
        Ok(())
    }

    #[test]
    fn low_orbit_with_drag_loses_specific_energy() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
        let orbit = EllipticOrbit::new(
            Distance::from_meters(R_EARTH + 250_000.0)?,
            Eccentricity::from_scalar(0.0)?,
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0)?,
            OrbitDirection::Prograde,
        )?;
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let dynamics = CompositeDynamics::new(TwoBodyGravity::new(mu), drag_model(0.05)?);
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy()
            .square_meters_per_square_second();
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let mut epoch = Epoch::from_seconds(0.0)?;

        for _ in 0..120 {
            integrator.step(&dynamics, &mut states.view_mut(), 0, epoch, dt)?;
            epoch = epoch.advance_by(dt);
        }

        let final_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy()
            .square_meters_per_square_second();

        assert!(final_energy < initial_energy);
        Ok(())
    }

    #[test]
    fn drag_decay_energy_loss_matches_physical_expectation() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
        let radius = R_EARTH + 250_000.0;
        let orbit = EllipticOrbit::new(
            Distance::from_meters(radius)?,
            Eccentricity::from_scalar(0.0)?,
            mu,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::from_radians(0.0)?,
            OrbitDirection::Prograde,
        )?;
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy()
            .square_meters_per_square_second();
        let initial_altitude_m = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .radius()
            .meters()
            - R_EARTH;
        let dynamics = CompositeDynamics::new(TwoBodyGravity::new(mu), drag_model(0.1)?);
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let mut epoch = Epoch::from_seconds(0.0)?;
        let period_s = orbit.period()?.seconds();

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_s))?;
            integrator.step(&dynamics, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step);
        }

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
        let final_energy = diagnostics
            .specific_energy()
            .square_meters_per_square_second();
        let final_altitude_m = diagnostics.radius().meters() - R_EARTH;
        let energy_loss = initial_energy - final_energy;

        let min_energy_loss = 1.0;
        let max_energy_loss = 100_000.0;
        assert!(
            energy_loss > min_energy_loss,
            "expected energy loss > {min_energy_loss} m²/s², got {energy_loss}",
        );
        assert!(
            energy_loss < max_energy_loss,
            "energy loss {energy_loss} exceeds sanity bound {max_energy_loss} m²/s²",
        );
        assert!(
            final_altitude_m < initial_altitude_m,
            "drag should reduce altitude: initial {initial_altitude_m} m, final {final_altitude_m} m",
        );
        Ok(())
    }
}
