pub use super::state::{CartesianAccelerationComponents, CartesianStateComponents};
pub use super::types::{
    AngularRate, AreaPerMass, Distance, Epoch, GravitationalParameter, MassDensity,
};
use crate::errors::DynamicsError;

/// Point-mass gravity model for a single central body.
pub struct TwoBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    pub mu: GravitationalParameter,
}

/// Gravity model for an oblate central body with J2 perturbation.
pub struct OblateBodyGravity {
    /// Standard gravitational parameter in cubic meters per second squared.
    pub mu: GravitationalParameter,
    /// Equatorial reference radius in meters.
    pub equatorial_radius: Distance,
    /// Dimensionless second zonal harmonic coefficient.
    pub j2: f64,
}

/// Spherical exponential atmosphere rotating about the inertial z axis.
pub struct ExponentialAtmosphere {
    /// Planet reference radius in meters.
    pub reference_radius: Distance,
    /// Reference altitude above `reference_radius`.
    pub reference_altitude: Distance,
    /// Density at `reference_altitude`.
    pub reference_density: MassDensity,
    /// Exponential scale height.
    pub scale_height: Distance,
    /// Planet angular rate about the inertial z axis.
    pub angular_rate: AngularRate,
}

/// Cannonball atmospheric drag model.
pub struct AtmosphericDrag {
    /// Atmosphere model.
    pub atmosphere: ExponentialAtmosphere,
    /// Drag coefficient times reference area divided by mass.
    pub area_per_mass: AreaPerMass,
}

/// Allocation-free composition of two local dynamics models.
pub struct CompositeDynamics<A, B> {
    /// First dynamics model.
    pub first: A,
    /// Second dynamics model.
    pub second: B,
}

/// Computes acceleration for a state at a given simulation time.
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
        CartesianAccelerationComponents::from_meters_per_second_squared(
            scale * px,
            scale * py,
            scale * pz,
        )
        .map_err(DynamicsError::from)
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
            1.5 * self.j2 * mu * equatorial_radius * equatorial_radius / radius_m.powi(5);
        let xy_j2_scale = j2_factor * (5.0 * z_sq_over_r_sq - 1.0);
        let z_j2_scale = j2_factor * (5.0 * z_sq_over_r_sq - 3.0);

        CartesianAccelerationComponents::from_meters_per_second_squared(
            central_scale * px + xy_j2_scale * px,
            central_scale * py + xy_j2_scale * py,
            central_scale * pz + z_j2_scale * pz,
        )
        .map_err(DynamicsError::from)
    }
}

impl ExponentialAtmosphere {
    /// Computes atmospheric density at `radius`.
    pub fn density_at_radius(&self, radius: Distance) -> Option<MassDensity> {
        self.density_at_radius_m(radius.meters())
    }

    fn density_at_radius_m(&self, radius_m: f64) -> Option<MassDensity> {
        let altitude_m = radius_m - self.reference_radius.meters();
        let reference_altitude_m = self.reference_altitude.meters();
        let density = self.reference_density.kilograms_per_cubic_meter()
            * (-(altitude_m - reference_altitude_m) / self.scale_height.meters()).exp();

        MassDensity::from_kilograms_per_cubic_meter(density).ok()
    }
}

impl Dynamics for AtmosphericDrag {
    fn acceleration_at(
        &self,
        state: CartesianStateComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let position = state.position.vector();
        let velocity = state.velocity.vector();
        let radius_m = position.norm();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let density = self
            .atmosphere
            .density_at_radius_m(radius_m)
            .ok_or(DynamicsError::InvalidAcceleration)?
            .kilograms_per_cubic_meter();
        let omega = self.atmosphere.angular_rate.radians_per_second();
        let atmosphere_velocity =
            nalgebra::Vector3::new(-omega * position.y, omega * position.x, 0.0);
        let relative_velocity = velocity - atmosphere_velocity;
        let relative_speed = relative_velocity.norm();
        let scale =
            -0.5 * density * self.area_per_mass.square_meters_per_kilogram() * relative_speed;
        let acceleration = scale * relative_velocity;

        CartesianAccelerationComponents::from_meters_per_second_squared(
            acceleration.x,
            acceleration.y,
            acceleration.z,
        )
        .map_err(DynamicsError::from)
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

        CartesianAccelerationComponents::from_meters_per_second_squared(
            first.x_meters_per_second_squared() + second.x_meters_per_second_squared(),
            first.y_meters_per_second_squared() + second.y_meters_per_second_squared(),
            first.z_meters_per_second_squared() + second.z_meters_per_second_squared(),
        )
        .map_err(DynamicsError::from)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::DomainError;
    use crate::physics::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::physics::diagnostics::OrbitalDiagnostics;
    use crate::physics::integrators::{Integrator, RungeKutta4Integrator};
    use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
    use crate::physics::state::{
        CartesianPositionComponents, CartesianStateBuffer, CartesianVelocityComponents,
    };
    use crate::physics::types::{Angle, Eccentricity, Position, TimeStep};
    use nalgebra::Vector3;

    type TestResult = Result<(), &'static str>;

    const EPS: f64 = 1.0e-9;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    fn earth_gravity() -> Option<TwoBodyGravity> {
        Some(TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).ok()?,
        })
    }

    fn earth_oblate_gravity(j2: f64) -> Option<OblateBodyGravity> {
        Some(OblateBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH).ok()?,
            equatorial_radius: Distance::from_meters(R_EARTH).ok()?,
            j2,
        })
    }

    fn atmosphere() -> Option<ExponentialAtmosphere> {
        Some(ExponentialAtmosphere {
            reference_radius: Distance::from_meters(R_EARTH).ok()?,
            reference_altitude: Distance::from_meters(250_000.0).ok()?,
            reference_density: MassDensity::from_kilograms_per_cubic_meter(1.0e-10).ok()?,
            scale_height: Distance::from_meters(50_000.0).ok()?,
            angular_rate: AngularRate::from_radians_per_second(7.292_115_9e-5).ok()?,
        })
    }

    fn drag_model(area_per_mass: f64) -> Option<AtmosphericDrag> {
        Some(AtmosphericDrag {
            atmosphere: atmosphere()?,
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(area_per_mass).ok()?,
        })
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
    ) -> Option<CartesianStateComponents> {
        Some(CartesianStateComponents {
            position: CartesianPositionComponents::from_meters(position.x, position.y, position.z)
                .ok()?,
            velocity: CartesianVelocityComponents::from_meters_per_second(
                velocity.x, velocity.y, velocity.z,
            )
            .ok()?,
        })
    }

    #[test]
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid acceleration")?;

        assert!(acceleration.x_meters_per_second_squared() < 0.0);
        assert_close(acceleration.y_meters_per_second_squared(), 0.0, EPS);
        assert_close(acceleration.z_meters_per_second_squared(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_has_inverse_square_magnitude() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid acceleration")?
            .vector();

        assert_close(acceleration.norm(), MU_EARTH / radius.powi(2), EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_matches_off_axis_vector_formula() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let position = Vector3::new(3.0 * radius, 4.0 * radius, 0.0);
        let expected = -MU_EARTH * position / position.norm().powi(3);

        let acceleration = gravity
            .acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(position.x, position.y, position.z)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid acceleration")?
            .vector();

        assert_close(acceleration.x, expected.x, EPS);
        assert_close(acceleration.y, expected.y, EPS);
        assert_close(acceleration.z, expected.z, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_falls_off_with_inverse_square_distance() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;
        let near_acceleration = gravity
            .acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                        .map_err(|_| "invalid near position")?,
                ),
                epoch,
            )
            .map_err(|_| "invalid near acceleration")?
            .vector()
            .norm();
        let far_acceleration = gravity
            .acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(2.0 * radius, 0.0, 0.0)
                        .map_err(|_| "invalid far position")?,
                ),
                epoch,
            )
            .map_err(|_| "invalid far acceleration")?
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
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?
            ),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn two_body_gravity_rejects_nonfinite_acceleration() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .map_err(|_| "invalid earth mu")?,
        };
        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(1.0e-150, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?
            ),
            Err(DynamicsError::Domain(DomainError::NotFinite)),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_with_zero_j2_matches_two_body_gravity() -> TestResult {
        let two_body = earth_gravity().ok_or("invalid two-body gravity")?;
        let oblate = earth_oblate_gravity(0.0).ok_or("invalid oblate gravity")?;
        let position =
            CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 200_000.0, 300_000.0)
                .map_err(|_| "invalid position")?;
        let epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;

        let two_body_acceleration = two_body
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .vector();
        let oblate_acceleration = oblate
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid oblate acceleration")?
            .vector();

        assert_close(oblate_acceleration.x, two_body_acceleration.x, EPS);
        assert_close(oblate_acceleration.y, two_body_acceleration.y, EPS);
        assert_close(oblate_acceleration.z, two_body_acceleration.z, EPS);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_is_finite_for_inclined_leo_position() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH).ok_or("invalid oblate gravity")?;
        let position =
            CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 500_000.0, 1_000_000.0)
                .map_err(|_| "invalid position")?;

        let acceleration = gravity
            .acceleration_at(
                state_from_position(position),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid acceleration")?
            .vector();

        assert!(acceleration.x.is_finite());
        assert!(acceleration.y.is_finite());
        assert!(acceleration.z.is_finite());
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH).ok_or("invalid oblate gravity")?;

        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            ),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_rejects_nonfinite_j2_acceleration() -> TestResult {
        let gravity = earth_oblate_gravity(f64::NAN).ok_or("invalid oblate gravity")?;

        assert!(matches!(
            gravity.acceleration_at(
                state_from_position(
                    CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 0.0, 0.0)
                        .map_err(|_| "invalid position")?,
                ),
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            ),
            Err(DynamicsError::Domain(DomainError::NotFinite)),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_equatorial_j2_increases_inward_acceleration() -> TestResult {
        let two_body = earth_gravity().ok_or("invalid two-body gravity")?;
        let oblate = earth_oblate_gravity(J2_EARTH).ok_or("invalid oblate gravity")?;
        let position = CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 0.0, 0.0)
            .map_err(|_| "invalid position")?;
        let epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;

        let two_body_x = two_body
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .x_meters_per_second_squared();
        let oblate_x = oblate
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid oblate acceleration")?
            .x_meters_per_second_squared();

        assert!(two_body_x < 0.0);
        assert!(oblate_x < two_body_x);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_polar_axis_j2_reduces_inward_acceleration() -> TestResult {
        let two_body = earth_gravity().ok_or("invalid two-body gravity")?;
        let oblate = earth_oblate_gravity(J2_EARTH).ok_or("invalid oblate gravity")?;
        let position = CartesianPositionComponents::from_meters(0.0, 0.0, R_EARTH + 400_000.0)
            .map_err(|_| "invalid position")?;
        let epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;

        let two_body_z = two_body
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .z_meters_per_second_squared();
        let oblate_z = oblate
            .acceleration_at(state_from_position(position), epoch)
            .map_err(|_| "invalid oblate acceleration")?
            .z_meters_per_second_squared();

        assert!(two_body_z < 0.0);
        assert!(oblate_z > two_body_z);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_density_decreases_with_altitude() -> TestResult {
        let atmosphere = atmosphere().ok_or("invalid atmosphere")?;
        let low = atmosphere
            .density_at_radius(
                Distance::from_meters(R_EARTH + 250_000.0).map_err(|_| "invalid low")?,
            )
            .ok_or("invalid low density")?
            .kilograms_per_cubic_meter();
        let high = atmosphere
            .density_at_radius(
                Distance::from_meters(R_EARTH + 350_000.0).map_err(|_| "invalid high")?,
            )
            .ok_or("invalid high density")?
            .kilograms_per_cubic_meter();

        assert!(high < low);
        Ok(())
    }

    #[test]
    fn exponential_atmosphere_matches_reference_density() -> TestResult {
        let atmosphere = atmosphere().ok_or("invalid atmosphere")?;
        let density = atmosphere
            .density_at_radius(
                Distance::from_meters(R_EARTH + 250_000.0).map_err(|_| "invalid radius")?,
            )
            .ok_or("invalid density")?;

        assert_close(density.kilograms_per_cubic_meter(), 1.0e-10, EPS);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_opposes_relative_velocity() -> TestResult {
        let drag = AtmosphericDrag {
            atmosphere: ExponentialAtmosphere {
                angular_rate: AngularRate::from_radians_per_second(0.0)
                    .map_err(|_| "invalid angular rate")?,
                ..atmosphere().ok_or("invalid atmosphere")?
            },
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(0.02)
                .map_err(|_| "invalid area per mass")?,
        };
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        )
        .ok_or("invalid state")?;

        let acceleration = drag
            .acceleration_at(
                state,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid drag acceleration")?
            .vector();

        assert!(acceleration.dot(&state.velocity.vector()) < 0.0);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_is_zero_when_relative_velocity_is_zero() -> TestResult {
        let atmosphere = atmosphere().ok_or("invalid atmosphere")?;
        let radius = R_EARTH + 250_000.0;
        let omega = atmosphere.angular_rate.radians_per_second();
        let drag = AtmosphericDrag {
            atmosphere,
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(0.02)
                .map_err(|_| "invalid area per mass")?,
        };
        let state = state_from_vectors(
            Vector3::new(radius, 0.0, 0.0),
            Vector3::new(0.0, omega * radius, 0.0),
        )
        .ok_or("invalid state")?;

        let acceleration = drag
            .acceleration_at(
                state,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?,
            )
            .map_err(|_| "invalid drag acceleration")?
            .vector();

        assert_close(acceleration.norm(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_rejects_nonfinite_acceleration() -> TestResult {
        let drag = AtmosphericDrag {
            atmosphere: ExponentialAtmosphere {
                reference_density: MassDensity::from_kilograms_per_cubic_meter(f64::MAX)
                    .map_err(|_| "invalid density")?,
                ..atmosphere().ok_or("invalid atmosphere")?
            },
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(f64::MAX)
                .map_err(|_| "invalid area per mass")?,
        };
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, f64::MAX, 0.0),
        )
        .ok_or("invalid state")?;

        assert!(matches!(
            drag.acceleration_at(
                state,
                Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?
            ),
            Err(DynamicsError::Domain(DomainError::NotFinite)),
        ));
        Ok(())
    }

    #[test]
    fn composite_dynamics_sums_component_accelerations() -> TestResult {
        let gravity = earth_gravity().ok_or("invalid gravity")?;
        let drag = drag_model(0.02).ok_or("invalid drag")?;
        let composite = CompositeDynamics {
            first: gravity,
            second: drag,
        };
        let state = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        )
        .ok_or("invalid state")?;
        let epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;

        let gravity_acceleration = composite
            .first
            .acceleration_at(state, epoch)
            .map_err(|_| "invalid gravity acceleration")?;
        let drag_acceleration = composite
            .second
            .acceleration_at(state, epoch)
            .map_err(|_| "invalid drag acceleration")?;
        let total = composite
            .acceleration_at(state, epoch)
            .map_err(|_| "invalid composite acceleration")?;

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
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid mu")?;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 250_000.0)
                .map_err(|_| "invalid radius")?,
            eccentricity: Eccentricity::from_scalar(0.0).map_err(|_| "invalid eccentricity")?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0).map_err(|_| "invalid phase")?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;
        let dynamics = CompositeDynamics {
            first: TwoBodyGravity { mu },
            second: drag_model(0.05).ok_or("invalid drag")?,
        };
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?
            .specific_energy
            .square_meters_per_square_second();
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0).map_err(|_| "invalid time step")?;
        let mut epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;

        for _ in 0..120 {
            integrator
                .step(&dynamics, &mut states.view_mut(), 0, epoch, dt)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(dt).map_err(|_| "invalid next epoch")?;
        }

        let final_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final diagnostics")?
            .specific_energy
            .square_meters_per_square_second();

        assert!(final_energy < initial_energy);
        Ok(())
    }

    #[test]
    fn drag_decay_energy_loss_matches_physical_expectation() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
            .map_err(|_| "invalid mu")?;
        let radius = R_EARTH + 250_000.0;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(radius).map_err(|_| "invalid radius")?,
            eccentricity: Eccentricity::from_scalar(0.0).map_err(|_| "invalid eccentricity")?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0).map_err(|_| "invalid phase")?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states
            .write_from_state(
                0,
                &orbit.to_cartesian_state().ok_or("invalid initial state")?,
            )
            .map_err(|_| "failed to write state")?;
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?
            .specific_energy
            .square_meters_per_square_second();
        let initial_altitude_m = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid initial diagnostics")?
            .radius
            .meters()
            - R_EARTH;
        let dynamics = CompositeDynamics {
            first: TwoBodyGravity { mu },
            second: drag_model(0.1).ok_or("invalid drag")?,
        };
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0).map_err(|_| "invalid time step")?;
        let mut epoch = Epoch::from_seconds(0.0).map_err(|_| "invalid epoch")?;
        let period_s = orbit.period().ok_or("invalid period")?.seconds();

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_s))
                .map_err(|_| "invalid step")?;
            integrator
                .step(&dynamics, &mut states.view_mut(), 0, epoch, step)
                .map_err(|_| "integration failed")?;
            epoch = epoch.advance_by(step).map_err(|_| "invalid next epoch")?;
        }

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)
            .ok_or("invalid final diagnostics")?;
        let final_energy = diagnostics
            .specific_energy
            .square_meters_per_square_second();
        let final_altitude_m = diagnostics.radius.meters() - R_EARTH;
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
