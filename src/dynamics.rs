use crate::errors::DynamicsError;
pub use crate::state::{CartesianAccelerationComponents, CartesianPositionComponents};
pub use crate::types::{Distance, Epoch, GravitationalParameter};

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

/// Computes acceleration for a state at a given simulation time.
pub trait Dynamics {
    /// Returns the acceleration at `position` and epoch `t`.
    fn acceleration_at(
        &self,
        position: CartesianPositionComponents,
        t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError>;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration_at(
        &self,
        position: CartesianPositionComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
        let px = position.x_meters();
        let py = position.y_meters();
        let pz = position.z_meters();
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();
        let mu = self.mu.meters_cubed_per_second_squared();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        debug_assert!(radius_m.is_finite());
        debug_assert!(radius_m > 0.0);
        debug_assert!(mu.is_finite());
        debug_assert!(mu > 0.0);

        let scale = -mu / (radius_sq_m * radius_m);
        CartesianAccelerationComponents::from_meters_per_second_squared(
            scale * px,
            scale * py,
            scale * pz,
        )
        .ok_or(DynamicsError::InvalidAcceleration)
    }
}

impl Dynamics for OblateBodyGravity {
    fn acceleration_at(
        &self,
        position: CartesianPositionComponents,
        _t: Epoch,
    ) -> Result<CartesianAccelerationComponents, DynamicsError> {
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

        debug_assert!(radius_m.is_finite());
        debug_assert!(radius_m > 0.0);
        debug_assert!(mu.is_finite());
        debug_assert!(mu > 0.0);
        debug_assert!(equatorial_radius.is_finite());
        debug_assert!(equatorial_radius > 0.0);

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
        .ok_or(DynamicsError::InvalidAcceleration)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::types::Position;
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
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        })
    }

    fn earth_oblate_gravity(j2: f64) -> Option<OblateBodyGravity> {
        Some(OblateBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            equatorial_radius: Distance::from_meters(R_EARTH)?,
            j2,
        })
    }

    #[test]
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(
                CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
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
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let acceleration = gravity
            .acceleration_at(
                CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
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
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let position = Vector3::new(3.0 * radius, 4.0 * radius, 0.0);
        let expected = -MU_EARTH * position / position.norm().powi(3);

        let acceleration = gravity
            .acceleration_at(
                CartesianPositionComponents::from_meters(position.x, position.y, position.z)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
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
                .ok_or("invalid earth mu")?,
        };
        let radius = R_EARTH + 400_000.0;
        let epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;
        let near_acceleration = gravity
            .acceleration_at(
                CartesianPositionComponents::from_meters(radius, 0.0, 0.0)
                    .ok_or("invalid near position")?,
                epoch,
            )
            .map_err(|_| "invalid near acceleration")?
            .vector()
            .norm();
        let far_acceleration = gravity
            .acceleration_at(
                CartesianPositionComponents::from_meters(2.0 * radius, 0.0, 0.0)
                    .ok_or("invalid far position")?,
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
        assert!(Position::from_vector(Vector3::new(f64::NAN, 0.0, 0.0)).is_none());
    }

    #[test]
    fn gravitational_parameter_rejects_zero() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(0.0).is_none());
    }

    #[test]
    fn gravitational_parameter_rejects_nan() {
        assert!(GravitationalParameter::from_meters_cubed_per_second_squared(f64::NAN).is_none());
    }

    #[test]
    fn two_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        assert!(matches!(
            gravity.acceleration_at(
                CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?
            ),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn two_body_gravity_rejects_nonfinite_acceleration() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)
                .ok_or("invalid earth mu")?,
        };
        assert!(matches!(
            gravity.acceleration_at(
                CartesianPositionComponents::from_meters(1.0e-150, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?
            ),
            Err(DynamicsError::InvalidAcceleration),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_with_zero_j2_matches_two_body_gravity() -> TestResult {
        let two_body = earth_gravity().ok_or("invalid two-body gravity")?;
        let oblate = earth_oblate_gravity(0.0).ok_or("invalid oblate gravity")?;
        let position =
            CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 200_000.0, 300_000.0)
                .ok_or("invalid position")?;
        let epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        let two_body_acceleration = two_body
            .acceleration_at(position, epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .vector();
        let oblate_acceleration = oblate
            .acceleration_at(position, epoch)
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
                .ok_or("invalid position")?;

        let acceleration = gravity
            .acceleration_at(position, Epoch::from_seconds(0.0).ok_or("invalid epoch")?)
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
                CartesianPositionComponents::from_meters(0.0, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
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
                CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 0.0, 0.0)
                    .ok_or("invalid position")?,
                Epoch::from_seconds(0.0).ok_or("invalid epoch")?,
            ),
            Err(DynamicsError::InvalidAcceleration),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_equatorial_j2_increases_inward_acceleration() -> TestResult {
        let two_body = earth_gravity().ok_or("invalid two-body gravity")?;
        let oblate = earth_oblate_gravity(J2_EARTH).ok_or("invalid oblate gravity")?;
        let position = CartesianPositionComponents::from_meters(R_EARTH + 400_000.0, 0.0, 0.0)
            .ok_or("invalid position")?;
        let epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        let two_body_x = two_body
            .acceleration_at(position, epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .x_meters_per_second_squared();
        let oblate_x = oblate
            .acceleration_at(position, epoch)
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
            .ok_or("invalid position")?;
        let epoch = Epoch::from_seconds(0.0).ok_or("invalid epoch")?;

        let two_body_z = two_body
            .acceleration_at(position, epoch)
            .map_err(|_| "invalid two-body acceleration")?
            .z_meters_per_second_squared();
        let oblate_z = oblate
            .acceleration_at(position, epoch)
            .map_err(|_| "invalid oblate acceleration")?
            .z_meters_per_second_squared();

        assert!(two_body_z < 0.0);
        assert!(oblate_z > two_body_z);
        Ok(())
    }
}
