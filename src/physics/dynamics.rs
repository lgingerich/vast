pub use super::types::{
    AngularRate, AreaPerMass, Distance, Epoch, GravitationalParameter, MassDensity,
};
use crate::errors::{DynamicsError, NumericalError};
use nalgebra::Vector3;

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
    /// Returns the acceleration for `position` and `velocity` at epoch `t`.
    fn acceleration_at(
        &self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        t: Epoch,
    ) -> Result<Vector3<f64>, DynamicsError>;
}

impl Dynamics for TwoBodyGravity {
    fn acceleration_at(
        &self,
        position: Vector3<f64>,
        _velocity: Vector3<f64>,
        _t: Epoch,
    ) -> Result<Vector3<f64>, DynamicsError> {
        let px = position.x;
        let py = position.y;
        let pz = position.z;
        let radius_sq_m = px * px + py * py + pz * pz;
        let radius_m = radius_sq_m.sqrt();
        let mu = self.mu.meters_cubed_per_second_squared();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let scale = -mu / (radius_sq_m * radius_m);
        Ok(Vector3::new(scale * px, scale * py, scale * pz))
    }
}

impl Dynamics for OblateBodyGravity {
    fn acceleration_at(
        &self,
        position: Vector3<f64>,
        _velocity: Vector3<f64>,
        _t: Epoch,
    ) -> Result<Vector3<f64>, DynamicsError> {
        let px = position.x;
        let py = position.y;
        let pz = position.z;
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

        Ok(Vector3::new(
            central_scale * px + xy_j2_scale * px,
            central_scale * py + xy_j2_scale * py,
            central_scale * pz + z_j2_scale * pz,
        ))
    }
}

impl ExponentialAtmosphere {
    /// Computes atmospheric density at `radius`.
    ///
    /// Returns an error if the computed density is non-finite.
    pub fn density_at_radius(&self, radius: Distance) -> Result<MassDensity, NumericalError> {
        MassDensity::from_kilograms_per_cubic_meter(self.density_at_radius_m(radius.meters()))
    }

    fn density_at_radius_m(&self, radius_m: f64) -> f64 {
        let altitude_m = radius_m - self.reference_radius.meters();
        let reference_altitude_m = self.reference_altitude.meters();
        self.reference_density.kilograms_per_cubic_meter()
            * (-(altitude_m - reference_altitude_m) / self.scale_height.meters()).exp()
    }
}

impl Dynamics for AtmosphericDrag {
    fn acceleration_at(
        &self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        _t: Epoch,
    ) -> Result<Vector3<f64>, DynamicsError> {
        let radius_m = position.norm();

        if radius_m == 0.0 {
            return Err(DynamicsError::PositionAtGravityCenter);
        }

        let density = self.atmosphere.density_at_radius_m(radius_m);
        let omega = self.atmosphere.angular_rate.radians_per_second();
        let atmosphere_velocity = Vector3::new(-omega * position.y, omega * position.x, 0.0);
        let relative_velocity = velocity - atmosphere_velocity;
        let relative_speed = relative_velocity.norm();
        let scale =
            -0.5 * density * self.area_per_mass.square_meters_per_kilogram() * relative_speed;

        Ok(scale * relative_velocity)
    }
}

impl<A: Dynamics, B: Dynamics> Dynamics for CompositeDynamics<A, B> {
    fn acceleration_at(
        &self,
        position: Vector3<f64>,
        velocity: Vector3<f64>,
        t: Epoch,
    ) -> Result<Vector3<f64>, DynamicsError> {
        let first = self.first.acceleration_at(position, velocity, t)?;
        let second = self.second.acceleration_at(position, velocity, t)?;
        Ok(first + second)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{J2_EARTH, MU_EARTH, R_EARTH};
    use crate::physics::diagnostics::OrbitalDiagnostics;
    use crate::physics::integrators::{Integrator, RungeKutta4Integrator};
    use crate::physics::orbits::{EllipticOrbit, OrbitDirection};
    use crate::physics::state::CartesianStateBuffer;
    use crate::physics::types::{
        Angle, Distance, Eccentricity, GravitationalParameter, Position, TimeStep,
    };
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
        Ok(TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        })
    }

    fn earth_oblate_gravity(j2: f64) -> Result<OblateBodyGravity, Box<dyn std::error::Error>> {
        Ok(OblateBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
            equatorial_radius: Distance::from_meters(R_EARTH)?,
            j2,
        })
    }

    fn atmosphere() -> Result<ExponentialAtmosphere, Box<dyn std::error::Error>> {
        Ok(ExponentialAtmosphere {
            reference_radius: Distance::from_meters(R_EARTH)?,
            reference_altitude: Distance::from_meters(250_000.0)?,
            reference_density: MassDensity::from_kilograms_per_cubic_meter(1.0e-10)?,
            scale_height: Distance::from_meters(50_000.0)?,
            angular_rate: AngularRate::from_radians_per_second(7.292_115_9e-5)?,
        })
    }

    fn drag_model(area_per_mass: f64) -> Result<AtmosphericDrag, Box<dyn std::error::Error>> {
        Ok(AtmosphericDrag {
            atmosphere: atmosphere()?,
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(area_per_mass)?,
        })
    }

    fn state_from_position(position: Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
        (position, Vector3::zeros())
    }

    fn state_from_vectors(
        position: Vector3<f64>,
        velocity: Vector3<f64>,
    ) -> (Vector3<f64>, Vector3<f64>) {
        (position, velocity)
    }

    #[test]
    fn two_body_gravity_points_toward_origin() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let radius = R_EARTH + 400_000.0;
        let (position, velocity) = state_from_position(Vector3::new(radius, 0.0, 0.0));
        let acceleration =
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert!(acceleration.x < 0.0);
        assert_close(acceleration.y, 0.0, EPS);
        assert_close(acceleration.z, 0.0, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_has_inverse_square_magnitude() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let radius = R_EARTH + 400_000.0;
        let (position, velocity) = state_from_position(Vector3::new(radius, 0.0, 0.0));
        let acceleration =
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert_close(acceleration.norm(), MU_EARTH / radius.powi(2), EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_matches_off_axis_vector_formula() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let radius = R_EARTH + 400_000.0;
        let position = Vector3::new(3.0 * radius, 4.0 * radius, 0.0);
        let expected = -MU_EARTH * position / position.norm().powi(3);
        let (position, velocity) = state_from_position(position);

        let acceleration =
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert_close(acceleration.x, expected.x, EPS);
        assert_close(acceleration.y, expected.y, EPS);
        assert_close(acceleration.z, expected.z, EPS);
        Ok(())
    }

    #[test]
    fn two_body_gravity_falls_off_with_inverse_square_distance() -> TestResult {
        let gravity = TwoBodyGravity {
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let radius = R_EARTH + 400_000.0;
        let epoch = Epoch::from_seconds(0.0)?;
        let (near_position, near_velocity) = state_from_position(Vector3::new(radius, 0.0, 0.0));
        let near_acceleration = gravity
            .acceleration_at(near_position, near_velocity, epoch)?
            .norm();
        let (far_position, far_velocity) =
            state_from_position(Vector3::new(2.0 * radius, 0.0, 0.0));
        let far_acceleration = gravity
            .acceleration_at(far_position, far_velocity, epoch)?
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
            mu: GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?,
        };
        let (position, velocity) = state_from_position(Vector3::zeros());
        assert!(matches!(
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_with_zero_j2_matches_two_body_gravity() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(0.0)?;
        let position = Vector3::new(R_EARTH + 400_000.0, 200_000.0, 300_000.0);
        let (position, velocity) = state_from_position(position);
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_acceleration = two_body.acceleration_at(position, velocity, epoch)?;
        let oblate_acceleration = oblate.acceleration_at(position, velocity, epoch)?;

        assert_close(oblate_acceleration.x, two_body_acceleration.x, EPS);
        assert_close(oblate_acceleration.y, two_body_acceleration.y, EPS);
        assert_close(oblate_acceleration.z, two_body_acceleration.z, EPS);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_is_finite_for_inclined_leo_position() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH)?;
        let (position, velocity) =
            state_from_position(Vector3::new(R_EARTH + 400_000.0, 500_000.0, 1_000_000.0));

        let acceleration =
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert!(acceleration.x.is_finite());
        assert!(acceleration.y.is_finite());
        assert!(acceleration.z.is_finite());
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_rejects_zero_radius() -> TestResult {
        let gravity = earth_oblate_gravity(J2_EARTH)?;
        let (position, velocity) = state_from_position(Vector3::zeros());

        assert!(matches!(
            gravity.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?),
            Err(DynamicsError::PositionAtGravityCenter),
        ));
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_equatorial_j2_increases_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(J2_EARTH)?;
        let (position, velocity) = state_from_position(Vector3::new(R_EARTH + 400_000.0, 0.0, 0.0));
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_x = two_body.acceleration_at(position, velocity, epoch)?.x;
        let oblate_x = oblate.acceleration_at(position, velocity, epoch)?.x;

        assert!(two_body_x < 0.0);
        assert!(oblate_x < two_body_x);
        Ok(())
    }

    #[test]
    fn oblate_body_gravity_polar_axis_j2_reduces_inward_acceleration() -> TestResult {
        let two_body = earth_gravity()?;
        let oblate = earth_oblate_gravity(J2_EARTH)?;
        let (position, velocity) = state_from_position(Vector3::new(0.0, 0.0, R_EARTH + 400_000.0));
        let epoch = Epoch::from_seconds(0.0)?;

        let two_body_z = two_body.acceleration_at(position, velocity, epoch)?.z;
        let oblate_z = oblate.acceleration_at(position, velocity, epoch)?.z;

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
        let drag = AtmosphericDrag {
            atmosphere: ExponentialAtmosphere {
                angular_rate: AngularRate::from_radians_per_second(0.0)?,
                ..atmosphere()?
            },
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        };
        let (position, velocity) = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );

        let acceleration = drag.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert!(acceleration.dot(&velocity) < 0.0);
        Ok(())
    }

    #[test]
    fn atmospheric_drag_is_zero_when_relative_velocity_is_zero() -> TestResult {
        let atmosphere = atmosphere()?;
        let radius = R_EARTH + 250_000.0;
        let omega = atmosphere.angular_rate.radians_per_second();
        let drag = AtmosphericDrag {
            atmosphere,
            area_per_mass: AreaPerMass::from_square_meters_per_kilogram(0.02)?,
        };
        let (position, velocity) = state_from_vectors(
            Vector3::new(radius, 0.0, 0.0),
            Vector3::new(0.0, omega * radius, 0.0),
        );

        let acceleration = drag.acceleration_at(position, velocity, Epoch::from_seconds(0.0)?)?;

        assert_close(acceleration.norm(), 0.0, EPS);
        Ok(())
    }

    #[test]
    fn composite_dynamics_sums_component_accelerations() -> TestResult {
        let gravity = earth_gravity()?;
        let drag = drag_model(0.02)?;
        let composite = CompositeDynamics {
            first: gravity,
            second: drag,
        };
        let (position, velocity) = state_from_vectors(
            Vector3::new(R_EARTH + 250_000.0, 0.0, 0.0),
            Vector3::new(0.0, 7_700.0, 0.0),
        );
        let epoch = Epoch::from_seconds(0.0)?;

        let gravity_acceleration = composite.first.acceleration_at(position, velocity, epoch)?;
        let drag_acceleration = composite
            .second
            .acceleration_at(position, velocity, epoch)?;
        let total = composite.acceleration_at(position, velocity, epoch)?;

        assert_close(total.x, gravity_acceleration.x + drag_acceleration.x, EPS);
        assert_close(total.y, gravity_acceleration.y + drag_acceleration.y, EPS);
        assert_close(total.z, gravity_acceleration.z + drag_acceleration.z, EPS);
        Ok(())
    }

    #[test]
    fn low_orbit_with_drag_loses_specific_energy() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(R_EARTH + 250_000.0)?,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let dynamics = CompositeDynamics {
            first: TwoBodyGravity { mu },
            second: drag_model(0.05)?,
        };
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy
            .square_meters_per_square_second();
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let mut epoch = Epoch::from_seconds(0.0)?;

        for _ in 0..120 {
            integrator.step(&dynamics, &mut states.view_mut(), 0, epoch, dt)?;
            epoch = epoch.advance_by(dt)?;
        }

        let final_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy
            .square_meters_per_square_second();

        assert!(final_energy < initial_energy);
        Ok(())
    }

    #[test]
    fn drag_decay_energy_loss_matches_physical_expectation() -> TestResult {
        let mu = GravitationalParameter::from_meters_cubed_per_second_squared(MU_EARTH)?;
        let radius = R_EARTH + 250_000.0;
        let orbit = EllipticOrbit {
            semi_major_axis: Distance::from_meters(radius)?,
            eccentricity: Eccentricity::from_scalar(0.0)?,
            mu,
            inclination: Angle::ZERO,
            right_ascension_of_ascending_node: Angle::ZERO,
            argument_of_periapsis: Angle::ZERO,
            true_anomaly: Angle::from_radians(0.0)?,
            direction: OrbitDirection::Prograde,
        };
        let mut states = CartesianStateBuffer::<1>::new();
        states.write_from_state(0, &orbit.to_cartesian_state()?)?;
        let initial_energy = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .specific_energy
            .square_meters_per_square_second();
        let initial_altitude_m = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?
            .radius
            .meters()
            - R_EARTH;
        let dynamics = CompositeDynamics {
            first: TwoBodyGravity { mu },
            second: drag_model(0.1)?,
        };
        let integrator = RungeKutta4Integrator;
        let dt = TimeStep::from_seconds(10.0)?;
        let mut epoch = Epoch::from_seconds(0.0)?;
        let period_s = orbit.period()?.seconds();

        while epoch.seconds() < period_s {
            let remaining_s = period_s - epoch.seconds();
            let step = TimeStep::from_seconds(dt.seconds().min(remaining_s))?;
            integrator.step(&dynamics, &mut states.view_mut(), 0, epoch, step)?;
            epoch = epoch.advance_by(step)?;
        }

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, mu)?;
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
