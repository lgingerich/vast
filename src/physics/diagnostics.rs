use nalgebra::Vector3;

use super::state::{CartesianState, CartesianStates};
use super::types::{Distance, Eccentricity, GravitationalParameter, Position, Speed, Velocity};
use crate::errors::NumericalError;

/// Specific orbital energy, measured in square meters per square second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpecificEnergy(f64);

impl SpecificEnergy {
    /// Creates a specific energy from square meters per square second.
    ///
    /// Returns an error if the value is not finite.
    pub fn new(value: f64) -> Result<Self, NumericalError> {
        if value.is_finite() {
            Ok(Self(value))
        } else {
            Err(NumericalError::NotFinite {
                field: "specific energy",
            })
        }
    }

    /// Returns this specific energy in square meters per square second.
    #[inline]
    pub const fn square_meters_per_square_second(self) -> f64 {
        self.0
    }

    /// Absolute difference from another specific energy.
    ///
    /// The absolute value keeps the result non-negative so the constructor
    /// invariant holds even when the underlying `f64` subtraction crosses
    /// the zero boundary.
    pub fn abs_diff(self, other: SpecificEnergy) -> Self {
        let diff = (self.0 - other.0).abs();
        Self(diff)
    }
}

/// Specific angular momentum, measured in square meters per second.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpecificAngularMomentum(f64);

impl SpecificAngularMomentum {
    /// Creates a specific angular momentum from square meters per second.
    ///
    /// Returns an error if the value is not finite or negative.
    pub fn new(value: f64) -> Result<Self, NumericalError> {
        if !value.is_finite() {
            return Err(NumericalError::NotFinite {
                field: "specific angular momentum",
            });
        }
        if value < 0.0 {
            return Err(NumericalError::Negative {
                field: "specific angular momentum",
            });
        }
        Ok(Self(value))
    }

    /// Returns this specific angular momentum in square meters per second.
    #[inline]
    pub const fn square_meters_per_second(self) -> f64 {
        self.0
    }

    /// Absolute difference from another specific angular momentum.
    ///
    /// The absolute value keeps the result non-negative so the constructor
    /// invariant holds even when the underlying `f64` subtraction crosses
    /// the zero boundary.
    pub fn abs_diff(self, other: SpecificAngularMomentum) -> Self {
        let diff = (self.0 - other.0).abs();
        Self(diff)
    }
}

/// Two-body orbital diagnostics for a Cartesian state.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct OrbitalDiagnostics {
    /// Current radius from the central body's center of mass.
    radius: Distance,
    /// Current inertial speed.
    speed: Speed,
    /// Specific orbital energy.
    specific_energy: SpecificEnergy,
    /// Specific angular momentum vector.
    specific_angular_momentum_vector: Vector3<f64>,
    /// Specific angular momentum magnitude.
    specific_angular_momentum: SpecificAngularMomentum,
    /// Eccentricity vector.
    eccentricity_vector: Vector3<f64>,
    /// Eccentricity magnitude.
    eccentricity: Eccentricity,
    /// Semi-major axis for bound elliptical orbits.
    semi_major_axis: Option<Distance>,
    /// Periapsis radius for bound elliptical orbits.
    periapsis_radius: Option<Distance>,
    /// Apoapsis radius for bound elliptical orbits.
    apoapsis_radius: Option<Distance>,
}

impl OrbitalDiagnostics {
    /// Computes two-body diagnostics from an inertial Cartesian state.
    ///
    /// Returns an error if the position is at the origin or a computed
    /// quantity is non-finite.
    pub fn from_state(
        state: &CartesianState,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
        Self::from_components(state.position, state.velocity, mu)
    }

    /// Computes two-body diagnostics from an indexed inertial Cartesian state view.
    pub fn from_states(
        states: &CartesianStates<'_>,
        index: usize,
        mu: GravitationalParameter,
    ) -> Result<Self, crate::errors::StateError> {
        let components = states.components(index)?;
        Ok(Self::from_components(
            Position::from_vector(components.position_vector())?,
            Velocity::from_vector(components.velocity_vector())?,
            mu,
        )?)
    }

    pub(crate) fn from_components(
        position: Position,
        velocity: Velocity,
        mu: GravitationalParameter,
    ) -> Result<Self, NumericalError> {
        let position = position.vector();
        let velocity = velocity.vector();
        let mu_f64 = mu.meters_cubed_per_second_squared();
        let radius = position.norm();
        let speed = velocity.norm();

        if radius == 0.0 {
            return Err(NumericalError::ZeroNorm { field: "position" });
        }

        let specific_energy_f64 = 0.5 * speed * speed - mu_f64 / radius;
        let specific_energy = SpecificEnergy::new(specific_energy_f64)?;
        let specific_angular_momentum_vector = position.cross(&velocity);
        let specific_angular_momentum = specific_angular_momentum_vector.norm();
        let eccentricity_vector =
            velocity.cross(&specific_angular_momentum_vector) / mu_f64 - position / radius;
        let eccentricity = eccentricity_vector.norm();

        let semi_major_axis = bound_semi_major_axis(mu, specific_energy)?;
        let periapsis_radius = match semi_major_axis {
            Some(axis) => {
                let axis_m = axis.meters();
                Some(Distance::new(axis_m * (1.0 - eccentricity))?)
            }
            None => None,
        };
        let apoapsis_radius = match semi_major_axis {
            Some(axis) => {
                let axis_m = axis.meters();
                Some(Distance::new(axis_m * (1.0 + eccentricity))?)
            }
            None => None,
        };

        Ok(Self {
            radius: Distance::new(radius)?,
            speed: Speed::new(speed)?,
            specific_energy,
            specific_angular_momentum_vector,
            specific_angular_momentum: SpecificAngularMomentum::new(specific_angular_momentum)?,
            eccentricity_vector,
            eccentricity: Eccentricity::new(eccentricity)?,
            semi_major_axis,
            periapsis_radius,
            apoapsis_radius,
        })
    }

    /// Returns the current radius from the central body's center of mass.
    #[inline]
    pub const fn radius(&self) -> Distance {
        self.radius
    }

    /// Returns the current inertial speed.
    #[inline]
    pub const fn speed(&self) -> Speed {
        self.speed
    }

    /// Returns the specific orbital energy.
    #[inline]
    pub const fn specific_energy(&self) -> SpecificEnergy {
        self.specific_energy
    }

    /// Returns the specific angular momentum vector.
    #[inline]
    pub fn specific_angular_momentum_vector(&self) -> &Vector3<f64> {
        &self.specific_angular_momentum_vector
    }

    /// Returns the specific angular momentum magnitude.
    #[inline]
    pub const fn specific_angular_momentum(&self) -> SpecificAngularMomentum {
        self.specific_angular_momentum
    }

    /// Returns the eccentricity vector.
    #[inline]
    pub fn eccentricity_vector(&self) -> &Vector3<f64> {
        &self.eccentricity_vector
    }

    /// Returns the eccentricity magnitude.
    #[inline]
    pub const fn eccentricity(&self) -> Eccentricity {
        self.eccentricity
    }

    /// Returns the semi-major axis for bound elliptical orbits.
    #[inline]
    pub const fn semi_major_axis(&self) -> Option<Distance> {
        self.semi_major_axis
    }

    /// Returns the periapsis radius for bound elliptical orbits.
    #[inline]
    pub const fn periapsis_radius(&self) -> Option<Distance> {
        self.periapsis_radius
    }

    /// Returns the apoapsis radius for bound elliptical orbits.
    #[inline]
    pub const fn apoapsis_radius(&self) -> Option<Distance> {
        self.apoapsis_radius
    }
}

/// Diagnostic metric available in a [`DiagnosticSeries`].
///
/// Values are in SI units (meters, seconds, m²/s², etc.).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Column {
    /// Radius from the central body center of mass, meters.
    Radius,
    /// Inertial speed, m/s.
    Speed,
    /// Specific orbital energy, m²/s².
    SpecificEnergy,
    /// Specific angular momentum magnitude, m²/s.
    SpecificAngularMomentum,
    /// Orbital eccentricity, dimensionless.
    Eccentricity,
    /// Semi-major axis, meters. `NaN` for unbound orbits.
    SemiMajorAxis,
}

/// Per-step diagnostic scalars collected across a trajectory.
///
/// Each column is a dense array with one entry per snapshot.
/// For the initial+final snapshots from [`crate::SimResult`], `len()` is 2.
/// Future per-step recording paths can produce series of arbitrary length.
///
/// Values are stored in SI units so callers can compute differences,
/// variance, and other reductions without boxing through domain newtypes.
#[derive(Debug, Clone, PartialEq)]
pub struct DiagnosticSeries {
    radius: Vec<f64>,
    speed: Vec<f64>,
    specific_energy: Vec<f64>,
    specific_angular_momentum: Vec<f64>,
    eccentricity: Vec<f64>,
    semi_major_axis: Vec<f64>,
}

impl DiagnosticSeries {
    /// Creates a series from a pair of snapshots (initial, final).
    pub fn from_pair(initial: &OrbitalDiagnostics, terminal: &OrbitalDiagnostics) -> Self {
        let sma =
            |d: &OrbitalDiagnostics| d.semi_major_axis.map(|a| a.meters()).unwrap_or(f64::NAN);
        Self {
            radius: vec![initial.radius.meters(), terminal.radius.meters()],
            speed: vec![
                initial.speed.meters_per_second(),
                terminal.speed.meters_per_second(),
            ],
            specific_energy: vec![
                initial.specific_energy.square_meters_per_square_second(),
                terminal.specific_energy.square_meters_per_square_second(),
            ],
            specific_angular_momentum: vec![
                initial.specific_angular_momentum.square_meters_per_second(),
                terminal
                    .specific_angular_momentum
                    .square_meters_per_second(),
            ],
            eccentricity: vec![
                initial.eccentricity.scalar(),
                terminal.eccentricity.scalar(),
            ],
            semi_major_axis: vec![sma(initial), sma(terminal)],
        }
    }

    /// Number of snapshots in the series.
    pub fn len(&self) -> usize {
        self.radius.len()
    }

    /// Returns `true` if the series is empty.
    pub fn is_empty(&self) -> bool {
        self.radius.is_empty()
    }

    /// Returns the values for a diagnostic column.
    ///
    /// # Examples
    ///
    /// ```
    /// # use vast::*;
    /// # fn main() -> Result<(), Box<dyn std::error::Error>> {
    /// # let body = CentralBody::earth()?;
    /// # let result = SimBuilder::new(body)
    /// #     .orbit(Orbit::circular_inclined(
    /// #         Distance::new((6678.0) * 1000.0)?,
    /// #         Angle::from_degrees(51.6)?,
    /// #         Angle::from_degrees(32.0)?,
    /// #     ))?
    /// #     .dynamics(TwoBodyGravity::from_body(&body))
    /// #     .integrator(VelocityVerletIntegrator)
    /// #     .timestep(TimeStep::new(10.0)?)
    /// #     .duration(Duration::new(5_330.0).unwrap())?;
    /// let s = &result.series;
    /// let radius = s.column(Column::Radius);
    /// let drift = (radius[1] - radius[0]).abs();
    /// # Ok(())
    /// # }
    /// ```
    pub fn column(&self, column: Column) -> &[f64] {
        match column {
            Column::Radius => &self.radius,
            Column::Speed => &self.speed,
            Column::SpecificEnergy => &self.specific_energy,
            Column::SpecificAngularMomentum => &self.specific_angular_momentum,
            Column::Eccentricity => &self.eccentricity,
            Column::SemiMajorAxis => &self.semi_major_axis,
        }
    }
}

fn bound_semi_major_axis(
    mu: GravitationalParameter,
    specific_energy: SpecificEnergy,
) -> Result<Option<Distance>, NumericalError> {
    let specific_energy = specific_energy.square_meters_per_square_second();
    if specific_energy >= 0.0 {
        return Ok(None);
    }

    let mu_f64 = mu.meters_cubed_per_second_squared();
    Ok(Some(Distance::new(-mu_f64 / (2.0 * specific_energy))?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        Angle, CartesianStateBuffer, Eccentricity, EllipticOrbit, MU_EARTH, OrbitDirection,
        Position, R_EARTH, Velocity,
    };

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const ALTITUDE_M: f64 = 400_000.0;
    const EPS: f64 = 1.0e-6;

    fn circular_orbit() -> Result<EllipticOrbit, Box<dyn std::error::Error>> {
        Ok(EllipticOrbit::new(
            Distance::new(R_EARTH + ALTITUDE_M)?,
            Eccentricity::new(0.0)?,
            earth_mu()?,
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            Angle::new(0.0)?,
            OrbitDirection::Prograde,
        )?)
    }

    fn earth_mu() -> Result<GravitationalParameter, Box<dyn std::error::Error>> {
        Ok(GravitationalParameter::new(MU_EARTH)?)
    }

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    #[test]
    fn circular_orbit_diagnostics_match_two_body_invariants() -> TestResult {
        let orbit = circular_orbit()?;
        let state = orbit.to_cartesian_state()?;
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        let diagnostics = OrbitalDiagnostics::from_states(&states.view(), 0, orbit.mu())?;

        let radius = orbit.semi_major_axis().meters();
        let speed = orbit.speed()?.meters_per_second();

        assert_close(diagnostics.radius.meters(), radius, EPS);
        assert_close(diagnostics.speed.meters_per_second(), speed, EPS);
        assert_close(
            diagnostics
                .specific_energy
                .square_meters_per_square_second(),
            -MU_EARTH / (2.0 * radius),
            EPS,
        );
        assert_close(
            diagnostics
                .specific_angular_momentum
                .square_meters_per_second(),
            radius * speed,
            EPS,
        );
        assert_close(diagnostics.eccentricity.scalar(), 0.0, 1.0e-12);
        assert_close(
            diagnostics
                .semi_major_axis
                .ok_or("missing semi-major axis")?
                .meters(),
            radius,
            EPS,
        );
        assert_close(
            diagnostics
                .periapsis_radius
                .ok_or("missing periapsis radius")?
                .meters(),
            radius,
            EPS,
        );
        assert_close(
            diagnostics
                .apoapsis_radius
                .ok_or("missing apoapsis radius")?
                .meters(),
            radius,
            EPS,
        );
        Ok(())
    }

    #[test]
    fn diagnostics_reject_zero_radius() -> TestResult {
        let state = CartesianState::new(
            Position::from_vector(Vector3::zeros())?,
            Velocity::from_vector(Vector3::zeros())?,
        );
        let mut states = CartesianStateBuffer::<1>::with_len(1)?;
        states.write_from_state(0, &state)?;

        assert!(OrbitalDiagnostics::from_states(&states.view(), 0, earth_mu()?).is_err());
        Ok(())
    }

    #[test]
    fn hyperbolic_state_has_no_bound_orbit_radii() -> TestResult {
        let radius = R_EARTH + ALTITUDE_M;
        let state = CartesianState::new(
            Position::from_vector(Vector3::new(radius, 0.0, 0.0))?,
            Velocity::from_vector(Vector3::new(0.0, 20_000.0, 0.0))?,
        );

        let diagnostics = OrbitalDiagnostics::from_state(&state, earth_mu()?)?;

        assert!(diagnostics.semi_major_axis.is_none());
        assert!(diagnostics.periapsis_radius.is_none());
        assert!(diagnostics.apoapsis_radius.is_none());
        Ok(())
    }

    #[test]
    fn diagnostics_reject_invalid_state_index() -> TestResult {
        let states = CartesianStateBuffer::<1>::with_len(1)?;

        assert!(OrbitalDiagnostics::from_states(&states.view(), 1, earth_mu()?).is_err());
        Ok(())
    }
}
