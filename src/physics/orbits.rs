use super::elements::EllipticElementSet;
use super::frame::Frame;
use super::state::CartesianState;
use super::types::{Angle, Distance, Duration, Eccentricity, Speed};
use crate::errors::StateError;

/// Keplerian (classical) two-body orbit around a central body.
///
/// `Orbit` is the single public orbit specification type. The geometric degrees
/// of freedom are stored according to the orbital regime; the central body's
/// gravitational parameter and equatorial radius are supplied when the spec is
/// converted to a Cartesian state or queried for derived quantities.
///
/// All variants represent bound orbits with eccentricity in `[0, 1)`. Use
/// [`Orbit::elliptic`] for the fully general case, or the circular convenience
/// constructors for equatorial and inclined orbits.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Orbit {
    /// Circular equatorial orbit.
    CircularEquatorial {
        /// Altitude above the central body's equatorial surface.
        altitude: Distance,
    },
    /// Circular inclined orbit.
    CircularInclined {
        /// Altitude above the central body's equatorial surface.
        altitude: Distance,
        /// Inclination from the equatorial plane.
        inclination: Angle,
        /// Right ascension of the ascending node.
        raan: Angle,
    },
    /// Elliptic orbit with all classical elements.
    Elliptic {
        /// Semi-major axis.
        semi_major_axis: Distance,
        /// Orbital eccentricity (0 ≤ e < 1).
        eccentricity: Eccentricity,
        /// Inclination from the equatorial plane.
        inclination: Angle,
        /// Right ascension of the ascending node.
        raan: Angle,
        /// Argument of periapsis.
        arg_periapsis: Angle,
        /// True anomaly at epoch.
        true_anomaly: Angle,
    },
}

impl Orbit {
    // ---------------- Constructors ----------------
    /// Creates a circular equatorial orbit specification at the given altitude
    /// above a central body.
    pub fn circular_equatorial(altitude: Distance) -> Self {
        Self::CircularEquatorial { altitude }
    }

    /// Creates a circular inclined orbit specification at the given altitude
    /// above a central body.
    pub fn circular_inclined(altitude: Distance, inclination: Angle, raan: Angle) -> Self {
        Self::CircularInclined {
            altitude,
            inclination,
            raan,
        }
    }

    /// Creates an orbit specification with all classical Keplerian elements.
    pub fn elliptic(
        semi_major_axis: Distance,
        eccentricity: Eccentricity,
        inclination: Angle,
        raan: Angle,
        arg_periapsis: Angle,
        true_anomaly: Angle,
    ) -> Self {
        Self::Elliptic {
            semi_major_axis,
            eccentricity,
            inclination,
            raan,
            arg_periapsis,
            true_anomaly,
        }
    }

    // ---------------- Public Getters ----------------
    /// Returns the orbital eccentricity.
    pub const fn eccentricity(&self) -> Eccentricity {
        match *self {
            Self::CircularEquatorial { .. } | Self::CircularInclined { .. } => {
                Eccentricity::CIRCULAR
            }
            Self::Elliptic { eccentricity, .. } => eccentricity,
        }
    }

    /// Returns the inclination.
    pub const fn inclination(&self) -> Angle {
        match *self {
            Self::CircularEquatorial { .. } => Angle::ZERO,
            Self::CircularInclined { inclination, .. } | Self::Elliptic { inclination, .. } => {
                inclination
            }
        }
    }

    /// Returns the right ascension of the ascending node.
    ///
    /// For circular equatorial orbits, where RAAN is undefined, this returns
    /// `Angle::ZERO` by convention.
    pub const fn raan(&self) -> Angle {
        match *self {
            Self::CircularEquatorial { .. } => Angle::ZERO,
            Self::CircularInclined { raan, .. } | Self::Elliptic { raan, .. } => raan,
        }
    }

    /// Returns the argument of periapsis.
    ///
    /// For circular orbits, where the argument of periapsis is undefined,
    /// this returns `Angle::ZERO` by convention.
    pub const fn argument_of_periapsis(&self) -> Angle {
        match *self {
            Self::CircularEquatorial { .. } | Self::CircularInclined { .. } => Angle::ZERO,
            Self::Elliptic { arg_periapsis, .. } => arg_periapsis,
        }
    }

    /// Returns the true anomaly.
    ///
    /// For circular orbits, where true anomaly is not an independent element,
    /// this returns `Angle::ZERO` by convention.
    pub const fn true_anomaly(&self) -> Angle {
        match *self {
            Self::CircularEquatorial { .. } | Self::CircularInclined { .. } => Angle::ZERO,
            Self::Elliptic { true_anomaly, .. } => true_anomaly,
        }
    }

    // ---------------- Public Methods ----------------

    /// Converts this orbital specification into an inertial Cartesian state
    /// around the given central body.
    ///
    /// Returns an error if the orbital parameters are physically invalid or a
    /// computed quantity is non-finite.
    pub fn to_cartesian_state(&self, frame: &Frame) -> Result<CartesianState, StateError> {
        let mu = frame.mu()?;
        let equatorial_radius = frame.equatorial_radius()?;
        frame.require_inertial_axes()?;

        match *self {
            Self::CircularEquatorial { altitude } => {
                let element_set = EllipticElementSet::new(
                    equatorial_radius + altitude,
                    Eccentricity::CIRCULAR,
                    mu,
                    Angle::ZERO,
                    Angle::ZERO,
                    Angle::ZERO,
                    Angle::ZERO,
                )?;
                element_set.to_cartesian_state()
            }
            Self::CircularInclined {
                altitude,
                inclination,
                raan,
            } => {
                let element_set = EllipticElementSet::new(
                    equatorial_radius + altitude,
                    Eccentricity::CIRCULAR,
                    mu,
                    inclination,
                    raan,
                    Angle::ZERO,
                    Angle::ZERO,
                )?;
                element_set.to_cartesian_state()
            }
            Self::Elliptic {
                semi_major_axis,
                eccentricity,
                inclination,
                raan,
                arg_periapsis,
                true_anomaly,
            } => {
                let element_set = EllipticElementSet::new(
                    semi_major_axis,
                    eccentricity,
                    mu,
                    inclination,
                    raan,
                    arg_periapsis,
                    true_anomaly,
                )?;
                element_set.to_cartesian_state()
            }
        }
    }

    /// Instantaneous orbital speed at the current true anomaly.
    ///
    /// Returns an error if the frame has no central body or the computed speed
    /// is non-finite.
    pub fn speed(&self, frame: &Frame) -> Result<Speed, StateError> {
        let mu = frame.mu()?.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis(frame)?.meters();
        let eccentricity = self.eccentricity().scalar();
        let true_anomaly = self.true_anomaly().radians();

        let radius = semi_major_axis * (1.0 - eccentricity * eccentricity)
            / (1.0 + eccentricity * true_anomaly.cos());

        Ok(Speed::new(
            (mu * (2.0 / radius - 1.0 / semi_major_axis)).sqrt(),
        ))
    }

    /// Orbital period from Kepler's third law.
    ///
    /// Returns an error if the frame has no central body or the computed
    /// period is non-finite.
    pub fn period(&self, frame: &Frame) -> Result<Duration, StateError> {
        let mu = frame.mu()?.meters_cubed_per_second_squared();
        let semi_major_axis = self.semi_major_axis(frame)?.meters();

        Ok(Duration::new(
            2.0 * std::f64::consts::PI * (semi_major_axis.powi(3) / mu).sqrt(),
        ))
    }

    /// Returns the semi-major axis for this orbit in the given frame.
    ///
    /// Circular variants compute the semi-major axis as
    /// `body.equatorial_radius() + altitude`. Returns an error if the frame
    /// has no central body (e.g. a barycenter frame).
    pub fn semi_major_axis(&self, frame: &Frame) -> Result<Distance, StateError> {
        match *self {
            Self::CircularEquatorial { altitude } | Self::CircularInclined { altitude, .. } => {
                Ok(frame.equatorial_radius()? + altitude)
            }
            Self::Elliptic {
                semi_major_axis, ..
            } => Ok(semi_major_axis),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::physics::constants::{MU_EARTH, R_EARTH};
    use crate::physics::frame::Frame;

    type TestResult = Result<(), Box<dyn std::error::Error>>;

    const EPS: f64 = 1.0e-9;

    fn assert_close(actual: f64, expected: f64, tolerance: f64) {
        assert!(
            (actual - expected).abs() <= tolerance,
            "expected {actual} to be within {tolerance} of {expected}",
        );
    }

    fn frame() -> Frame {
        Frame::earth_inertial()
    }

    fn elliptic_orbit(
        eccentricity: Eccentricity,
        true_anomaly: Angle,
    ) -> Result<Orbit, Box<dyn std::error::Error>> {
        Ok(Orbit::elliptic(
            Distance::new(R_EARTH + 1_000_000.0),
            eccentricity,
            Angle::new(0.0),
            Angle::new(0.0),
            Angle::new(0.0),
            true_anomaly,
        ))
    }

    #[test]
    fn elliptic_orbit_speed_at_periapsis_matches_vis_viva() -> TestResult {
        let frame = frame();
        let e = Eccentricity::new(0.3);
        let orbit = elliptic_orbit(e, Angle::new(0.0))?;
        let a = orbit.semi_major_axis(&frame)?.meters();
        let mu = MU_EARTH;
        let r_p = a * (1.0 - e.scalar());
        let expected = (mu * (2.0 / r_p - 1.0 / a)).sqrt();

        assert_close(orbit.speed(&frame)?.meters_per_second(), expected, EPS);
        Ok(())
    }

    #[test]
    fn elliptic_orbit_speed_at_apoapsis_matches_vis_viva() -> TestResult {
        let frame = frame();
        let e = Eccentricity::new(0.3);
        let orbit = elliptic_orbit(e, Angle::new(std::f64::consts::PI))?;
        let a = orbit.semi_major_axis(&frame)?.meters();
        let mu = MU_EARTH;
        let r_a = a * (1.0 + e.scalar());
        let expected = (mu * (2.0 / r_a - 1.0 / a)).sqrt();

        assert_close(orbit.speed(&frame)?.meters_per_second(), expected, EPS);
        Ok(())
    }

    #[test]
    fn elliptic_orbit_period_matches_kepler_third_law() -> TestResult {
        let frame = frame();
        let e = Eccentricity::new(0.4);
        let orbit = elliptic_orbit(e, Angle::new(1.0))?;
        let expected = 2.0
            * std::f64::consts::PI
            * (orbit.semi_major_axis(&frame)?.meters().powi(3) / MU_EARTH).sqrt();

        assert_close(orbit.period(&frame)?.seconds(), expected, EPS);
        Ok(())
    }

    #[test]
    fn zero_eccentricity_orbit_produces_circular_cartesian_state() -> TestResult {
        let radius_m = R_EARTH + 400_000.0;
        let true_anomaly = Angle::new(0.8);
        let orbit = Orbit::elliptic(
            Distance::new(radius_m),
            Eccentricity::new(0.0),
            Angle::ZERO,
            Angle::ZERO,
            Angle::ZERO,
            true_anomaly,
        );

        let state = orbit.to_cartesian_state(&frame())?;
        let speed = (MU_EARTH / radius_m).sqrt();

        assert_close(state.position.vector().norm(), radius_m, EPS);
        assert_close(state.velocity.vector().norm(), speed, EPS);
        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_periapsis_velocity_is_perpendicular_to_position() -> TestResult {
        let e = Eccentricity::new(0.5);
        let orbit = elliptic_orbit(e, Angle::new(0.0))?;
        let state = orbit.to_cartesian_state(&frame())?;

        assert_close(
            state.position.vector().dot(&state.velocity.vector()),
            0.0,
            1.0e-5,
        );
        Ok(())
    }

    #[test]
    fn identical_elliptic_orbits_produce_identical_velocity() -> TestResult {
        let e = Eccentricity::new(0.3);
        let first = elliptic_orbit(e, Angle::new(0.7))?.to_cartesian_state(&frame())?;
        let second = elliptic_orbit(e, Angle::new(0.7))?.to_cartesian_state(&frame())?;

        assert_close(second.velocity.vector().x, first.velocity.vector().x, EPS);
        assert_close(second.velocity.vector().y, first.velocity.vector().y, EPS);
        assert_close(second.velocity.vector().z, first.velocity.vector().z, EPS);
        Ok(())
    }

    #[test]
    fn elliptic_orbit_rejects_eccentricity_at_or_above_one() -> TestResult {
        let e = Eccentricity::new(1.0);

        assert!(
            elliptic_orbit(e, Angle::new(0.0))
                .is_ok_and(|o| o.to_cartesian_state(&frame()).is_err())
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_rejects_parabolic_eccentricity() -> TestResult {
        let e = Eccentricity::new(2.0);

        assert!(
            elliptic_orbit(e, Angle::new(0.0))
                .is_ok_and(|o| o.to_cartesian_state(&frame()).is_err())
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_nonzero_inclination_raan_and_argp() -> TestResult {
        let frame = frame();
        let e = Eccentricity::new(0.2);
        let orbit = Orbit::elliptic(
            Distance::new(R_EARTH + 1_000_000.0),
            e,
            Angle::new(0.6),
            Angle::new(0.9),
            Angle::new(1.2),
            Angle::new(0.4),
        );
        let state = orbit.to_cartesian_state(&frame)?;

        assert_close(
            state.position.vector().norm(),
            orbit.semi_major_axis(&frame)?.meters() * (1.0 - e.scalar() * e.scalar())
                / (1.0 + e.scalar() * orbit.true_anomaly().radians().cos()),
            EPS,
        );
        assert!(
            state.position.vector().z.abs() > 0.0,
            "expected nonzero z for inclined orbit"
        );
        Ok(())
    }

    #[test]
    fn elliptic_orbit_to_cartesian_state_has_correct_radius() -> TestResult {
        let frame = frame();
        let e = Eccentricity::new(0.2);
        let elliptic = elliptic_orbit(e, Angle::new(0.7))?;

        let state = elliptic.to_cartesian_state(&frame)?;

        let radius = elliptic.semi_major_axis(&frame)?.meters() * (1.0 - e.scalar() * e.scalar())
            / (1.0 + e.scalar() * elliptic.true_anomaly().radians().cos());
        assert_close(state.position.vector().norm(), radius, EPS);
        Ok(())
    }
}
