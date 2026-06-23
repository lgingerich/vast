use crate::errors::StateError;
use crate::{EquatorialRadius, GravitationalParameter, J2Coefficient, RotationRate};

/// A celestial body modeled by the simulator.
///
/// Acts as a key into the body's physical constants (gravitational parameter,
/// equatorial radius, rotation rate, and J2 coefficient) so that frames,
/// dynamics, and propagation paths can resolve body-specific values uniformly.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum BodyId {
    /// Earth.
    Earth,
    /// The Moon.
    Moon,
}

impl BodyId {
    /// Returns the body's gravitational parameter μ = G·M.
    pub const fn mu(&self) -> GravitationalParameter {
        match self {
            Self::Earth => GravitationalParameter::earth(),
            Self::Moon => GravitationalParameter::moon(),
        }
    }

    /// Returns the body's equatorial radius.
    pub const fn equatorial_radius(&self) -> EquatorialRadius {
        match self {
            Self::Earth => EquatorialRadius::earth(),
            Self::Moon => EquatorialRadius::moon(),
        }
    }

    /// Returns the body's inertial rotation rate about its spin axis.
    pub const fn rotation_rate(&self) -> RotationRate {
        match self {
            Self::Earth => RotationRate::earth(),
            Self::Moon => RotationRate::moon(),
        }
    }

    /// Returns the body's J2 oblateness coefficient, if modeled.
    ///
    /// `None` indicates the body's oblateness is not modeled for this simulation.
    pub const fn j2(&self) -> Option<J2Coefficient> {
        match self {
            Self::Earth => Some(J2Coefficient::earth()),
            Self::Moon => Some(J2Coefficient::moon()),
        }
    }
}

/// Reference point of a [`Frame`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Origin {
    /// Origin at the center of mass of a celestial body.
    CelestialBody(BodyId),
    /// Origin at the combined center of mass of a system (e.g. the Earth-Moon barycenter).
    Barycenter(&'static [BodyId]),
}

/// Orientation of a [`Frame`]'s coordinate axes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Axes {
    /// Earth Mean Equator and Equinox of J2000 (EME2000) inertial axes.
    J2000Eq,
    /// Axes corotating with the central body: x toward the prime meridian, z along the rotation axis.
    BodyFixed(BodyId),
}

/// A reference frame defined by an [`Origin`] and an [`Axes`] orientation.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct Frame {
    origin: Origin,
    axes: Axes,
}

impl Frame {
    /// Creates a new frame from an origin and an axis orientation.
    pub const fn new(origin: Origin, axes: Axes) -> Self {
        Self { origin, axes }
    }

    // ---------------- Accessors ----------------

    /// Returns the frame's origin.
    pub const fn origin(&self) -> Origin {
        self.origin
    }

    /// Returns the frame's axis orientation.
    pub const fn axes(&self) -> Axes {
        self.axes
    }

    /// Returns the central body of a [`Origin::CelestialBody`] origin, or `None`
    /// for a [`Origin::Barycenter`].
    pub const fn central_body(&self) -> Option<BodyId> {
        match &self.origin {
            Origin::CelestialBody(body) => Some(*body),
            Origin::Barycenter(_) => None,
        }
    }

    /// Returns the body whose rotation defines [`Axes::BodyFixed`] axes, or
    /// `None` for inertial [`Axes::J2000Eq`] axes.
    pub const fn axes_body(&self) -> Option<BodyId> {
        match &self.axes {
            Axes::BodyFixed(body) => Some(*body),
            Axes::J2000Eq => None,
        }
    }

    /// Returns the bodies whose mass defines a [`Origin::Barycenter`] origin,
    /// or `None` for a [`Origin::CelestialBody`] origin.
    pub const fn barycenter_bodies(&self) -> Option<&'static [BodyId]> {
        match &self.origin {
            Origin::Barycenter(bodies) => Some(*bodies),
            Origin::CelestialBody(_) => None,
        }
    }

    // ---------------- Body-Constant Resolution ----------------

    /// Returns the central body's gravitational parameter.
    pub fn mu(&self) -> Result<GravitationalParameter, StateError> {
        Ok(self.central_body().ok_or(StateError::InvalidFrame)?.mu())
    }

    /// Returns the central body's equatorial radius.
    pub fn equatorial_radius(&self) -> Result<EquatorialRadius, StateError> {
        Ok(self
            .central_body()
            .ok_or(StateError::InvalidFrame)?
            .equatorial_radius())
    }

    /// Returns the central body's inertial rotation rate.
    pub fn rotation_rate(&self) -> Result<RotationRate, StateError> {
        Ok(self
            .central_body()
            .ok_or(StateError::InvalidFrame)?
            .rotation_rate())
    }

    /// Returns the central body's J2 coefficient, if modeled.
    pub fn j2(&self) -> Result<Option<J2Coefficient>, StateError> {
        Ok(self.central_body().ok_or(StateError::InvalidFrame)?.j2())
    }

    // ---------------- Validation ----------------

    /// Returns `Ok` when this frame has inertial axes.
    pub fn require_inertial_axes(&self) -> Result<(), StateError> {
        match self.axes {
            Axes::J2000Eq => Ok(()),
            Axes::BodyFixed(_) => Err(StateError::InvalidAxes),
        }
    }

    // ---------------- Named Frames ----------------

    /// The Earth-centered inertial frame: origin at Earth's center of mass with
    /// J2000 mean equator and equinox axes.
    pub const fn earth_inertial() -> Self {
        Self::new(Origin::CelestialBody(BodyId::Earth), Axes::J2000Eq)
    }

    /// The Moon-centered inertial frame: origin at the Moon's center of mass
    /// with J2000 mean equator and equinox axes.
    pub const fn moon_inertial() -> Self {
        Self::new(Origin::CelestialBody(BodyId::Moon), Axes::J2000Eq)
    }

    /// The Earth-centered, Earth-fixed frame: origin at Earth's center of mass
    /// with axes corotating with Earth.
    pub const fn earth_body_fixed() -> Self {
        Self::new(
            Origin::CelestialBody(BodyId::Earth),
            Axes::BodyFixed(BodyId::Earth),
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn body_id_dispatch_matches_named_constants() {
        assert_eq!(BodyId::Earth.mu(), GravitationalParameter::earth());
        assert_eq!(BodyId::Moon.mu(), GravitationalParameter::moon());

        assert_eq!(BodyId::Earth.equatorial_radius(), EquatorialRadius::earth());
        assert_eq!(BodyId::Moon.equatorial_radius(), EquatorialRadius::moon());

        assert_eq!(BodyId::Earth.rotation_rate(), RotationRate::earth());
        assert_eq!(BodyId::Moon.rotation_rate(), RotationRate::moon());

        assert_eq!(BodyId::Earth.j2(), Some(J2Coefficient::earth()));
        assert_eq!(BodyId::Moon.j2(), Some(J2Coefficient::moon()));
    }

    #[test]
    fn frame_resolves_central_body_constants() {
        let frame = Frame::earth_inertial();

        assert_eq!(frame.mu(), Ok(GravitationalParameter::earth()));
        assert_eq!(frame.equatorial_radius(), Ok(EquatorialRadius::earth()));
        assert_eq!(frame.rotation_rate(), Ok(RotationRate::earth()));
        assert_eq!(frame.j2(), Ok(Some(J2Coefficient::earth())));
    }

    #[test]
    fn barycenter_frame_cannot_resolve_single_body_constants() {
        let frame = Frame::new(
            Origin::Barycenter(&[BodyId::Earth, BodyId::Moon]),
            Axes::J2000Eq,
        );

        assert_eq!(frame.mu(), Err(StateError::InvalidFrame));
        assert_eq!(frame.equatorial_radius(), Err(StateError::InvalidFrame));
        assert_eq!(frame.rotation_rate(), Err(StateError::InvalidFrame));
        assert_eq!(frame.j2(), Err(StateError::InvalidFrame));
    }

    #[test]
    fn inertial_axes_requirement_rejects_body_fixed_axes() {
        assert_eq!(Frame::earth_inertial().require_inertial_axes(), Ok(()));
        assert_eq!(
            Frame::earth_body_fixed().require_inertial_axes(),
            Err(StateError::InvalidAxes),
        );
    }

    #[test]
    fn earth_inertial_frame_is_central_body_with_j2000_axes() {
        let frame = Frame::earth_inertial();

        assert_eq!(frame.origin(), Origin::CelestialBody(BodyId::Earth));
        assert_eq!(frame.axes(), Axes::J2000Eq);
        assert_eq!(frame.central_body(), Some(BodyId::Earth));
        assert_eq!(frame.axes_body(), None);
        assert_eq!(frame.barycenter_bodies(), None);
    }

    #[test]
    fn moon_inertial_frame_is_central_body_with_j2000_axes() {
        let frame = Frame::moon_inertial();

        assert_eq!(frame.origin(), Origin::CelestialBody(BodyId::Moon));
        assert_eq!(frame.axes(), Axes::J2000Eq);
        assert_eq!(frame.central_body(), Some(BodyId::Moon));
        assert_eq!(frame.axes_body(), None);
        assert_eq!(frame.barycenter_bodies(), None);
    }

    #[test]
    fn earth_body_fixed_frame_matches_body_for_origin_and_axes() {
        let frame = Frame::earth_body_fixed();

        assert_eq!(frame.origin(), Origin::CelestialBody(BodyId::Earth));
        assert_eq!(frame.axes(), Axes::BodyFixed(BodyId::Earth));
        assert_eq!(frame.central_body(), Some(BodyId::Earth));
        assert_eq!(frame.axes_body(), Some(BodyId::Earth));
        assert_eq!(frame.barycenter_bodies(), None);
    }

    #[test]
    fn custom_frame_allows_mismatched_body_origin_and_axes() {
        // A body-fixed Moon frame centered on Earth is physically odd but
        // structurally valid, so the API must not conflate origin and axes.
        let frame = Frame::new(
            Origin::CelestialBody(BodyId::Earth),
            Axes::BodyFixed(BodyId::Moon),
        );

        assert_eq!(frame.central_body(), Some(BodyId::Earth));
        assert_eq!(frame.axes_body(), Some(BodyId::Moon));
    }

    #[test]
    fn barycenter_frame_reports_bodies_and_no_central_body() {
        let bodies: &'static [BodyId] = &[BodyId::Earth, BodyId::Moon];
        let frame = Frame::new(Origin::Barycenter(bodies), Axes::J2000Eq);

        assert_eq!(frame.central_body(), None);
        assert_eq!(frame.barycenter_bodies(), Some(bodies));
        assert_eq!(frame.barycenter_bodies().unwrap(), bodies);
    }

    #[test]
    fn barycenter_with_body_fixed_axes_reports_axes_body() {
        let frame = Frame::new(
            Origin::Barycenter(&[BodyId::Earth, BodyId::Moon]),
            Axes::BodyFixed(BodyId::Earth),
        );

        assert_eq!(frame.central_body(), None);
        assert_eq!(frame.axes_body(), Some(BodyId::Earth));
    }

    #[test]
    fn empty_barycenter_origin_is_allowed() {
        let frame = Frame::new(Origin::Barycenter(&[]), Axes::J2000Eq);

        assert_eq!(frame.barycenter_bodies(), Some(&[] as &[BodyId]));
    }

    #[test]
    fn frame_derives_copy_clone_equality_and_hash() {
        let a = Frame::earth_inertial();
        let b = a;
        let c = Frame::earth_inertial();

        assert_eq!(a, b);
        assert_eq!(a, c);

        // Structurally different frames must not compare equal.
        assert_ne!(a, Frame::moon_inertial());
        assert_ne!(a, Frame::earth_body_fixed());

        use std::collections::hash_map::DefaultHasher;
        use std::hash::{Hash, Hasher};

        fn hash_of(frame: &Frame) -> u64 {
            let mut hasher = DefaultHasher::new();
            frame.hash(&mut hasher);
            hasher.finish()
        }

        assert_eq!(hash_of(&a), hash_of(&b));
    }

    #[test]
    fn frame_debug_contains_discriminant_names() {
        let out = format!("{:?}", Frame::earth_body_fixed());
        assert!(out.contains("CelestialBody"));
        assert!(out.contains("BodyFixed"));
        assert!(out.contains("Earth"));
    }
}
