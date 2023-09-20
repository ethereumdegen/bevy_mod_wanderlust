NOTES

check on orientation inertia value matrices 





This is inside of rapier geometry 



/// Structure containing the result of a successful ray cast.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct RayIntersection {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with `origin + dir * toi` where `origin` is the origin of the ray;
    /// `dir` is its direction and `toi` is the value of this field.
    pub toi: Real,

    /// The intersection point between the ray and the object.
    pub point: Vect,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    pub normal: Vect,

    /// Feature at the intersection point.
    pub feature: FeatureId,
}

impl RayIntersection {
    pub(crate) fn from_rapier(
        inter: rapier::parry::query::RayIntersection,
        unscaled_origin: Vect,
        unscaled_dir: Vect,
    ) -> Self {
        Self {
            toi: inter.toi,
            point: unscaled_origin + unscaled_dir * inter.toi,
            normal: inter.normal.into(),
            feature: inter.feature,
        }
    }
}

/// The result of a time-of-impact (TOI) computation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Toi {
    /// The time at which the objects touch.
    pub toi: Real,
    /// The local-space closest point on the first shape at the time of impact.
    ///
    /// Undefined if `status` is `Penetrating`.
    pub witness1: Vect,
    /// The local-space closest point on the second shape at the time of impact.
    ///
    /// Undefined if `status` is `Penetrating`.
    pub witness2: Vect,
    /// The local-space outward normal on the first shape at the time of impact.
    ///
    /// Undefined if `status` is `Penetrating`.
    pub normal1: Vect,
    /// The local-space outward normal on the second shape at the time of impact.
    ///
    /// Undefined if `status` is `Penetrating`.
    pub normal2: Vect,
    /// The way the time-of-impact computation algorithm terminated.
    pub status: TOIStatus,
}

impl Toi {
    pub(crate) fn from_rapier(physics_scale: Real, toi: rapier::parry::query::TOI) -> Self {
        Self {
            toi: toi.toi,
            witness1: (toi.witness1 * physics_scale).into(),
            witness2: (toi.witness2 * physics_scale).into(),
            normal1: toi.normal1.into(),
            normal2: toi.normal2.into(),
            status: toi.status,
        }
    }
}


