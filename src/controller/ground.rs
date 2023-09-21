use crate::controller::*;
use bevy::utils::HashSet;

//maybe take this from parry direct ? 
use bevy_xpbd_3d::parry::query::RayIntersection;

use crate::backend::Collider;

  use crate::backend::query::{QueryFilter, RayCastResult};


/// How to detect if something below the controller is suitable
/// for standing on.
#[derive(Component, Reflect)]
#[reflect(Component, Default)]
pub struct GroundCaster {
    /// A timer to track how long to skip the ground check for (see [`jump_skip_ground_check_duration`](ControllerSettings::jump_skip_ground_check_duration)).
    pub skip_ground_check_timer: f32,
    /// Override skip ground check. If true, never checks for the ground.
    pub skip_ground_check_override: bool,
    /// An offset to start the ground check from, relative to the character's origin.
    pub cast_origin: Vec3,
    /// How long of a ray to cast to detect the ground. Setting this unnecessarily high will permanently count the player as grounded,
    /// and too low will allow the player to slip and become disconnected from the ground easily.
    pub cast_length: f32,
    /// What shape of ray to cast. See [`Collider`] and [`RapierContext::cast_shape`](bevy_rapier::prelude::RapierContext).
    #[reflect(ignore)]
    pub cast_collider: Option<Collider>,
    /// Set of entities that should be ignored when ground casting.
    pub exclude_from_ground: HashSet<Entity>,

    /// Threshold, in radians, of when a controller will start to slip on a surface.
    ///
    /// The controller will still be able to jump and overall be considered grounded.
    pub unstable_ground_angle: f32,
    /// The maximum angle that the ground can be, in radians, before it is no longer considered suitable for being "grounded" on.
    ///
    /// For example, if this is set to `π/4` (45 degrees), then a controller standing on a slope steeper than 45 degrees will slip and fall, and will not have
    /// their jump refreshed by landing on that surface.
    pub max_ground_angle: f32,
}

impl Default for GroundCaster {
    fn default() -> Self {
        Self {
            skip_ground_check_timer: 0.0,
            skip_ground_check_override: false,
            cast_origin: Vec3::ZERO,
            cast_length: 1.0,
            cast_collider: Some( Collider::ball(0.45)) ,
            exclude_from_ground: default(),
            unstable_ground_angle: 45.0 * (std::f32::consts::PI / 180.0),
            max_ground_angle: 60.0 * (std::f32::consts::PI / 180.0),
        }
    }
}


#[derive(Copy, Clone)]
pub struct Ground {
    /// Entity found in ground cast.
    pub entity: Entity,
    /// Specifics of the ground contact.
    pub cast: CastResult,
    /// Is this ground stable for the collider.
    pub stable: bool,
    /// Is this ground viable for the collider.
    pub viable: bool,
    /// Angular velocity of the ground body.
    pub angular_velocity: Vec3,
    /// Linear velocity of the ground body.
    pub linear_velocity: Vec3,
    /// Linear velocity at the point of contact.
    pub point_velocity: Vec3,
}



/// The cached ground cast. Contains the entity hit, the hit info, and velocity of the entity
/// hit.
#[derive(Component, Default)]
pub enum GroundCast {
    Touching(Ground),
    Last(Ground),
    #[default]
    None,
}

impl GroundCast {
    pub fn last(&self) -> Option<&Ground> {
        match self {
            Self::Touching(ground) | Self::Last(ground) => Some(ground),
            Self::None => None,
        }
    }

    pub fn grounded(&self) -> bool {
        match self {
            Self::Touching(_) => true,
            Self::Last(_) | Self::None => false,
        }
    }

    pub fn into_last(&mut self) {
        match self {
            GroundCast::Touching(ground) => {
                *self = GroundCast::Last(ground.clone());
            }
            _ => {}
        }
    }
}



 
/// The cached viable ground cast. Contains the entity hit, the hit info, and velocity of the entity
/// hit.
#[derive(Component, Default, Deref, DerefMut)]
pub struct ViableGroundCast(
    /// Ground that was found this frame
    pub GroundCache,
);

/// Current/last ground.
#[derive(Default)]
pub enum GroundCache {
    /// This will stay the ground until we leave the ground entirely.
    Ground(Ground),
    /// Cached ground.
    Last(Ground),
    /// No stable ground.
    #[default]
    None,
}

 
impl GroundCache {
    /// Update the ground depending on the current ground cast.
    pub fn update(&mut self, ground: Option<Ground>) {
        match ground {
            Some(ground) => {
                *self = Self::Ground(ground);
            }
            None => {
                self.into_last();
            }
        }
    }

    /// Archive this ground cast.
    pub fn into_last(&mut self) {
        match self {
            Self::Ground(ground) => {
                *self = Self::Last(ground.clone());
            }
            _ => {}
        }
    }

    /// Ground we are currently touching
    pub fn current(&self) -> Option<&Ground> {
        match self {
            Self::Ground(ground) => Some(ground),
            _ => None,
        }
    }

    /// Last ground we touched, this includes the ground we are currently touching.
    pub fn last(&self) -> Option<&Ground> {
        match self {
            Self::Ground(ground) | Self::Last(ground) => Some(ground),
            Self::None => None,
        }
    }
}
    

/// Is the character grounded?
#[derive(Component, Default, Reflect, Deref)]
#[reflect(Component, Default)]
pub struct Grounded(pub bool);

/// Force applied to the ground the controller is on.
#[derive(Copy, Clone, Component, Default, Reflect)]
#[reflect(Component, Default)]
pub struct GroundForce {
    /// Change in linear velocity.
    pub linear: Vec3,
    /// Change in angular velocity.
    pub angular: Vec3,
} 

pub fn determine_groundedness(mut query: Query<(&Float, &GroundCast, &mut Grounded)>) {
    for (float, cast, mut grounded) in &mut query {
        let float_offset = if let GroundCast::Touching(ground) = cast {
            Some(ground.cast.toi - float.distance)
        } else {
            None
        };

        grounded.0 = float_offset
            .map(|offset| offset <= float.max_offset && offset >= float.min_offset)
            .unwrap_or(false);
    }
}






/// Details about a shape/ray-cast.
#[derive(Default, Debug, Copy, Clone, Reflect)]
pub struct CastResult {
    /// Time-of-impact to the other shape.
    pub toi: f32,
    /// Normal of the other shape.
    pub normal: Vec3,
    /// Witness point for the shape/ray cast.
    pub point: Vec3,
}

impl CastResult {
    /// Get the tangential normal biased downwards.
    pub fn down_tangent(&self, up_vector: Vec3) -> Vec3 {
        let (x, z) = self.normal.any_orthonormal_pair();
        let projected_x = up_vector.project_onto(x);
        let projected_z = up_vector.project_onto(z);
        -(projected_x + projected_z)
    }

    /// Cast has a viable normal based on a max angle.
    pub fn viable(&self, up_vector: Vec3, max_angle: f32) -> bool {
        self.normal.angle_between(up_vector).abs() < max_angle
    }
}
 
 
 /* 
impl CastResult {
    /// Use the first shape in the shape-cast as the cast result.
  pub fn from_toi1(toi: Toi) -> Self {
        Self {
            toi: toi.toi,
            normal: toi.normal1,
            point: toi.witness1,
        }
    }

    /// Use the second shape in the shape-cast as the cast result.
    pub fn from_toi2(toi: Toi) -> Self {
        Self {
            toi: toi.toi,
            normal: toi.normal2,
            point: toi.witness2,
        }
    }
}*/

/*
impl From<RayIntersection> for CastResult {
    fn from(intersection: RayIntersection) -> Self {
        Self {
            toi: intersection.toi,
            normal: intersection.normal.into(),
            
            point: intersection.point ,
            
           
        }
    }
}*/



/*

spatial_query: &SpatialQuery,
    origin: Vec3,
    direction: Vec3,
    max_toi: f32,
     solid: bool,
    filter: QueryFilter,
    

*/

/*
fn ground_cast(
    spatial_query: &SpatialQuery,
    colliders: &Query<&Collider>,
    globals: &Query<&GlobalTransform>,
    mut shape_pos: Vec3,
    shape_rot: Quat,
    shape_vel: Vec3,
    shape: &Collider,
    max_toi: f32,
    filter: QueryFilter,
) -> Option<(Entity, RayCastResult)> {
    for _ in 0..12 {
        if let Some((entity, rayhit)) =
            crate::backend::cast_shape(
                spatial_query, 
                shape_pos,
                 shape_rot,
                  shape_vel, 
                  shape,
                   max_toi,
                   filter
                   )
        {
            /*
            if toi.status != TOIStatus::Penetrating {
                return Some((entity, toi.into()));
            } */

            /*
            match (globals.get(entity), colliders.get(entity)) {
                (Ok(ground_global), Ok(ground_collider)) => {
                    let cast_iso = Isometry3 {
                        translation: shape_pos.into(),
                        rotation: shape_rot.into(),
                    };

                    let (_, ground_rotation, ground_translation) =
                        ground_global.to_scale_rotation_translation();
                    let ground_iso = Isometry3 {
                        translation: ground_translation.into(),
                        rotation: ground_rotation.into(),
                    };

                    if let Ok(Some(contact)) = bevy_rapier3d::parry::query::contact(
                        &cast_iso,
                        &*shape.raw,
                        &ground_iso,
                        &*ground_collider.raw,
                        0.0,
                    ) {
                        let normal: Vec3 = contact.normal2.into();
                        // This prevents some issues where we get a near 0.0 time-of-impact due to floating point imprecision.
                        const EXTRA_CORRECTION: f32 = 1.5;
                        let correction = normal * (-contact.dist).max(0.05) * EXTRA_CORRECTION;
                        shape_pos += correction;
                    }
                }
                _ => {}
            };
            */
            return None;
        } else {
            return None;
        }
    }

    // We need to offset it so the point of contact is identical to the shape cast.
    let offset = shape
        .cast_local_ray(Vec3::ZERO, shape_vel, 10.0, false)
        .unwrap_or(0.);
    shape_pos = shape_pos + shape_vel * offset;

    /*ctx.cast_ray_and_get_normal(shape_pos, shape_vel, max_toi, true, filter)
    .map(|(entity, inter)| (entity, inter.into()))*/
    None
}
*/
/*
fn intersections_with_ray_cast(
    ctx: &RapierContext,
    max_toi: f32,
    filter: QueryFilter,
) {
    let offset = shape
        .cast_local_ray(Vec3::ZERO, shape_vel, 10.0, false)
        .unwrap_or(0.);
    let shape_pos = shape_pos + shape_vel * offset;

    loop {
        if let Some((entity, inter)) =
            ctx.cast_ray_and_get_normal(shape_pos, shape_vel, max_toi, true, filter)
        {
            collisions.push((entity, inter));
        } else {
            break;
        }
    }
}
*/

/*
/// Details about a shape/ray-cast.
#[derive(Default, Debug, Copy, Clone, Reflect)]
pub struct CastResult {
    /// Time-of-impact to the other shape.
    pub toi: f32,
    /// Normal of the other shape.
    pub normal: Vec3,
    /// Witness point for the shape/ray cast.
    pub point: Vec3,
}


impl CastResult {
    /// Get the tangential normal biased downwards.
    pub fn down_tangent(&self, up_vector: Vec3) -> Vec3 {
        let (x, z) = self.normal.any_orthonormal_pair();
        let projected_x = up_vector.project_onto(x);
        let projected_z = up_vector.project_onto(z);
        -(projected_x + projected_z)
    }

    /// Cast has a viable normal based on a max angle.
    pub fn viable(&self, up_vector: Vec3, max_angle: f32) -> bool {
        self.normal.angle_between(up_vector).abs() < max_angle
    }
}

impl CastResult {
    /// Use the first shape in the shape-cast as the cast result.
    pub fn from_toi1(toi: Toi) -> Self {
        Self {
            toi: toi.toi,
            normal: toi.normal1,
            point: toi.witness1,
        }
    }

    /// Use the second shape in the shape-cast as the cast result.
    pub fn from_toi2(toi: Toi) -> Self {
        Self {
            toi: toi.toi,
            normal: toi.normal2,
            point: toi.witness2,
        }
    }
}

impl From<RayIntersection> for CastResult {
    fn from(intersection: RayIntersection) -> Self {
        Self {
            toi: intersection.toi,
            normal: intersection.normal,
            point: intersection.point,
        }
    }
}

*/