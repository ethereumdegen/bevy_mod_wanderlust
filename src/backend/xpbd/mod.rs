
use bevy::prelude::*;

#[cfg(feature = "xpbd_3d")]
pub use bevy_xpbd_3d as xpbd;
#[cfg(feature = "xpbd_2d")]
pub use bevy_xpbd_2d as xpbd;

//use xpbd::prelude::*; 


pub mod mass;
pub use mass::Mass;
//pub use mass::*;

pub mod velocity;
pub use velocity::Velocity;

//pub use xpbd::prelude::{Collider,SpatialQueryFilter};

use bevy_xpbd_3d::prelude::*;
pub use bevy_xpbd_3d::prelude::Collider;


mod context;
pub use context::*;

mod ground; 
pub use ground::*;


/// Contains common physics settings for character controllers.
#[derive(Bundle)]
pub struct XpbdPhysicsBundle {
    /// See [`RigidBody`].
    pub rigidbody: RigidBody,
    /// See [`Collider`].
    pub collider: Collider,
    /// See [`GravityScale`].
    pub gravity: GravityScale,
    /// See [`Friction`].
    pub friction: Friction,
    /// See [`Restitution`].
    pub restitution: Restitution,
}

impl Default for XpbdPhysicsBundle {
    fn default() -> Self {
        Self {
            rigidbody: default(),
            collider: Collider::capsule_endpoints(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(0.0, 0.5, 0.0),
                0.5,
            ),
            gravity: GravityScale(0.0),
            friction: Friction::new(0.0).with_combine_rule(CoefficientCombine::Min),
            restitution: Restitution::new(0.0).with_combine_rule(CoefficientCombine::Min),
        }
    }
}

pub fn apply_forces() {}
pub fn apply_ground_forces() {}
pub fn setup_physics_context() {}

pub type SpatialQuery<'w, 's> = bevy_xpbd_3d::prelude::SpatialQuery<'w, 's>;

use crate::backend::query::{RayCastResult, QueryFilter };

 

/*
pub fn cast_ray(
    spatial_query: &SpatialQuery,
    origin: Vec3,
    direction: Vec3,
    max_toi: f32,
    solid: bool,
    filter: QueryFilter,
) -> Option<RayCastResult> {
    spatial_query.cast_ray(
        origin,
        direction,
        max_toi,
        solid,
        SpatialQueryFilter {
            excluded_entities: filter.exclude,
            ..default()
        },
    ).map(|result| {
        let point = origin + direction * result.time_of_impact;
        RayCastResult {
            entity: result.entity,
            normal: result.normal,
            point: point,
            toi: result.time_of_impact,
        }
    })
}

*/

 
/*

pub fn cast_shape (
    spatial_query: &SpatialQuery,
    mut shape_pos: Vec3,
    shape_rot: Quat,
    shape_vel: Vec3,
    shape: &Collider,
    max_toi: f32,
    filter: QueryFilter,
) -> Option<RayCastResult> { 
    
    //fix me ! 
    None 
}*/
/*
    /// Cast a shape downwards using the parameters.
    pub fn cast_shape(
        &self,
        ctx: &RapierContext,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        let Some((entity, toi)) = ctx
            .cast_shape(self.position, self.rotation, self.direction, self.shape, self.max_toi, self.filter) else { return None };

        if toi.status == TOIStatus::Penetrating || toi.toi <= std::f32::EPSILON {
            return None;
        }

        let (entity, cast) = (entity, CastResult::from_toi1(toi));

        gizmos.ray(self.position, self.direction * cast.toi, Color::BLUE);
        gizmos.sphere(
            self.position + self.direction * cast.toi,
            self.rotation,
            0.3,
            Color::BLUE,
        );

        Some((entity, cast))
    }

    /// A fallback to a simple raycasting downwards.
    ///
    /// Used in the case that we are unable to correct penetration.
    pub fn cast_ray(&self, ctx: &RapierContext) -> Option<(Entity, CastResult)> {
        // This should only occur if the controller fails to correct penetration
        // of colliders.

        // local shape offset from origin to bottom of shape
        let offset = self
            .shape
            .cast_local_ray(Vec3::ZERO, self.direction, 10.0, false)
            .unwrap_or(0.);
        let ray_pos = self.position + self.direction * offset;

        ctx.cast_ray_and_get_normal(ray_pos, self.direction, self.max_toi, true, self.filter)
            .map(|(entity, inter)| (entity, inter.into()))
    }
    
    */