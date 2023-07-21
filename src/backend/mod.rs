use bevy::{
    utils::HashSet,
    prelude::*
};

#[cfg(feature = "rapier")]
mod rapier;
#[cfg(feature = "rapier")]
pub use rapier::{
    //apply_forces,
    //apply_ground_forces,
    //cast_ray,
    //cast_shape,
    //setup_physics_context,
    RapierPhysicsBundle as BackendPhysicsBundle,
    SpatialQuery,
    Velocity,
    Mass,
};

#[cfg(feature = "xpbd")]
mod xpbd;
#[cfg(feature = "xpbd")]
pub use xpbd::{
    apply_forces,
    apply_ground_forces,
    cast_ray,
    //cast_shape,
    setup_physics_context,
    SpatialQuery,
    XpbdPhysicsBundle as BackendPhysicsBundle,
    Velocity,
    Mass,
};

#[derive(Debug, Copy, Clone, Reflect)]
pub struct RayCastResult {
    pub entity: Entity,
    pub toi: f32,
    pub normal: Vec3,
    pub point: Vec3,
}

#[derive(Debug, Copy, Clone, Reflect)]
pub struct ShapeCastResult {
    pub entity: Entity,
    pub toi: f32,
    pub normal1: Vec3,
    pub normal2: Vec3,
    pub point1: Vec3,
    pub point2: Vec3,
}

pub struct Filter {
    pub exclude: HashSet<Entity>,
}