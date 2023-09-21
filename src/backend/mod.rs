use bevy::{
    utils::HashSet,
   // prelude::*
};

pub mod query;

#[cfg(feature = "rapier")]
mod rapier;
#[cfg(feature = "rapier")]
pub use rapier::{
    //apply_forces,
    //apply_ground_forces,
    //cast_ray,
    //cast_shape,
    //setup_physics_context,
    
    find_ground,
    determine_groundedness,
    
    RapierPhysicsBundle as BackendPhysicsBundle,
    SpatialQuery,
    Velocity,
    Mass, 
    Collider
};

#[cfg(feature = "xpbd")]
mod xpbd; 

//#[cfg(feature = "xpbd")]
//pub use  crate::backend::xpbd::{XpbdPhysicsBundle,Velocity} ;

#[cfg(feature = "xpbd")]
pub use crate::backend::xpbd::{
    find_ground,
    determine_groundedness,
    apply_forces,
    apply_ground_forces,
    //cast_ray,
    //cast_shape,
    setup_physics_context,
    SpatialQuery,
    XpbdPhysicsBundle as BackendPhysicsBundle,
    Velocity,
    Mass,
    Collider
};

 
