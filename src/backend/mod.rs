

#[cfg(feature = "rapier")]
mod rapier;
#[cfg(feature = "rapier")]
pub use rapier::{
    setup_physics_context,
    apply_forces,
    apply_ground_forces,
    get_velocity_from_rapier as get_velocity_from_backend,
    get_mass_from_rapier as get_mass_from_backend,
    RapierPhysicsBundle as BackendPhysicsBundle,
};


#[cfg(feature = "xpbd")]
mod rapier;
#[cfg(feature = "xpbd")]
pub use xpbd::{
    setup_physics_context,
    apply_forces,
    apply_ground_forces,
    get_velocity_from_xpbd as get_velocity_from_backend,
    get_mass_from_xpbd as get_mass_from_backend,
    XpbdPhysicsBundle as BackendPhysicsBundle,
};