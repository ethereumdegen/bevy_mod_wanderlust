
use bevy::{
    ecs::query::WorldQuery,
     prelude::*,
};
 use crate::backend::xpbd;
 

#[derive(WorldQuery)]
pub struct Mass {
    mass: &'static xpbd::Mass,
    inertia: &'static xpbd::Inertia,
    center_of_mass: &'static xpbd::CenterOfMass,
}

impl<'a> MassItem<'a> {
    pub fn mass(&self) -> f32 {
        self.mass.0
    }

    pub fn inertia(&self) -> Mat3 {
        self.inertia.0
    }

    pub fn local_center_of_mass(&self) -> Vec3 {
        self.center_of_mass.0
    }
}