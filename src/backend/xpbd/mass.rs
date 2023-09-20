/*
use bevy::{
    ecs::query::WorldQuery,
     prelude::*,
};
 use crate::backend::xpbd;
 
*/
 use bevy_xpbd_3d::prelude::{Mass as xpbd_Mass,Inertia,CenterOfMass};
 
 //use crate::backend::xpbd::Mass as xpbd_Mass;
use bevy::{
    ecs::query::WorldQuery,
     prelude::*,
};

#[derive(WorldQuery)]
pub struct Mass {
    mass: &'static  xpbd_Mass,
    inertia: &'static Inertia,
    center_of_mass: &'static CenterOfMass,
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