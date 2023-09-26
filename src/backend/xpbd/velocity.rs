
use bevy::{
    prelude::*,
    ecs::query::WorldQuery,
};
use crate::backend::xpbd;


#[derive(WorldQuery)]
pub struct Velocity {
    linear: &'static xpbd::LinearVelocity,
    angular: &'static xpbd::AngularVelocity,
}

impl<'a> VelocityItem<'a> {
    pub fn linear(&self) -> Vec3 {
        **self.linear
    }

    pub fn angular(&self) -> Vec3 {
        **self.angular
    }
}