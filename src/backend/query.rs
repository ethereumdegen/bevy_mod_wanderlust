

use bevy::{prelude::*, utils::HashSet};

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

#[derive(Debug,   Clone, Reflect)]
pub struct QueryFilter {
    pub exclude: HashSet<Entity>,
}
 
 
impl QueryFilter {
    
    pub fn new() -> Self { 
        Self {
            exclude: HashSet::new()            
        } 
    }
    
    pub fn exclude_sensors(self) -> Self{
        self //fix me ? 
    }
    
}
 