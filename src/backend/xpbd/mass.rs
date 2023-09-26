/*
use bevy::{
    ecs::query::WorldQuery,
     prelude::*,
};
 use crate::backend::xpbd;
 
*/
 
 extern crate nalgebra as na;
 
 use bevy_xpbd_3d::{prelude::{Mass as xpbd_Mass,Inertia,CenterOfMass}, math::{Scalar }};
 
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
impl<'a> MassItem<'a>  {
    pub fn mass(&self) -> f32 {
        self.mass.0
    }
    
    //fix mee !! 
    pub fn inertia(&self) -> Vec3 {
        compute_principal_inertia( self.inertia.0 )
    }

    pub fn inertia_matrix(&self) -> Mat3 {
           self.inertia.0
    }

    pub fn local_center_of_mass(&self) -> Vec3 {
        self.center_of_mass.0
    }
}



//temporary soln.. maybe not efficient ?
fn compute_principal_inertia(inertia_tensor: Mat3) -> Vec3{
    let col0 = inertia_tensor.x_axis;
    let col1 = inertia_tensor.y_axis;
    let col2 = inertia_tensor.z_axis;

    let inertia_na = na::Matrix3::new(
        col0.x, col1.x, col2.x,
        col0.y, col1.y, col2.y,
        col0.z, col1.z, col2.z,
    );

    // Compute the eigenvalues of the inertia tensor
    let eigenvalues = inertia_na.symmetric_eigen().eigenvalues;

    eigenvalues.into()
}
