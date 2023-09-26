use bevy::prelude::*;
 

mod gravity;
mod ground;
mod input;
mod movement;
mod orientation;

use crate::physics::*;
use crate::backend::*;
use crate::spring::Spring;

pub use {gravity::*, ground::*, input::*, movement::*, orientation::*};

/// Components required for calculating controller forces.
#[derive(Bundle)]
pub struct Controller {
    /// How strong the controller should be pulled down if the ground
    /// isn't there.
    pub gravity: Gravity,
    /// Calculated gravity force.
    pub gravity_force: GravityForce,

    /// How to detect if something below the controller is suitable
    /// for standing on.
    pub ground_caster: GroundCaster,
    /// Ground entity found that is considered ground.
    pub ground_cast: GroundCast,
    /// Ground entity found that is considered viable ground.
    pub viable_ground_cast: ViableGroundCast,
    /// Is the controller currently considered on viable ground.
    pub grounded: Grounded,
    /// Force applied to the ground the controller is on.
    pub ground_force: GroundForce,

    /// Adjusting speed of the controller.
    pub movement: Movement,
    /// Calculated force for moving the controller.
    pub movement_force: MovementForce,

    /// How the controller's jumping should behave.
    pub jump: Jump,
    /// Calculated force for allowing the controller to jump.
    pub jump_force: JumpForce,

    /// How the far to float and how stiff that floating should be.
    pub float: Float,
    /// Calculated force for keeping the controller floating.
    pub float_force: FloatForce,

    /// How to keep the controller upright, as well as
    /// facing a specific direction.
    pub upright: Upright,
    /// Calculated force for keeping the controller upright.
    pub upright_force: UprightForce,

    /// How should the forces be applied to the physics engine.
    pub force_settings: ForceSettings,
}

impl Default for Controller {
    fn default() -> Self {
        Self {
            gravity: default(),
            gravity_force: default(),

            ground_caster: default(),
            ground_cast: default(),
            viable_ground_cast: default(),
            grounded: default(),
            ground_force: default(),

            movement: default(),
            movement_force: default(),
            jump: default(),
            jump_force: default(),

            float: default(),
            float_force: default(),
            upright: default(),
            upright_force: default(),

            force_settings: default(),
        }
    }
}

/// Settings for how the forces applied to the physics engine should be calculated.
#[derive(Component, Reflect)]
#[reflect(Component, Default)]
pub struct ForceSettings {
    /// Scaling factor for the force applied to the ground to keep the character moving/off the ground.
    pub opposing_force_scale: f32,
    /// Scaling factor for the movement impulse applied to the ground.
    /// Setting this to 0.0 would make it so things don't "slip" out from the characters feet.
    pub opposing_movement_force_scale: f32,
}

impl Default for ForceSettings {
    fn default() -> Self {
        Self {
            opposing_force_scale: 1.0,
            opposing_movement_force_scale: 0.0,
        }
    }
}
