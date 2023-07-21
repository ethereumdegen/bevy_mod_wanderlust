use crate::{controller::*, physics::*};
use bevy::prelude::*;
#[cfg(feature = "rapier3d")]
use bevy_rapier3d::prelude::*;
#[cfg(feature = "rapier2d")]
use bevy_rapier2d::prelude::*;

/// Contains common physics settings for character controllers.
#[derive(Bundle)]
pub struct RapierPhysicsBundle {
    /// See [`RigidBody`].
    pub rigidbody: RigidBody,
    /// See [`Collider`].
    pub collider: Collider,
    /// See [`Velocity`].
    pub velocity: Velocity,
    /// See [`GravityScale`].
    pub gravity: GravityScale,
    /// See [`Sleeping`].
    pub sleeping: Sleeping,
    /// See [`Ccd`].
    pub ccd: Ccd,
    /// See [`ExternalImpulse`].
    pub force: ExternalImpulse,
    /// See [`LockedAxes`].
    pub locked_axes: LockedAxes,
    /// See [`Friction`].
    pub friction: Friction,
    /// See [`Damping`].
    pub damping: Damping,
    /// See [`Restitution`].
    pub restitution: Restitution,
    /// See [`ReadMassProperties`].
    pub read_mass_properties: ReadMassProperties,
}

impl Default for RapierPhysicsBundle {
    fn default() -> Self {
        Self {
            rigidbody: default(),
            collider: Collider::capsule(Vec3::new(0.0, 0.0, 0.0), Vec3::new(0.0, 0.5, 0.0), 0.5),
            velocity: default(),
            gravity: GravityScale(0.0),
            sleeping: default(),
            ccd: default(),
            force: default(),
            locked_axes: default(),
            friction: Friction {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
            damping: Damping {
                linear_damping: 0.0,
                angular_damping: 0.0,
            },
            restitution: Restitution {
                coefficient: 0.0,
                combine_rule: CoefficientCombineRule::Min,
            },
            read_mass_properties: default(),
        }
    }
}

/// Apply forces to the controller to make it float, move, jump, etc.
pub fn apply_forces(
    mut forces: Query<(&mut ExternalImpulse, &ControllerForce)>,
    ctx: Res<RapierContext>,
) {
    let dt = ctx.integration_parameters.dt;
    for (mut impulse, force) in &mut forces {
        impulse.impulse += force.linear * dt;
        impulse.torque_impulse += force.angular * dt;
    }
}

/// Apply the opposing ground force to the entity we are pushing off of to float.
pub fn apply_ground_forces(
    mut impulses: Query<&mut ExternalImpulse>,
    ground_forces: Query<(&GroundForce, &GroundCast)>,
    ctx: Res<RapierContext>,
) {
    let dt = ctx.integration_parameters.dt;
    for (force, cast) in &ground_forces {
        if let GroundCast::Touching(ground) = cast {
            if let Some(ground_body) = ctx.collider_parent(ground.entity) {
                if let Ok(mut impulse) = impulses.get_mut(ground_body) {
                    impulse.impulse += force.linear * dt;
                    impulse.torque_impulse += force.angular * dt;
                }
            }
        }
    }
}

/// Sync rapier masses over to our masses.
pub fn get_mass_from_rapier(mut query: Query<(&mut ControllerMass, &ReadMassProperties)>) {
    for (mut mass, rapier_mass) in &mut query {
        mass.mass = rapier_mass.0.mass;
        mass.inertia = rapier_mass.0.principal_inertia;
        mass.com = rapier_mass.0.local_center_of_mass;
    }
}

/// Sync rapier velocities over to our velocities.
pub fn get_velocity_from_rapier(mut query: Query<(&mut ControllerVelocity, &Velocity)>) {
    for (mut vel, rapier_vel) in &mut query {
        vel.linear = rapier_vel.linvel;
        vel.angular = rapier_vel.angvel;
    }
}

/// *Note: Most users will not need to use this directly. Use [`WanderlustPlugin`](crate::plugins::WanderlustPlugin) instead.
/// Alternatively, if one only wants to disable the system, use [`WanderlustPhysicsTweaks`](WanderlustPhysicsTweaks).*
///
/// This system adds some tweaks to rapier's physics settings that make the character controller behave better.
pub fn setup_physics_context(mut ctx: ResMut<RapierContext>) {
    let params = &mut ctx.integration_parameters;
    // This prevents any noticeable jitter when running facefirst into a wall.
    params.erp = 0.99;
    // This prevents (most) noticeable jitter when running facefirst into an inverted corner.
    params.max_velocity_iterations = 16;
    // TODO: Fix jitter that occurs when running facefirst into a normal corner.
}