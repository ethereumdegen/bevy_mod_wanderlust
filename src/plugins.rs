use crate::controller::*;
use bevy::prelude::*;

/// The [character controller](CharacterController) plugin. Necessary to have the character controller
/// work.
pub struct WanderlustPlugin {
    tweaks: bool,
}

impl WanderlustPlugin {
    /// Apply tweaks to rapier to try to avoid some jitters/issues.
    pub fn do_tweaks(tweaks: bool) -> Self {
        Self { tweaks }
    }
}

impl Default for WanderlustPlugin {
    fn default() -> Self {
        Self { tweaks: true }
    }
}

#[derive(SystemSet, Copy, Clone, Debug, Default, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct WanderlustSet;

impl Plugin for WanderlustPlugin {
    fn build(&self, app: &mut App) {
        app.register_type::<ControllerInput>()
            .register_type::<Option<Vec3>>();

        if self.tweaks {
            app.add_systems(Startup, crate::backend::setup_physics_context);
        }

        #[cfg(feature = "rapier3d")]
        app.configure_set(
            Update,
            WanderlustSet.before(bevy_rapier3d::prelude::PhysicsSet::SyncBackend),
        );
        #[cfg(feature = "rapier2d")]
        app.configure_set(
            Update,
            WanderlustSet.before(bevy_rapier2d::prelude::PhysicsSet::SyncBackend),
        );

        app.add_systems(
            Update,
            (
               // find_ground,
                //determine_groundedness,
                crate::backend::find_ground,
                determine_groundedness,
                gravity_force,
                movement_force,
                float_force,
                upright_force,
                jump_force,
                accumulate_forces,
                crate::backend::apply_forces,
                crate::backend::apply_ground_forces,
            )
                .in_set(WanderlustSet)
                .chain(),
        );

        #[cfg(feature = "debug-lines")]
        app.add_systems(Update, |casts: Query<&GroundCast>, mut gizmos: Gizmos| {
            for cast in &casts {
                if let Some((entity, toi, velocity)) = cast.cast {
                    gizmos.sphere(toi.witness1, Quat::IDENTITY, 0.3, Color::LIME_GREEN);
                }
            }
        });
    }
}
 

/// *Note: Most users will not need to use this directly. Use [`WanderlustPlugin`](crate::plugins::WanderlustPlugin) instead.
/// Alternatively, if one only wants to disable the system, use [`WanderlustPhysicsTweaks`](WanderlustPhysicsTweaks).*
///
/// This system adds some tweaks to rapier's physics settings that make the character controller behave better.
pub fn setup_physics_context(/*mut ctx: ResMut<RapierContext>*/) {
    /*
    let params = &mut ctx.integration_parameters;
    // This prevents any noticeable jitter when running facefirst into a wall.
    params.erp = 0.99;
    // This prevents (most) noticeable jitter when running facefirst into an inverted corner.
    params.max_velocity_iterations = 16;
    // TODO: Fix jitter that occurs when running facefirst into a normal corner.
    */
}
 
