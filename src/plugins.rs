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
        app.configure_set(Update, WanderlustSet.before(bevy_rapier3d::prelude::PhysicsSet::SyncBackend));
        #[cfg(feature = "rapier2d")]
        app.configure_set(Update, WanderlustSet.before(bevy_rapier2d::prelude::PhysicsSet::SyncBackend));

        app.add_systems(
            Update,
            (
                crate::backend::get_mass_from_backend,
                crate::backend::get_velocity_from_backend,
                find_ground,
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
                .chain()
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
