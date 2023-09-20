use crate::controller::*;

 
use bevy::utils::HashSet;

use bevy::prelude::*;
use bevy_xpbd_3d::prelude::Collider;

 use crate::physics::ControllerVelocity;
 use crate::backend::xpbd::QueryFilter;
 
 
 
/*use bevy_rapier3d::{
    na::Isometry3,
    parry::{
        bounding_volume::BoundingVolume,
        query::{DefaultQueryDispatcher, PersistentQueryDispatcher},
    },
    rapier::geometry::ContactManifold,
};*/




/// Performs groundcasting and updates controller state accordingly.
pub fn find_ground(
    time: Res<Time>,
    mut casters: Query<(
        Entity,
        &GlobalTransform,
        &Gravity,
        &mut GroundCaster,
        &mut GroundCast,
        &mut ViableGroundCast,
    )>,

    velocities: Query<&Velocity>,
    masses: Query<&ReadMassProperties>,
    globals: Query<&GlobalTransform>,
    colliders: Query<&Collider>,

    ctx: Res<RapierContext>,
    mut gizmos: Gizmos,
) {
    let dt = ctx.integration_parameters.dt;
    if time.delta_seconds() == 0.0 {
        return;
    }

    for (entity, tf, gravity, mut caster, mut ground, mut viable_ground) in &mut casters {
        if caster.skip_ground_check_timer == 0.0 && !caster.skip_ground_check_override {
            let cast_position = tf.transform_point(caster.cast_origin);
            let cast_rotation = tf.to_scale_rotation_translation().1;
            let cast_direction = -gravity.up_vector;
            let Ok(caster_collider) = colliders.get(entity) else { continue };
            let shape = caster.cast_collider.as_ref().unwrap_or(caster_collider);

            let predicate =
                |collider| collider != entity && !caster.exclude_from_ground.contains(&collider);
            let filter = QueryFilter::new().exclude_sensors().predicate(&predicate);

            let mut viable_params = GroundCastParams {
                position: cast_position,
                rotation: cast_rotation,
                direction: cast_direction,
                shape: &shape,
                max_toi: caster.cast_length,
                filter: filter,
            };

            let mut any_params = viable_params.clone();

            let next_viable_ground = viable_params
                .viable_cast_iters(
                    &*ctx,
                    &globals,
                    caster.max_ground_angle,
                    gravity.up_vector,
                    5,
                    &mut gizmos,
                )
                .map(|(entity, cast)| {
                    Ground::from_cast(
                        entity,
                        cast,
                        gravity.up_vector,
                        &*caster,
                        &*ctx,
                        &masses,
                        &velocities,
                        &globals,
                    )
                });
            viable_ground.update(next_viable_ground);

            let next_ground = any_params
                .cast_iters(&*ctx, &globals, gravity.up_vector, 5, &mut gizmos)
                .map(|(entity, cast)| {
                    Ground::from_cast(
                        entity,
                        cast,
                        gravity.up_vector,
                        &*caster,
                        &*ctx,
                        &masses,
                        &velocities,
                        &globals,
                    )
                });
            ground.update(next_ground);
        } else {
            caster.skip_ground_check_timer = (caster.skip_ground_check_timer - dt).max(0.0);
        };

        /*
        let next_ground = match casted {
            Some((entity, result)) => {
                let ground_entity = ctx.collider_parent(entity).unwrap_or(entity);

                let mass = if let Ok(mass) = masses.get(ground_entity) {
                    mass.0.clone()
                } else {
                    MassProperties::default()
                };

                let local_com = mass.local_center_of_mass;

                let ground_velocity = velocities
                    .get(ground_entity)
                    .copied()
                    .unwrap_or(Velocity::default());

                let global = globals
                    .get(ground_entity)
                    .unwrap_or(&GlobalTransform::IDENTITY);
                let com = global.transform_point(local_com);
                let point_velocity =
                    ground_velocity.linvel + ground_velocity.angvel.cross(result.point - com);

                let (stable, viable) = if result.normal.length() > 0.0 {
                    let ground_angle = result.normal.angle_between(gravity.up_vector);
                    let viable = ground_angle <= caster.max_ground_angle;
                    let stable = ground_angle <= caster.unstable_ground_angle && viable;
                    (stable, viable)
                } else {
                    (false, false)
                };

                Some(Ground {
                    entity: ground_entity,
                    cast: result,
                    stable: stable,
                    viable: viable,
                    linear_velocity: ground_velocity.linvel,
                    angular_velocity: ground_velocity.angvel,
                    point_velocity: point_velocity,
                })
            }
            None => None,
        };
        */

        // If we hit something, just get back up instead of waiting.
        if ctx.contacts_with(entity).next().is_some() {
            caster.skip_ground_check_timer = 0.0;
        }
    }
}

/// Are we currently touching the ground with a fudge factor included.
pub fn determine_groundedness(
    mut query: Query<(
        &GlobalTransform,
        &Gravity,
        &Float,
        &ViableGroundCast,
        &ControllerVelocity,
        &mut Grounded,
    )>,
) {
    for (global, gravity, float, viable_ground, velocity, mut grounded) in &mut query {
        grounded.0 = false;
        if let Some(ground) = viable_ground.current() {
            let up_velocity = velocity.linear.dot(gravity.up_vector);
            let translation = global.translation();
            let updated_toi =
                translation.dot(gravity.up_vector) - ground.cast.point.dot(gravity.up_vector);
            //gizmos.sphere(ground.cast.point, Quat::IDENTITY, 0.3, Color::RED);
            //gizmos.sphere(translation, Quat::IDENTITY, 0.3, Color::GREEN);
            let offset = float.distance - updated_toi;

            //let up_velocity = up_velocity.clamp(-float.distance, float.distance);
            // Loosen constraints based on velocity.
            let max = if up_velocity > float.max_offset {
                float.max_offset + up_velocity
            } else {
                float.max_offset
            };
            let min = if up_velocity < float.min_offset {
                float.min_offset + up_velocity
            } else {
                float.min_offset
            };
            grounded.0 = offset >= min && offset <= max;
            /*
            info!(
                "grounded: {:?}, {:.3?} <= {:.3?} <= {:.3?}",
                grounded.0, min, offset, max
            );
            */
        };
    }
}
