use crate::controller::*;

 
use bevy::utils::HashSet;

use bevy::prelude::*;
use bevy_xpbd_3d::math::Scalar;
use bevy_xpbd_3d::parry::query::ContactManifold;
use bevy_xpbd_3d::prelude::{Collider, LinearVelocity, Mass };
use bevy_xpbd_3d::resources::DeltaTime;

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



impl Ground {
    /// Construct a `Ground` based on the results of `GroundCastParams`.
    pub fn from_cast(
        entity: Entity,
        cast: CastResult,
        up_vector: Vec3,
        caster: &GroundCaster,
       // ctx: &RapierContext,
        //masses: &Query<&ReadMassProperties>,
         masses: &Query<&Mass>,
        velocities: &Query<&LinearVelocity>,
        globals: &Query<&GlobalTransform>,
    ) -> Self {
        let ground_entity = ctx.collider_parent(entity).unwrap_or(entity);

        let mass = if let Ok(mass) = masses.get(ground_entity) {
            mass.0.clone()
        } else {
            Mass::default()
        };

        let local_com = mass.local_center_of_mass;

        let ground_velocity = velocities
            .get(ground_entity)
            .copied()
            .unwrap_or(LinearVelocity::default());

        let global = globals
            .get(ground_entity)
            .unwrap_or(&GlobalTransform::IDENTITY);
        let com = global.transform_point(local_com);
        let point_velocity =
            ground_velocity.linvel + ground_velocity.angvel.cross(cast.point - com);

        let (stable, viable) = if cast.normal.length() > 0.0 {
            let viable = cast.viable(up_vector, caster.max_ground_angle);
            let stable = cast.viable(up_vector, caster.unstable_ground_angle) && viable;
            (stable, viable)
        } else {
            (false, false)
        };

        Ground {
            entity: ground_entity,
            cast: cast,
            stable: stable,
            viable: viable,
            linear_velocity: ground_velocity.linvel,
            angular_velocity: ground_velocity.angvel,
            point_velocity: point_velocity,
        }
    }
}




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

    velocities: Query<&LinearVelocity>,
    masses: Query<&Mass>,
    globals: Query<&GlobalTransform>,
    colliders: Query<&Collider>,
    
    delta_time: Res<DeltaTime>, 
    //ctx: Res<XpbdContext>,   //this get us dt 
    mut gizmos: Gizmos,
) {
    let dt = delta_time.0 ;
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
                  //  &*ctx,
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
                      //  &*ctx,
                        &masses,
                        &velocities,
                        &globals,
                    )
                });
            viable_ground.update(next_viable_ground);

            let next_ground = any_params
                .cast_iters(
                  //  &*ctx, 
                    &globals, gravity.up_vector, 5, &mut gizmos)
                .map(|(entity, cast)| {
                    Ground::from_cast(
                        entity,
                        cast,
                        gravity.up_vector,
                        &*caster,
                        //&*ctx,
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







/// Parameters to robust ground shape/raycasting.
#[derive(Clone)]
pub struct GroundCastParams<'c, 'f> {
    /// Position of the shape in world-space.
    pub position: Vec3,
    /// Rotation of the shape.
    pub rotation: Quat,
    /// Direction to cast, this should be normalized.
    pub direction: Vec3,
    /// Shape to use in the shapecast.
    pub shape: &'c Collider,
    /// Maximum distance we should cast.
    pub max_toi: f32,
    /// Filter collider types/entities from this ground cast.
    pub filter: QueryFilter ,
}

/// Arbitrary "slop"/"fudge" amount to adjust various things.
pub const FUDGE: f32 = 0.05;

impl<'c, 'f> GroundCastParams<'c, 'f> {
    /// Ground cast
    pub fn cast_iters(
        &mut self,
     //   ctx: &XpbdContext,
        globals: &Query<&GlobalTransform>,
        up_vector: Vec3,
        iterations: usize,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        for _ in 0..iterations {
            if let Some((entity, cast)) = self.cast(
                //ctx,
                 globals, up_vector, gizmos) {
                return Some((entity, cast));
            }
        }

        None
    }

    /// Robust viable ground casting, will try multiple times
    /// if the cast fails to find viable ground.
    pub fn viable_cast_iters(
        &mut self,
      //  ctx: &XpbdContext,
        globals: &Query<&GlobalTransform>,
        max_angle: f32,
        up_vector: Vec3,
        iterations: usize,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        for _ in 0..iterations {
            if let Some((entity, cast)) =
                self.viable_cast(
                  //  ctx,
                     globals, up_vector, max_angle, gizmos)
            {
                return Some((entity, cast));
            }
        }

        None
    }

    /// Find the first ground we can cast to.
    pub fn cast(
        &mut self,
       // ctx: &XpbdContext,
        globals: &Query<&GlobalTransform>,
        up_vector: Vec3,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        self.correct_penetrations(
            //ctx, 
            globals
            );

        let (entity, mut cast) = if let Some((entity, cast)) = self.cast_shape(ctx, gizmos) {
            (entity, cast)
        } else {
            if let Some((entity, cast)) = self.cast_ray(ctx) {
                (entity, cast)
            } else {
                return None;
            }
        };
        let Some(sampled_normal) = self.sample_normals(
            //ctx,
             cast, 
             up_vector, 
             gizmos
             ) else { return None };
        cast.normal = sampled_normal;

        // Either none of the samples
        if cast.normal.length_squared() > 0.0 {
            Some((entity, cast))
        } else {
            None
        }
    }

    /// Robust viable ground casting.
    pub fn viable_cast(
        &mut self,
       // ctx: &XpbdContext,
        globals: &Query<&GlobalTransform>,
        up_vector: Vec3,
        max_angle: f32,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        let Some((entity, cast)) = self.cast(
            //ctx,
             globals, up_vector, gizmos) else { return None };

        if cast.viable(up_vector, max_angle) {
            Some((entity, cast))
        } else {
            self.slide(cast, up_vector, gizmos);
            None
        }
    }

    /// Push the ground cast parameteres out of any colliders it is penetrating.
    pub fn correct_penetrations(
        &mut self,
      //   ctx: &XpbdContext, 
         globals: &Query<&GlobalTransform>) {
        let manifolds =
            contact_manifolds(
             //   ctx, 
                self.position, self.rotation, self.shape, &self.filter);

        for (entity, manifold) in &manifolds {
            let local_normal: Vec3 = manifold.local_n2.into();

            let Ok(contact_global) = globals.get(*entity) else { continue };
            let normal = contact_global.to_scale_rotation_translation().1 * local_normal;
            //for point in &manifold.points {
            let correction = normal * 0.05;
            self.position += correction;
            //}
        }
    }

    /// Cast a shape downwards using the parameters.
    pub fn cast_shape(
        &self,
      //  ctx: &XpbdContext,
        gizmos: &mut Gizmos,
    ) -> Option<(Entity, CastResult)> {
        let Some((entity, toi)) = ctx
            .cast_shape(self.position, self.rotation, self.direction, self.shape, self.max_toi, self.filter) else { return None };

       
       //how can i add this back in ? 
       /* if toi.status == TOIStatus::Penetrating || toi.toi <= std::f32::EPSILON {
            return None;
        }*/

        let (entity, cast) = (entity, CastResult::from_toi1(toi));

        gizmos.ray(self.position, self.direction * cast.toi, Color::BLUE);
        gizmos.sphere(
            self.position + self.direction * cast.toi,
            self.rotation,
            0.3,
            Color::BLUE,
        );

        Some((entity, cast))
    }

    /// A fallback to a simple raycasting downwards.
    ///
    /// Used in the case that we are unable to correct penetration.
    pub fn cast_ray(&self, 
        //ctx: &XpbdContext
        ) -> Option<(Entity, CastResult)> {
        // This should only occur if the controller fails to correct penetration
        // of colliders.

        // local shape offset from origin to bottom of shape
        let offset = self
            .shape
            .cast_local_ray(Vec3::ZERO, self.direction, 10.0, false)
            .unwrap_or(0.);
        let ray_pos = self.position + self.direction * offset;

        ctx.cast_ray_and_get_normal(ray_pos, self.direction, self.max_toi, true, self.filter)
            .map(|(entity, inter)| (entity, inter.into()))
    }

    /// Adjust to cast down the slope of the currently found ground.
    ///
    /// This is used so the controller doesn't repeatedly fall down
    /// a slope despite there being some viable ground right beneath the
    /// non-viable ground.
    pub fn slide(&mut self, cast: CastResult, up_vector: Vec3, gizmos: &mut Gizmos) {
        let projected_position = self.position + self.direction * cast.toi;
        //let offset = cast.point.distance(projected_position);

        let down_tangent = cast.down_tangent(up_vector);
        self.direction = down_tangent.normalize_or_zero();
        self.position = projected_position;

        gizmos.ray(cast.point, down_tangent * 0.3, Color::CYAN);
        self.max_toi -= cast.toi;
        //max_toi -= (toi.toi - offset).max(0.0);
        self.max_toi = self.max_toi.max(0.0);
    }

    /// Sample a couple of points around the contact point.
    ///
    /// This way we get more reliable normals by averaging rather than relying
    /// on just the shapecast (which tends to interpolate normals while on edges).
    pub fn sample_normals(
        &self,
        //ctx: &XpbdContext,
        cast: CastResult,
        up_vector: Vec3,
        gizmos: &mut Gizmos,
    ) -> Option<Vec3> {
        // try to get a better normal rather than an edge interpolated normal.
        // project back onto original shape position
        let ray_dir = self.direction;
        let ray_origin = cast.point + -ray_dir * cast.toi;

        let (x, z) = ray_dir.any_orthonormal_pair();
        let samples = [-x + z, -x - z, x + z, x - z, Vec3::ZERO];

        // Initial correction, sample points around the contact point
        // for the closest normal
        let mut sampled = Vec::new();
        let valid_radius = FUDGE * 2.0;
        gizmos.sphere(cast.point, Quat::IDENTITY, valid_radius, Color::RED); // Bounding sphere of valid ray normals
        for sample in samples {
            let Some((_, inter)) = ctx.cast_ray_and_get_normal(
                ray_origin - sample * FUDGE,
                ray_dir,
                self.max_toi,
                true,
                self.filter,
            ) else { continue };

            if inter.toi > 0.0
                && inter.normal.length_squared() > 0.0
                && inter.point.distance(cast.point) < valid_radius
            {
                gizmos.ray(inter.point, inter.normal * 0.2, Color::RED);
                sampled.push(inter.normal);
            }
        }

        let mut sum = Vec3::ZERO;
        let mut weights = 0.0;
        for sample in sampled {
            let alignment = sample.dot(up_vector).abs();
            sum += alignment * sample;
            weights += alignment;
        }
        let weighted_average = sum / weights;
        gizmos.ray(cast.point, weighted_average * 0.5, Color::MAROON);

        if weighted_average.length_squared() > 0.0 {
            Some(weighted_average)
        } else {
            None
        }
    }
}


fn contact_manifolds(
    
    position: Vec3,
    rotation: Quat,
    collider: &Collider,
    filter: &QueryFilter,
    
    //collisions: Res<Collisions>,
    //mut bodies: Query<(&RigidBody, &mut Position)>,
) {
    // Iterate through collisions and move the kinematic body to resolve penetration
    //for contacts in collisions.iter() {
        // If the collision didn't happen during this substep, skip the collision
        //if !contacts.during_current_substep {
        //    continue;
       // }
        if let Ok([(rb1, mut position1), (rb2, mut position2)]) =
            bodies.get_many_mut([contacts.entity1, contacts.entity2])
        {
            for manifold in contacts.manifolds.iter() {
                for contact in manifold.contacts.iter() {
                    if contact.penetration <= Scalar::EPSILON {
                        continue;
                    }
                    if rb1.is_kinematic() && !rb2.is_kinematic() {
                        position1.0 -= contact.normal * contact.penetration;
                    } else if rb2.is_kinematic() && !rb1.is_kinematic() {
                        position2.0 += contact.normal * contact.penetration;
                    }
                }
            }
        }
  //  }
}



// Get a list of contacts for a given shape.
/*pub fn contact_manifolds(
   
    position: Vec3,
    rotation: Quat,
    collider: &Collider,
    filter: &QueryFilter,
) -> Vec<(Entity, ContactManifold<>)> {
    let physics_scale = ctx.physics_scale();

    let shape = &collider.raw;
    let shape_iso = Isometry3 {
        translation: (position * physics_scale).into(),
        rotation: rotation.into(),
    };

    let shape_aabb = shape.compute_aabb(&shape_iso).loosened(FUDGE);

    let mut manifolds = Vec::new();
    ctx.query_pipeline
        .colliders_with_aabb_intersecting_aabb(&shape_aabb, |handle| {
            if let Some(collider) = ctx.colliders.get(*handle) {
                if RapierContext::with_query_filter(&ctx, *filter, |rapier_filter| {
                    rapier_filter.test(&ctx.bodies, *handle, collider)
                }) {
                    let mut new_manifolds = Vec::new();
                    let pos12 = shape_iso.inv_mul(collider.position());
                    let _ = DefaultQueryDispatcher.contact_manifolds(
                        &pos12,
                        shape.as_ref(),
                        collider.shape(),
                        0.01,
                        &mut new_manifolds,
                        &mut None,
                    );

                    if let Some(entity) = ctx.collider_entity(*handle) {
                        manifolds
                            .extend(new_manifolds.into_iter().map(|manifold| (entity, manifold)));
                    }
                }
            }

            true
        });

    manifolds
}

*/