use crate::controller::*;
/// Keeps the controller properly oriented in a floating state.

#[derive(Component, Reflect)]
#[reflect(Component, Default)]
pub struct Float {
    /// How far to attempt to float away from the ground.
    pub distance: f32,
    /// While floating, the character can be floating at a different exact distance than [`float_distance`] depending on other forces acting on them.
    /// This field controls how much lower than [`float_distance`] they can be and still be considered grounded.
    ///
    /// This helps keep jumps more consistent when the ground cast length is longer than the float distance.
    pub min_offset: f32,
    /// While floating, the character can be floating at a different exact distance than [`float_distance`] depending on other forces acting on them.
    /// This field controls how much higher than [`float_distance`] they can be and still be considered grounded.
    ///
    /// This helps keep jumps more consistent when the ground cast length is longer than the float distance.
    pub max_offset: f32,
    /// How strongly to float away from the ground.
    pub spring: Spring,
}

impl Default for Float {
    fn default() -> Self {
        Self {
            distance: 0.55,
            min_offset: -0.3,
            max_offset: 0.05,
            spring: Spring {
                strength: 100.0,
                damping: 0.8,
            },
        }
    }
}

/// Force applied to push the controller off the ground.
#[derive(Component, Default, Reflect)]
#[reflect(Component, Default)]
pub struct FloatForce {
    /// Linear force.
    pub linear: Vec3,
}

/// Calculate "floating" force, as seen [here](https://www.youtube.com/watch?v=qdskE8PJy6Q)
pub fn float_force(
    mut query: Query<(
        &mut FloatForce,
        &Float,
        &GroundCast,
        &GroundCaster,
        Velocity,
        Mass,
        &Gravity,
    )>,
) {
    for (mut force, float, cast, ground_caster, velocity, mass, gravity) in &mut query {
        force.linear = Vec3::ZERO;

        let GroundCast::Touching(ground) = cast else { continue };

        let up_vector = gravity.up_vector;

        let controller_point_velocity =
            velocity.linear() + velocity.angular().cross(Vec3::ZERO - mass.local_center_of_mass());
        let vel_align = up_vector.dot(controller_point_velocity);
        let ground_vel_align = up_vector.dot(ground.linear_velocity);

        let relative_velocity = vel_align - ground_vel_align;

        let displacement = float.distance - ground.cast.toi;

        if displacement > 0.0 {
            let strength = displacement * float.spring.strength;
            let damping = relative_velocity * float.spring.damp_coefficient(mass.mass());
            force.linear += up_vector * (strength - damping);
        }
    }
}

/// How to keep the controller upright, as well as
/// facing a specific direction.
#[derive(Component, Reflect)]
#[reflect(Component, Default)]
pub struct Upright {
    /// How strongly to force the character upright/avoid overshooting. Alternatively, see [`LockedAxes`] to lock rotation entirely.
    pub spring: Spring,
    /// The direction to face towards, or `None` to not rotate to face any direction. Must be perpendicular to the up vector and normalized.
    pub forward_vector: Option<Vec3>,
}

impl Default for Upright {
    fn default() -> Self {
        Self {
            spring: Spring {
                strength: 10.0,
                damping: 0.5,
            },
            forward_vector: None,
        }
    }
}

/// Forces applied to keep the controller upright and optionally facing a direction.
#[derive(Component, Default, Reflect)]
#[reflect(Component, Default)]
pub struct UprightForce {
    /// Angular force.
    pub angular: Vec3,
}

pub fn upright_force(
    mut query: Query<(
        &mut UprightForce,
        &Upright,
        &GlobalTransform,
        &Gravity,
        Mass,
        Velocity,
        &GroundCast,
    )>,
) {
    for (mut impulse, upright, tf, gravity, mass, velocity, ground_cast) in &mut query {
        impulse.angular = {
            let desired_axis = if let Some(forward) = upright.forward_vector {
                let right = gravity.up_vector.cross(forward).normalize();
                let up = forward.cross(right);
                let target_rot = Quat::from_mat3(&Mat3::from_cols(right, up, forward));
                let current = tf.to_scale_rotation_translation().1;
                let rot = target_rot * current.inverse();
                let (axis, mut angle) = rot.to_axis_angle();
                if angle > std::f32::consts::PI {
                    angle -= 2.0 * std::f32::consts::PI;
                }
                axis * angle
            } else {
                let current = tf.up();
                current.cross(gravity.up_vector)
            };

            //upright.spring.damping = 0.1;

            let damping = Vec3::new(
                upright.spring.damp_coefficient(mass.inertia().x),
                upright.spring.damp_coefficient(mass.inertia().y),
                upright.spring.damp_coefficient(mass.inertia().z),
            );

            let ground_rot = if let Some(ground) = ground_cast.last() {
                ground.angular_velocity
            } else {
                Vec3::ZERO
            };

            let velocity = velocity.angular() - ground_rot;

            let spring = (desired_axis * upright.spring.strength) - (velocity * damping);
            spring.clamp_length_max(upright.spring.strength)
        };
    }
}
