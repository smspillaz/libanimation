/*
 * animation-glib/glide/glide.h
 *
 * libanimation is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * libanimation is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with eos-companion-app-service.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * GObject Interface for "glide" animation.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/box.h>
#include <animation-glib/vector.h>
#include <animation-glib/transform/transform.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_GLIDE_ANIMATION animation_glide_animation_get_type ()
G_DECLARE_FINAL_TYPE (AnimationGlideAnimation, animation_glide_animation, ANIMATION, GLIDE_ANIMATION, AnimationTransformAnimation)

void animation_glide_animation_set_initial_distance (AnimationGlideAnimation *animation,
                                                     float                    initial_distance);
float animation_glide_animation_get_initial_distance (AnimationGlideAnimation *animation);

void animation_glide_animation_set_x_rotation_angle_degrees (AnimationGlideAnimation *animation,
                                                             float                    x_rotation_angle_degrees);
float animation_glide_animation_get_x_rotation_angle_degrees (AnimationGlideAnimation *animation);

void animation_glide_animation_set_y_rotation_angle_degrees (AnimationGlideAnimation *animation,
                                                             float                    y_rotation_angle_degrees);
float animation_glide_animation_get_y_rotation_angle_degrees (AnimationGlideAnimation *animation);

void animation_glide_animation_set_x_axis_location_unit (AnimationGlideAnimation *animation,
                                                         float                    x_axis_location_unit);

float animation_glide_animation_get_x_axis_location_unit (AnimationGlideAnimation *animation);
void animation_glide_animation_set_y_axis_location_unit (AnimationGlideAnimation *animation,
                                                         float                    y_axis_location_unit);

float animation_glide_animation_get_y_axis_location_unit (AnimationGlideAnimation *animation);

void animation_glide_animation_set_stepper (AnimationGlideAnimation *animation,
                                            AnimationStepper        *stepper);
AnimationStepper * animation_glide_animation_get_stepper (AnimationGlideAnimation *animation);

AnimationGlideAnimation * animation_glide_new (float               initial_distance,
                                               float               x_rotation_angle_degrees,
                                               float               y_rotation_angle_degrees,
                                               float               x_axis_location_unit,
                                               float               y_axis_location_unit,
                                               unsigned int        screen_width,
                                               const AnimationBox *target_box,
                                               unsigned int        length);

G_END_DECLS
