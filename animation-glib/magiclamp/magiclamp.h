/*
 * animation-glib/magiclamp/magiclamp.h
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
 * GObject Interface for "magiclamp" animation.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/box.h>
#include <animation-glib/grid/grid.h>
#include <animation-glib/vector.h>
#include <animation-glib/transform/transform.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_MAGIC_LAMP_ANIMATION animation_magiclamp_animation_get_type ()
G_DECLARE_FINAL_TYPE (AnimationMagicLampAnimation, animation_magiclamp_animation, ANIMATION, MAGIC_LAMP_ANIMATION, AnimationGridAnimation)

void animation_magiclamp_animation_set_source (AnimationMagicLampAnimation *animation,
                                               AnimationBox                 box);
void animation_magiclamp_animation_get_source (AnimationMagicLampAnimation *animation,
                                               AnimationBox                *out_box);

void animation_magiclamp_animation_get_target (AnimationMagicLampAnimation *animation,
                                               AnimationBox                *out_box);

void animation_magiclamp_animation_set_bend_factor (AnimationMagicLampAnimation *animation,
                                                   float                        bend_factor);
float animation_magiclamp_animation_get_bend_factor (AnimationMagicLampAnimation *animation);

void animation_magiclamp_animation_set_offset_factor (AnimationMagicLampAnimation *animation,
                                                      float                        offset_factor);
float animation_magiclamp_animation_get_offset_factor (AnimationMagicLampAnimation *animation);

void animation_magiclamp_animation_set_stretch_factor (AnimationMagicLampAnimation *animation,
                                                       float                        stretch_factor);
float animation_magiclamp_animation_get_stretch_factor (AnimationMagicLampAnimation *animation);

void animation_magiclamp_animation_set_deform_speed_factor (AnimationMagicLampAnimation *animation,
                                                            float                        deform_speed_factor);
float animation_magiclamp_animation_get_deform_speed_factor (AnimationMagicLampAnimation *animation);

void animation_magiclamp_animation_set_stepper (AnimationMagicLampAnimation *animation,
                                                AnimationStepper            *stepper);
AnimationStepper * animation_magiclamp_animation_get_stepper (AnimationMagicLampAnimation *animation);

AnimationMagicLampAnimation * animation_magiclamp_new (const AnimationBox    *source_box,
                                                       const AnimationBox    *target_box,
                                                       const AnimationVector *resolution,
                                                       float                  bend_factor,
                                                       float                  offset_factor,
                                                       float                  stretch_factor,
                                                       float                  deform_speed_factor,
                                                       AnimationStepper      *stepper);

G_END_DECLS
