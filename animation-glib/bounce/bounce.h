/*
 * animation-glib/bounce/bounce.h
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
 * GObject Interface for "bounce" animation.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/box.h>
#include <animation-glib/stepper/stepper.h>
#include <animation-glib/transform/transform.h>
#include <animation-glib/vector.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_BOUNCE_ANIMATION animation_bounce_animation_get_type ()
G_DECLARE_FINAL_TYPE (AnimationBounceAnimation, animation_bounce_animation, ANIMATION, BOUNCE_ANIMATION, AnimationTransformAnimation)

float animation_bounce_animation_get_initial_scale (AnimationBounceAnimation *animation);
void animation_bounce_animation_set_initial_scale (AnimationBounceAnimation *animation,
                                                   float                     initial_scale);

float animation_bounce_animation_get_maximum_scale (AnimationBounceAnimation *animation);
void animation_bounce_animation_set_maximum_scale (AnimationBounceAnimation *animation,
                                                   float                     maximum_scale);

unsigned int animation_bounce_animation_get_n_bounce (AnimationBounceAnimation *animation);
void animation_bounce_animation_set_n_bounce (AnimationBounceAnimation *animation,
                                              unsigned int              n_bounce);

void animation_bounce_animation_get_target (AnimationBounceAnimation *animation,
                                            AnimationBox             *out_box);

void animation_bounce_animation_set_stepper (AnimationBounceAnimation *animation,
                                             AnimationStepper         *stepper);
AnimationStepper * animation_bounce_animation_get_stepper (AnimationBounceAnimation *animation);

AnimationBounceAnimation * animation_bounce_new (float               initial_scale,
                                                 float               maximum_scale,
                                                 unsigned int        n_bounce,
                                                 const AnimationBox *target,
                                                 AnimationStepper   *stepper);

G_END_DECLS

#pragma once
