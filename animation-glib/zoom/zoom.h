/*
 * animation-glib/zoom/zoom.h
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
 * GObject Interface for "zoom" animation.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/box.h>
#include <animation-glib/vector.h>
#include <animation-glib/stepper/stepper.h>
#include <animation-glib/transform/transform.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_ZOOM_ANIMATION animation_zoom_animation_get_type ()
G_DECLARE_FINAL_TYPE (AnimationZoomAnimation, animation_zoom_animation, ANIMATION, ZOOM_ANIMATION, AnimationTransformAnimation)

void animation_zoom_animation_set_from (AnimationZoomAnimation *animation,
                                        AnimationBox            box);
void animation_zoom_animation_get_from (AnimationZoomAnimation *animation,
                                        AnimationBox           *out_box);

void animation_zoom_animation_get_to (AnimationZoomAnimation *animation,
                                      AnimationBox           *out_box);

void animation_zoom_animation_set_stepper (AnimationZoomAnimation *animation,
                                           AnimationStepper       *stepper);
AnimationStepper * animation_zoom_animation_get_stepper (AnimationZoomAnimation *animation);

AnimationZoomAnimation * animation_zoom_new (const AnimationBox *from,
                                             const AnimationBox *to,
                                             AnimationStepper   *stepper);

G_END_DECLS
