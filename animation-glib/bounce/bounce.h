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
#include <animation-glib/vector.h>
#include <animation-glib/transform/transform.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_BOUNCE_ANIMATION animation_bounce_animation_get_type ()
G_DECLARE_FINAL_TYPE (AnimationBounceAnimation, animation_bounce_animation, ANIMATION, BOUNCE_ANIMATION, AnimationTransformAnimation)

AnimationBounceAnimation * animation_bounce_new (float              initial_scale,
                                                 float              maximum_scale,
                                                 unsigned int       n_bounce,
                                                 const AnimationBox *target,
                                                 unsigned int       length);

G_END_DECLS

#pragma once
