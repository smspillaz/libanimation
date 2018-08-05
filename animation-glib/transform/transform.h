/*
 * animation-glib/transform/transform.h
 *
 * Copyright 2018 Endless Mobile, Inc.
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
 * GObject Interface for affine transform animations.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/vector.h>
#include <animation-glib/vector4d.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_TRANSFORM_ANIMATION animation_transform_animation_get_type ()
G_DECLARE_DERIVABLE_TYPE (AnimationTransformAnimation, animation_transform_animation, ANIMATION, TRANSFORM_ANIMATION, GObject)

struct _AnimationTransformAnimationClass {
  GObjectClass parent_class;
};

gboolean animation_transform_animation_step (AnimationTransformAnimation *transform_animation,
                                             unsigned int                 ms);

float animation_transform_animation_progress (AnimationTransformAnimation *transform_animation);

float const * animation_transform_animation_matrix (AnimationTransformAnimation *transform_animation);

void animation_transform_animation_extremes (AnimationTransformAnimation *transform_animation,
                                             AnimationVector const       *corners,
                                             AnimationVector4D           *out_extremes);

G_END_DECLS
