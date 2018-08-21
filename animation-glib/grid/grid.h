/*
 * animation-glib/grid/grid.h
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
 * GObject base class for grid animations.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/vector.h>
#include <animation-glib/vector4d.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_GRID_ANIMATION animation_grid_animation_get_type ()
G_DECLARE_DERIVABLE_TYPE (AnimationGridAnimation, animation_grid_animation, ANIMATION, GRID_ANIMATION, GObject)

struct _AnimationGridAnimationClass {
  GObjectClass parent_class;
};

gboolean animation_grid_animation_step (AnimationGridAnimation *grid_animation,
                                        unsigned int       ms);

float animation_grid_animation_progress (AnimationGridAnimation *grid_animation);

void animation_grid_animation_deform_uv_to_model_space (AnimationGridAnimation *grid_animation,
                                                        AnimationVector        *uv,
                                                        AnimationVector        *model_space_point);

void animation_grid_animation_resolution (AnimationGridAnimation *grid_animation,
                                          AnimationVector        *out_resolution);

void animation_grid_animation_extremes (AnimationGridAnimation *grid_animation,
                                        AnimationVector const  *corners,
                                        AnimationVector4D      *out_extremes);

G_END_DECLS
