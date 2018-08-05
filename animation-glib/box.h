/*
 * animation-glib/box.h
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
 * GObject Interface for "wobbly" textures, 2D box type.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/vector.h>

G_BEGIN_DECLS

typedef struct {
  AnimationVector top_left;
  AnimationVector bottom_right;
} AnimationBox;

#define ANIMATION_TYPE_BOX animation_box_get_type ()

GType animation_box_get_type ();

G_END_DECLS
