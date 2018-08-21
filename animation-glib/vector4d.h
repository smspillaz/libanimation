/*
 * animation-glib/vector4d.h
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
 * GObject boxed type 4D vector implementation.
 */
#pragma once

#include <glib-object.h>

G_BEGIN_DECLS

typedef struct {
  double x;
  double y;
  double z;
  double w;
} AnimationVector4D;

#define ANIMATION_TYPE_VECTOR4D animation_vector4d_get_type ()

GType animation_vector4d_get_type (void);

G_END_DECLS
