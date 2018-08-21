/*
 * animation-glib/vector4d.cpp
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

#include <animation-glib/vector4d.h>

static gpointer
animation_vector4d_copy (gpointer ptr)
{
  AnimationVector4D *src = reinterpret_cast <AnimationVector4D *> (ptr);
  AnimationVector4D *dst = g_new0 (AnimationVector4D, 1);

  *dst = *src;

  return reinterpret_cast <gpointer> (dst);
}

G_DEFINE_BOXED_TYPE (AnimationVector4D,
                     animation_vector4d,
                     animation_vector4d_copy,
                     g_free);
