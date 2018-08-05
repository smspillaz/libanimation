/*
 * animation-glib/box.cpp
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
 * GObject Interface for "wobbly" textures, box
 * type implementation.
 */

#include <animation-glib/box.h>

static gpointer
animation_box_copy (gpointer ptr)
{
  AnimationBox *src = reinterpret_cast <AnimationBox *> (ptr);
  AnimationBox *dst = g_new0 (AnimationBox, 1);

  *dst = *src;

  return reinterpret_cast <gpointer> (dst);
}

G_DEFINE_BOXED_TYPE (AnimationBox,
                     animation_box,
                     animation_box_copy,
                     g_free);
