/*
 * animation-glib/wobbly/anchor.h
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
 * GObject Interface for "wobbly" textures, Anchor type.
 *
 * An anchor is an object type privately holding an
 * anchor. The owner can release the anchor, which happens
 * implicitly when its ref-count drops to zero or when
 * the release() method is called.
 */
#ifndef WOBBLY_GLIB_ANCHOR_H
#define WOBBLY_GLIB_ANCHOR_H

#include <glib-object.h>

#include <animation-glib/vector.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_WOBBLY_ANCHOR animation_wobbly_anchor_get_type ()
G_DECLARE_FINAL_TYPE (AnimationWobblyAnchor, animation_wobbly_anchor, ANIMATION, WOBBLY_ANCHOR, GObject)

void animation_wobbly_anchor_move_by (AnimationWobblyAnchor *anchor,
                                      AnimationVector        vector);

void animation_wobbly_anchor_release (AnimationWobblyAnchor *anchor);

G_END_DECLS

#endif
