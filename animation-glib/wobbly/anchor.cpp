/*
 * animation-glib/wobbly/anchor.cpp
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
 * GObject Interface for "wobbly" textures, Anchor
 * type implementation.
 */

#include <animation-glib/wobbly/anchor.h>

#include <animation/wobbly/wobbly.h>

#include "wobbly-anchor-private.h"

struct _AnimationWobblyAnchor
{
  GObject parent_instance;
};

typedef struct _AnimationWobblyAnchorPrivate
{
  wobbly::Anchor *anchor;
} AnimationWobblyAnchorPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationWobblyAnchor,
                            animation_wobbly_anchor,
                            G_TYPE_OBJECT);

/**
 * animation_wobbly_anchor_move_by:
 * @anchor: A #AnimationWobblyAnchor
 * @vector: A #AnimationVector to move the anchor by
 *
 * Move the anchor by @vector, causing force to be exerted
 * on the underlying anchor's model.
 */
void
animation_wobbly_anchor_move_by (AnimationWobblyAnchor *anchor,
                                 AnimationVector        vector)
{
  AnimationWobblyAnchorPrivate *priv =
    reinterpret_cast <AnimationWobblyAnchorPrivate *> (animation_wobbly_anchor_get_instance_private (anchor));

  if (priv->anchor != nullptr)
    priv->anchor->MoveBy (animation::Point (vector.x, vector.y));
}


/**
 * animation_wobbly_anchor_release:
 * @anchor: A #AnimationWobblyAnchor
 *
 * Release the anchor, allowing the relevant control points
 * to move freely and have force exerted on them. Attempting
 * to move the anchor past this point is inert.
 */
void
animation_wobbly_anchor_release (AnimationWobblyAnchor *anchor)
{
  AnimationWobblyAnchorPrivate *priv =
    reinterpret_cast <AnimationWobblyAnchorPrivate *> (animation_wobbly_anchor_get_instance_private (anchor));

  if (priv->anchor != nullptr)
    {
      delete priv->anchor;
      priv->anchor = nullptr;
    }
}

static void
animation_wobbly_anchor_finalize (GObject *object)
{
  AnimationWobblyAnchor *anchor = ANIMATION_WOBBLY_ANCHOR (object);

  animation_wobbly_anchor_release (anchor);
}

static void
animation_wobbly_anchor_init (AnimationWobblyAnchor *store)
{
}


static void
animation_wobbly_anchor_class_init (AnimationWobblyAnchorClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->finalize = animation_wobbly_anchor_finalize;
}

AnimationWobblyAnchor *
animation_wobbly_anchor_new_for_native_anchor_rvalue (wobbly::Anchor &&anchor)
{
  AnimationWobblyAnchor *anchor_object =
    ANIMATION_WOBBLY_ANCHOR (g_object_new (ANIMATION_TYPE_WOBBLY_ANCHOR, NULL));
  AnimationWobblyAnchorPrivate *priv =
    reinterpret_cast <AnimationWobblyAnchorPrivate *> (animation_wobbly_anchor_get_instance_private (anchor_object));

  priv->anchor = new wobbly::Anchor (std::move (anchor));

  return anchor_object;
}
