/*
 * wobbly-glib/anchor.cpp
 *
 * Copyright 2018 Endless Mobile, Inc.
 *
 * libwobbly is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * libwobbly is distributed in the hope that it will be useful,
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

#include <wobbly-glib/anchor.h>

#include <wobbly/wobbly.h>

#include "wobbly-anchor-private.h"

struct _WobblyAnchor
{
  GObject parent_instance;
};

typedef struct _WobblyAnchorPrivate
{
  wobbly::Anchor *anchor;
} WobblyAnchorPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (WobblyAnchor,
                            wobbly_anchor,
                            G_TYPE_OBJECT)

/**
 * wobbly_anchor_move_by:
 * @anchor: A #WobblyAnchor
 * @vector: A #WobblyVector to move the anchor by
 *
 * Move the anchor by @vector, causing force to be exerted
 * on the underlying anchor's model.
 */
void
wobbly_anchor_move_by (WobblyAnchor *anchor,
                       WobblyVector  vector)
{
  WobblyAnchorPrivate *priv =
    reinterpret_cast <WobblyAnchorPrivate *> (wobbly_anchor_get_instance_private (anchor));

  if (priv->anchor != nullptr)
    priv->anchor->MoveBy (wobbly::Point (vector.x, vector.y));
}


/**
 * wobbly_anchor_release:
 * @anchor: A #WobblyAnchor
 *
 * Release the anchor, allowing the relevant control points
 * to move freely and have force exerted on them. Attempting
 * to move the anchor past this point is inert.
 */
void
wobbly_anchor_release (WobblyAnchor *anchor)
{
  WobblyAnchorPrivate *priv =
    reinterpret_cast <WobblyAnchorPrivate *> (wobbly_anchor_get_instance_private (anchor));

  if (priv->anchor != nullptr)
    {
      delete priv->anchor;
      priv->anchor = nullptr;
    }
}

static void
wobbly_anchor_finalize (GObject *object)
{
  WobblyAnchor *anchor = WOBBLY_ANCHOR (object);

  wobbly_anchor_release (anchor);

  G_OBJECT_CLASS (wobbly_anchor_parent_class)->finalize (object);
}

static void
wobbly_anchor_init (WobblyAnchor *store)
{
}


static void
wobbly_anchor_class_init (WobblyAnchorClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->finalize = wobbly_anchor_finalize;
}

WobblyAnchor *
wobbly_anchor_new_for_native_anchor_rvalue (wobbly::Anchor &&anchor)
{
  WobblyAnchor *anchor_object =
    WOBBLY_ANCHOR (g_object_new (WOBBLY_TYPE_ANCHOR, NULL));
  WobblyAnchorPrivate *priv =
    reinterpret_cast <WobblyAnchorPrivate *> (wobbly_anchor_get_instance_private (anchor_object));

  priv->anchor = new wobbly::Anchor (std::move (anchor));

  return anchor_object;
}
