/*
 * wobbly-glib/anchor.h
 *
 * GObject Interface for "wobbly" textures, Anchor type.
 *
 * An anchor is an object type privately holding an
 * anchor. The owner can release the anchor, which happens
 * implicitly when its ref-count drops to zero or when
 * the release() method is called.
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_GLIB_ANCHOR_H
#define WOBBLY_GLIB_ANCHOR_H

#include <glib-object.h>

#include <wobbly-glib/vector.h>

G_BEGIN_DECLS

#define WOBBLY_TYPE_ANCHOR wobbly_anchor_get_type ()
G_DECLARE_FINAL_TYPE (WobblyAnchor, wobbly_anchor, WOBBLY, ANCHOR, GObject)

void wobbly_anchor_move_by (WobblyAnchor *anchor,
                            WobblyVector  vector);

void wobbly_anchor_release (WobblyAnchor *anchor);

G_END_DECLS

#endif
