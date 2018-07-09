/*
 * wobbly-glib/anchor-private.h
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

#include <wobbly/wobbly.h>

#ifndef WOBBLY_GLIB_ANCHOR_PRIVATE_H
#define WOBBLY_GLIB_ANCHOR_PRIVATE_H

WobblyAnchor * wobbly_anchor_new_for_native_anchor_rvalue (wobbly::Anchor &&anchor);

#endif
