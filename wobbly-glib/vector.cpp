/*
 * wobbly-glib/vector.cpp
 *
 * GObject Interface for "wobbly" textures, vector
 * type implementation.
 *
 * See LICENCE.md for Copyright information
 */

#include <wobbly-glib/vector.h>

static gpointer
wobbly_vector_copy (gpointer ptr)
{
  WobblyVector *src = reinterpret_cast <WobblyVector *> (ptr);
  WobblyVector *dst = g_new0 (WobblyVector, 1);

  *dst = *src;

  return reinterpret_cast <gpointer> (dst);
}

G_DEFINE_BOXED_TYPE (WobblyVector,
                     wobbly_vector,
                     wobbly_vector_copy,
                     g_free);
