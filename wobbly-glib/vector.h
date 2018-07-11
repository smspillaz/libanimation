/*
 * wobbly-glib/vector.h
 *
 * GObject Interface for "wobbly" textures, 2D vector type
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_GLIB_VECTOR_H
#define WOBBLY_GLIB_VECTOR_H

#include <glib-object.h>

G_BEGIN_DECLS

typedef struct {
  double x;
  double y;
} WobblyVector;

#define WOBBLY_TYPE_VECTOR wobbly_vector_get_type ()

GType wobbly_vector_get_type (void);

G_END_DECLS

#endif
