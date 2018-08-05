/*
 * animation-glib/transform/transform.cpp
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
 * GObject base class for affine transform based animations.
 */

#include <animation/transform/transform.h>

#include <animation-glib/transform/transform.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace at = animation::transform;

struct _AnimationTransformAnimation
{
  GObject parent_instance;
};

typedef struct _AnimationTransformAnimationPrivate
{
  at::TransformAnimation *interface;
} AnimationTransformAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationTransformAnimation,
                            animation_transform_animation,
                            G_TYPE_OBJECT)

enum {
  PROP_0,
  PROP_INTERFACE,
  NPROPS
};

static GParamSpec *animation_transform_animation_props [NPROPS] = { NULL, };

gboolean
animation_transform_animation_step (AnimationTransformAnimation *transform_animation,
                                    unsigned int                 ms)
{
  AnimationTransformAnimationPrivate *priv =
    reinterpret_cast <AnimationTransformAnimationPrivate *> (animation_transform_animation_get_instance_private (transform_animation));

  return priv->interface->Step (ms);
}

/**
 * animation_transform_animation_matrix: (skip)
 * @transform_animation: A #AnimationTransformAnimation
 *
 * Get the 4x4 column-major transformation matrix for this
 * representing the state of this animation.
 *
 * Returns: (array fixed-size=16): The 4x4 column-major ordered
 *          transformation matrix.
 */
float const *
animation_transform_animation_matrix (AnimationTransformAnimation *transform_animation)
{
  AnimationTransformAnimationPrivate *priv =
    reinterpret_cast <AnimationTransformAnimationPrivate *> (animation_transform_animation_get_instance_private (transform_animation));

  return priv->interface->Matrix ();
}

/**
 * animation_transform_animation_extremes:
 * @transform_animation: A #AnimationTransformAnimation
 * @corners: (array fixed-size=4): The four #AnimationVector values
 *           describing the location of the surface corners.
 * @out_extremes: (array fixed-size=4) (out): The transformed four #AnimationVector
 *                values describing the location of the transformed surface
 *                surface corners.
 *
 * Get the 4x4 column-major transformation matrix for this
 * representing the state of this animation.
 */
void
animation_transform_animation_extremes (AnimationTransformAnimation *transform_animation,
                                        AnimationVector const       *corners,
                                        AnimationVector             *out_extremes)
{
  g_return_if_fail (corners != NULL);
  g_return_if_fail (out_extremes != NULL);

  AnimationTransformAnimationPrivate *priv =
    reinterpret_cast <AnimationTransformAnimationPrivate *> (animation_transform_animation_get_instance_private (transform_animation));

  std::array <animation::Point, 4> points = {
    animation::Point (corners[0].x, corners[0].y),
    animation::Point (corners[1].x, corners[1].y),
    animation::Point (corners[2].x, corners[2].y),
    animation::Point (corners[3].x, corners[3].y)
  };

  std::array <animation::Point, 4> extremes = priv->interface->Extremes (points);

  out_extremes[0].x = agd::get <0> (extremes[0]);
  out_extremes[0].y = agd::get <1> (extremes[0]);
  out_extremes[1].x = agd::get <0> (extremes[1]);
  out_extremes[1].y = agd::get <1> (extremes[1]);
  out_extremes[2].x = agd::get <0> (extremes[2]);
  out_extremes[2].y = agd::get <1> (extremes[2]);
  out_extremes[3].x = agd::get <0> (extremes[3]);
  out_extremes[3].y = agd::get <1> (extremes[3]);
}

static void
animation_transform_animation_set_property (GObject      *object,
                                            guint         prop_id,
                                            const GValue *value,
                                            GParamSpec   *pspec)
{
  AnimationTransformAnimation *model = ANIMATION_TRANSFORM_ANIMATION (object);
  AnimationTransformAnimationPrivate *priv =
    reinterpret_cast <AnimationTransformAnimationPrivate *> (animation_transform_animation_get_instance_private (model));

  switch (prop_id)
    {
    case PROP_INTERFACE:
      priv->interface = reinterpret_cast <at::TransformAnimation *> (g_value_get_pointer (value));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_transform_animation_get_property (GObject    *object,
                                            guint       prop_id,
                                            GValue     *value,
                                            GParamSpec *pspec)
{
  switch (prop_id)
    {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_transform_animation_finalize (GObject *object)
{
  AnimationTransformAnimation *model = ANIMATION_TRANSFORM_ANIMATION (object);
  AnimationTransformAnimationPrivate *priv =
    reinterpret_cast <AnimationTransformAnimationPrivate *> (animation_transform_animation_get_instance_private (model));

  delete priv->interface;
  priv->interface = nullptr;

  G_OBJECT_CLASS (animation_transform_animation_parent_class)->finalize (object);
}

static void
animation_transform_animation_init (AnimationTransformAnimation *model)
{
}


static void
animation_transform_animation_class_init (AnimationTransformAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->get_property = animation_transform_animation_get_property;
  object_class->set_property = animation_transform_animation_set_property;
  object_class->finalize = animation_transform_animation_finalize;

  animation_transform_animation_props[PROP_INTERFACE] =
    g_param_spec_pointer ("interface",
                          "Internal Interface",
                          "Internal C++ interface that this class wraps",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_transform_animation_props);
}
