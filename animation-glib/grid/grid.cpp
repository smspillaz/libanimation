/*
 * animation-glib/grid/grid.cpp
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
 * GObject base class for grid animations.
 */

#include <animation/grid/grid.h>

#include <animation-glib/grid/grid.h>
#include <animation-glib/vector.h>
#include <animation-glib/vector4d.h>
#include <animation-glib/vector4d-internal.h>

namespace agd = animation::geometry::dimension;
namespace agr = animation::grid;

typedef struct _AnimationGridAnimationPrivate
{
  agr::GridAnimation *interface;
} AnimationGridAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationGridAnimation,
                            animation_grid_animation,
                            G_TYPE_OBJECT)

enum {
  PROP_0,
  PROP_INTERFACE,
  NPROPS
};

static GParamSpec *animation_grid_animation_props [NPROPS] = { NULL, };

gboolean
animation_grid_animation_step (AnimationGridAnimation *grid_animation,
                               unsigned int            ms)
{
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  return priv->interface->Step (ms);
}

/**
 * animation_grid_animation_deform_uv_to_model_space:
 * @grid_animation: An #AnimationGridAnimation
 * @uv: An #AnimationVector representing a point in unit coordinate
 *      space to be deformed.
 * @model_space_point: (out caller-allocates): An #AnimationVector to write
 *                                             the deformed co-ordinate to.
 *
 * Determine where a unit coordinate lies in model space.
 */
void
animation_grid_animation_deform_uv_to_model_space (AnimationGridAnimation *grid_animation,
                                                   AnimationVector        *uv,
                                                   AnimationVector        *model_space_point)
{
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  g_return_if_fail (model_space_point != NULL);

  animation::Point point (uv->x, uv->y);
  animation::Point deformed (priv->interface->DeformUVToModelSpace (point));

  model_space_point->x = agd::get <0> (deformed);
  model_space_point->y = agd::get <1> (deformed);
}

/**
 * animation_grid_animation_resolution:
 * @grid_animation: An #AnimationGridAnimation
 * @out_resolution: (out caller-allocates): An #AnimationVector specifying the
 *                  ideal resolution of the animated surface grid.
 *
 * Return the expected grid resolution that would be required to
 * make this animation look smooth. The renderer should subdivide
 * the animated surface into this many equal sized chunks.
 */
void
animation_grid_animation_resolution (AnimationGridAnimation *grid_animation,
                                     AnimationVector        *out_resolution)
{
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  g_return_if_fail (out_resolution != NULL);

  auto resolution_cpp (priv->interface->Resolution ());

  out_resolution->x = agd::get <0> (resolution_cpp);
  out_resolution->y = agd::get <1> (resolution_cpp);
}

/**
 * animation_grid_animation_extremes:
 * @grid_animation: A #AnimationGridAnimation
 * @corners: (array fixed-size=4): The four #AnimationVector4D values
 *           describing the location of the surface corners.
 * @out_extremes: (array fixed-size=4) (out): The grided four #AnimationVector
 *                values describing the location of the grided surface
 *                surface corners.
 *
 * Get the four co-ordinates of a 3D plane which bound the animated surface.
 */
void
animation_grid_animation_extremes (AnimationGridAnimation *grid_animation,
                                   AnimationVector const  *corners,
                                   AnimationVector4D      *out_extremes)
{
  g_return_if_fail (corners != NULL);
  g_return_if_fail (out_extremes != NULL);

  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  std::array <animation::Point, 4> points = {
    animation::Point (corners[0].x, corners[0].y),
    animation::Point (corners[1].x, corners[1].y),
    animation::Point (corners[2].x, corners[2].y),
    animation::Point (corners[3].x, corners[3].y)
  };

  std::array <animation::Vector4D, 4> extremes = priv->interface->Extremes (points);

  agd::assign (out_extremes[0], extremes[0]);
  agd::assign (out_extremes[1], extremes[1]);
  agd::assign (out_extremes[2], extremes[2]);
  agd::assign (out_extremes[3], extremes[3]);
}

float
animation_grid_animation_progress (AnimationGridAnimation *grid_animation)
{
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  return priv->interface->Progress ();
}

static void
animation_grid_animation_set_property (GObject      *object,
                                            guint         prop_id,
                                            const GValue *value,
                                            GParamSpec   *pspec)
{
  AnimationGridAnimation *grid_animation = ANIMATION_GRID_ANIMATION (object);
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  switch (prop_id)
    {
    case PROP_INTERFACE:
      priv->interface = reinterpret_cast <agr::GridAnimation *> (g_value_get_pointer (value));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_grid_animation_get_property (GObject    *object,
                                       guint       prop_id,
                                       GValue     *value,
                                       GParamSpec *pspec)
{
  AnimationGridAnimation *grid_animation = ANIMATION_GRID_ANIMATION (object);
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  switch (prop_id)
    {
    case PROP_INTERFACE:
      g_value_set_pointer (value, reinterpret_cast <gpointer> (priv->interface));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_grid_animation_finalize (GObject *object)
{
  AnimationGridAnimation *grid_animation = ANIMATION_GRID_ANIMATION (object);
  AnimationGridAnimationPrivate *priv =
    reinterpret_cast <AnimationGridAnimationPrivate *> (animation_grid_animation_get_instance_private (grid_animation));

  delete priv->interface;
  priv->interface = nullptr;

  G_OBJECT_CLASS (animation_grid_animation_parent_class)->finalize (object);
}

static void
animation_grid_animation_init (AnimationGridAnimation *model)
{
}


static void
animation_grid_animation_class_init (AnimationGridAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->get_property = animation_grid_animation_get_property;
  object_class->set_property = animation_grid_animation_set_property;
  object_class->finalize = animation_grid_animation_finalize;

  animation_grid_animation_props[PROP_INTERFACE] =
    g_param_spec_pointer ("interface",
                          "Internal Interface",
                          "Internal C++ interface that this class wraps",
                          static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_grid_animation_props);
}
