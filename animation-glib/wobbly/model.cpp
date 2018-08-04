/*
 * animation-glib/wobbly/wobbly.cpp
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
 * GObject Interface for "wobbly" textures.
 */

#include <animation-glib/wobbly/anchor.h>
#include <animation-glib/wobbly/model.h>
#include <animation-glib/wobbly/vector.h>

#include <animation/wobbly/wobbly.h>

#include "wobbly-anchor-private.h"

struct _AnimationWobblyModel
{
  GObject parent_instance;
};

typedef struct _AnimationWobblyModelPrivate
{
  wobbly::Model::Settings  settings;
  wobbly::Model           *model;

  /* We need to keep track of these during
   * construction while model isn't set */
  AnimationVector             prop_position;
  AnimationVector             prop_size;
} AnimationWobblyModelPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationWobblyModel,
                            animation_wobbly_model,
                            G_TYPE_OBJECT)

enum {
  PROP_0,
  PROP_SPRING_K,
  PROP_FRICTION,
  PROP_MAXIMUM_RANGE,
  PROP_POSITION,
  PROP_SIZE,
  NPROPS
};

static GParamSpec *animation_wobbly_model_props [NPROPS] = { NULL, };

namespace wgd = wobbly::geometry::dimension;

/**
 * animation_wobbly_model_grab_anchor:
 * @model: A #AnimationWobblyModel
 * @position: A #AnimationVector specifying the position in absolute space
 *            where the closest object in the mesh should be grabbed.
 *
 * Grab the closest control point in the mesh to @position and
 * return it as a #AnimationWobblyAnchor. The control point will not have spring
 * force exerted upon it, though other control points will have forces
 * exerted on them by the anchored control point.
 *
 * The control point remains grabbed until animation_wobbly_anchor_release() is
 * called on the anchor, or the anchor is destroyed.
 *
 * Returns: (transfer full): A #AnimationWobblyAnchor controlling the closest control
 *                           point to @position.
 */
AnimationWobblyAnchor *
animation_wobbly_model_grab_anchor (AnimationWobblyModel *model,
                                    AnimationVector       position)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));
  wobbly::Anchor anchor (priv->model->GrabAnchor (wobbly::Point (position.x,
                                                                 position.y)));

  return animation_wobbly_anchor_new_for_native_anchor_rvalue (std::move (anchor));
}

/**
 * animation_wobbly_model_insert_anchor:
 * @model: A #AnimationWobblyModel
 * @position: A #AnimationVector specifying the position in absolute space
 *            where the anchoring control point should be inserted
 *
 * Insert a new control point into the mesh at @position and split the
 * closest spring to it in half, with both halves attaching to the
 * inserted control point. The control point will not have spring
 * force exerted upon it, though other control points will have forces
 * exerted on them by the anchored control point.
 *
 * The control point remains in place until animation_wobbly_anchor_release() is
 * called on the anchor, or the anchor is destroyed.
 *
 * Returns: (transfer full): A #AnimationWobblyAnchor controlling the inserted control
 *                           at @position.
 */
AnimationWobblyAnchor *
animation_wobbly_model_insert_anchor (AnimationWobblyModel *model,
                                      AnimationVector       position)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));
  wobbly::Anchor anchor (priv->model->InsertAnchor (wobbly::Point (position.x,
                                                                   position.y)));

  return animation_wobbly_anchor_new_for_native_anchor_rvalue (std::move (anchor));
}

/**
 * animation_wobbly_model_step:
 * @model: A #AnimationWobblyModel
 * @ms: Time period to integrate over.
 *
 * Compute all the instantaneous forces exerted by springs in the
 * mesh and integrate over @ms to determine the velocities and
 * changes in model control point positions.
 *
 * Returns: %TRUE if further integration is required, %FALSE otherwise.
 */
gboolean
animation_wobbly_model_step (AnimationWobblyModel *model,
                             unsigned int          ms)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  return priv->model->Step (ms);
}

/**
 * animation_wobbly_model_deform_texcoords:
 * @model: A #AnimationWobblyModel
 * @uv: A #AnimationVector specifying the unit coordinates relative to
 *      the top right corner of a square to be deformed.
 * @deformed: (out caller-allocates): A #AnimationVector specifying the
 *             absolute position in space that the unit co-ordinate would
 *             be deformed to.
 *
 * Deform texture-coordinates into real space according to the model.
 */
void
animation_wobbly_model_deform_texcoords (AnimationWobblyModel *model,
                                         AnimationVector       uv,
                                         AnimationVector      *deformed)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  g_return_if_fail (deformed != NULL);

  wobbly::Point deformed_point (priv->model->DeformTexcoords (wobbly::Point (uv.x, uv.y)));
  *deformed = {
    wobbly::geometry::dimension::get <0> (deformed_point),
    wobbly::geometry::dimension::get <1> (deformed_point)
  };
}

/**
 * animation_wobbly_model_deform_query_extremes:
 * @model: A #AnimationWobblyModel
 * @uv: A #AnimationVector specifying the unit coordinates relative to
 *      the top right corner of a square to be deformed.
 * @top_left: (out caller-allocates): The top left extreme.
 * @top_right: (out caller-allocates): The top right extreme.
 * @bottom_left: (out caller-allocates): The bottom left extreme.
 * @bottom_right: (out caller-allocates): The bottom right extreme.
 *
 * Query the bounding 4-vertex polygon around the model.
 */
void
animation_wobbly_model_query_extremes (AnimationWobblyModel *model,
                                       AnimationVector      *top_left,
                                       AnimationVector      *top_right,
                                       AnimationVector      *bottom_left,
                                       AnimationVector      *bottom_right)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  std::array <wobbly::Point, 4> extremes (priv->model->Extremes ());

  if (top_left != nullptr)
    *top_left = { wgd::get <0> (extremes[0]), wgd::get <1> (extremes[0]) };

  if (top_right != nullptr)
    *top_right = { wgd::get <0> (extremes[1]), wgd::get <1> (extremes[1]) };

  if (bottom_left != nullptr)
    *bottom_left = { wgd::get <0> (extremes[2]), wgd::get <1> (extremes[2]) };

  if (bottom_right != nullptr)
    *bottom_right = { wgd::get <0> (extremes[3]), wgd::get <1> (extremes[3]) };
}

/**
 * animation_wobbly_model_move_to:
 * @model: A #AnimationWobblyModel
 * @position: A #AnimationVector to move the top right corner to.
 *
 * Move the entire model to have its top right corner at @position
 * without causing any deformation.
 */
void
animation_wobbly_model_move_to (AnimationWobblyModel *model,
                                AnimationVector       position)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->prop_position = position;

  if (priv->model != nullptr)
    priv->model->MoveModelTo (wobbly::Point (position.x, position.y));
}

/**
 * animation_wobbly_model_move_by:
 * @model: A #AnimationWobblyModel
 * @delta: A #AnimationVector to move the top right corner by.
 *
 * Move the entire model by @position without causing any deformation.
 */
void
animation_wobbly_model_move_by (AnimationWobblyModel *model,
                                AnimationVector       delta)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->prop_position.x += delta.x;
  priv->prop_position.y += delta.y;

  if (priv->model != nullptr)
    priv->model->MoveModelBy (wobbly::Point (delta.x, delta.y));
}

/**
 * animation_wobbly_model_resize:
 * @model: A #AnimationWobblyModel
 * @size: A #AnimationVector specifying the size in each dimension.
 *
 * Resize the model according to @size.
 */
void
animation_wobbly_model_resize (AnimationWobblyModel *model,
                               AnimationVector       size)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->prop_size = size;

  if (priv->model != nullptr)
    priv->model->ResizeModel (size.x, size.y);
}

void
animation_wobbly_model_set_spring_k (AnimationWobblyModel *model,
                                     double                spring_constant)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->settings.springConstant = spring_constant;
}

void
animation_wobbly_model_set_friction (AnimationWobblyModel *model,
                                     double                friction)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->settings.friction = friction;
}

void
animation_wobbly_model_set_maximum_range (AnimationWobblyModel *model,
                                          double                range)
{
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->settings.maximumRange = range;
}

static void
animation_wobbly_model_set_property (GObject      *object,
                                     guint         prop_id,
                                     const GValue *value,
                                     GParamSpec   *pspec)
{
  AnimationWobblyModel *model = ANIMATION_WOBBLY_MODEL (object);

  switch (prop_id)
    {
    case PROP_SPRING_K:
      animation_wobbly_model_set_spring_k (model, g_value_get_double (value));
      break;
    case PROP_FRICTION:
      animation_wobbly_model_set_friction (model, g_value_get_double (value));
      break;
    case PROP_MAXIMUM_RANGE:
      animation_wobbly_model_set_maximum_range (model, g_value_get_double (value));
      break;
    case PROP_POSITION:
      animation_wobbly_model_move_to (model, *(reinterpret_cast <AnimationVector *> (g_value_get_boxed (value))));
      break;
    case PROP_SIZE:
      animation_wobbly_model_resize (model, *(reinterpret_cast <AnimationVector *> (g_value_get_boxed (value))));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_wobbly_model_get_property (GObject    *object,
                                     guint       prop_id,
                                     GValue     *value,
                                     GParamSpec *pspec)
{
  AnimationWobblyModel *model = ANIMATION_WOBBLY_MODEL (object);
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  switch (prop_id)
    {
    case PROP_SPRING_K:
      g_value_set_double (value, priv->settings.springConstant);
      break;
    case PROP_FRICTION:
      g_value_set_double (value, priv->settings.friction);
      break;
    case PROP_MAXIMUM_RANGE:
      g_value_set_double (value, priv->settings.maximumRange);
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_wobbly_model_finalize (GObject *object)
{
  AnimationWobblyModel *model = ANIMATION_WOBBLY_MODEL (object);
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  delete priv->model;
  priv->model = nullptr;

  G_OBJECT_CLASS (animation_wobbly_model_parent_class)->finalize (object);
}

static void
animation_wobbly_model_constructed (GObject *object)
{
  AnimationWobblyModel *model = ANIMATION_WOBBLY_MODEL (object);
  AnimationWobblyModelPrivate *priv =
    reinterpret_cast <AnimationWobblyModelPrivate *> (animation_wobbly_model_get_instance_private (model));

  priv->model = new wobbly::Model (wobbly::Point (priv->prop_position.x,
                                                  priv->prop_position.y),
                                   priv->prop_size.x,
                                   priv->prop_size.y,
                                   priv->settings);
}

static void
animation_wobbly_model_init (AnimationWobblyModel *model)
{
}


static void
animation_wobbly_model_class_init (AnimationWobblyModelClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructed = animation_wobbly_model_constructed;
  object_class->get_property = animation_wobbly_model_get_property;
  object_class->set_property = animation_wobbly_model_set_property;
  object_class->finalize = animation_wobbly_model_finalize;

  animation_wobbly_model_props[PROP_SPRING_K] =
    g_param_spec_double ("spring-k",
                         "Spring Constant",
                         "Multiplier for force exerted by springs",
                         0.1,
                         10.0,
                         wobbly::Model::DefaultSpringConstant,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_wobbly_model_props[PROP_FRICTION] =
    g_param_spec_double ("friction",
                         "Friction Constant",
                         "Multiplier for friction exerted by springs",
                         0.1,
                         10.0,
                         wobbly::Model::Friction,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_wobbly_model_props[PROP_MAXIMUM_RANGE] =
    g_param_spec_double ("movement-range",
                         "Object Movement Range",
                         "How far away connected points can be from their rest point",
                         10.0,
                         1000.0,
                         wobbly::Model::DefaultObjectRange,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_wobbly_model_props[PROP_POSITION] =
    g_param_spec_boxed ("position",
                        "Model position",
                        "Where the model is in absolute 2D space",
                        ANIMATION_TYPE_VECTOR,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  animation_wobbly_model_props[PROP_SIZE] =
    g_param_spec_boxed ("size",
                        "Model size",
                        "Where how big the model is in 2D space",
                        ANIMATION_TYPE_VECTOR,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_wobbly_model_props);
}

AnimationWobblyModel *
animation_wobbly_model_new (AnimationVector position,
                            AnimationVector size,
                            double       spring_constant,
                            double       friction,
                            double       maximum_range)
{
  return ANIMATION_WOBBLY_MODEL (g_object_new (ANIMATION_WOBBLY_TYPE_MODEL,
                                               "spring-k", spring_constant,
                                               "friction", friction,
                                               "movement-range", maximum_range,
                                               "position", &position,
                                               "size", &size,
                                               nullptr));
}
