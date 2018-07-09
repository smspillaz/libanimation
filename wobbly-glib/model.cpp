/*
 * wobbly-glib/wobbly.cpp
 *
 * GObject Interface for "wobbly" textures
 *
 * See LICENCE.md for Copyright information
 */

#include <wobbly-glib/anchor.h>
#include <wobbly-glib/model.h>
#include <wobbly-glib/vector.h>

#include <wobbly/wobbly.h>

#include "wobbly-anchor-private.h"

struct _WobblyModel
{
  GObject parent_instance;
};

typedef struct _WobblyModelPrivate
{
  wobbly::Model::Settings  settings;
  wobbly::Model           *model;

  /* We need to keep track of these during
   * construction while model isn't set */
  WobblyVector             prop_position;
  WobblyVector             prop_size;
} WobblyModelPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (WobblyModel,
                            wobbly_model,
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

static GParamSpec *wobbly_model_props [NPROPS] = { NULL, };

namespace wgd = wobbly::geometry::dimension;

/**
 * wobbly_model_grab_anchor:
 * @model: A #WobblyModel
 * @position: A #WobblyVector specifying the position in absolute space
 *            where the closest object in the mesh should be grabbed.
 *
 * Grab the closest control point in the mesh to @position and
 * return it as a #WobblyAnchor. The control point will not have spring
 * force exerted upon it, though other control points will have forces
 * exerted on them by the anchored control point.
 *
 * The control point remains grabbed until wobbly_anchor_release() is
 * called on the anchor, or the anchor is destroyed.
 *
 * Returns: (transfer full): A #WobblyAnchor controlling the closest control
 *                           point to @position.
 */
WobblyAnchor *
wobbly_model_grab_anchor (WobblyModel *model,
                          WobblyVector position)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));
  wobbly::Anchor anchor (priv->model->GrabAnchor (wobbly::Point (position.x,
                                                                 position.y)));

  return wobbly_anchor_new_for_native_anchor_rvalue (std::move (anchor));
}

/**
 * wobbly_model_insert_anchor:
 * @model: A #WobblyModel
 * @position: A #WobblyVector specifying the position in absolute space
 *            where the anchoring control point should be inserted
 *
 * Insert a new control point into the mesh at @position and split the
 * closest spring to it in half, with both halves attaching to the
 * inserted control point. The control point will not have spring
 * force exerted upon it, though other control points will have forces
 * exerted on them by the anchored control point.
 *
 * The control point remains in place until wobbly_anchor_release() is
 * called on the anchor, or the anchor is destroyed.
 *
 * Returns: (transfer full): A #WobblyAnchor controlling the inserted control
 *                           at @position.
 */
WobblyAnchor *
wobbly_model_insert_anchor (WobblyModel *model,
                            WobblyVector position)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));
  wobbly::Anchor anchor (priv->model->InsertAnchor (wobbly::Point (position.x,
                                                                   position.y)));

  return wobbly_anchor_new_for_native_anchor_rvalue (std::move (anchor));
}

/**
 * wobbly_model_step:
 * @model: A #WobblyModel
 * @ms: Time period to integrate over.
 *
 * Compute all the instantaneous forces exerted by springs in the
 * mesh and integrate over @ms to determine the velocities and
 * changes in model control point positions.
 *
 * Returns: %TRUE if further integration is required, %FALSE otherwise.
 */
gboolean
wobbly_model_step (WobblyModel  *model,
                   unsigned int  ms)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  return priv->model->Step (ms);
}

/**
 * wobbly_model_deform_texcoords:
 * @model: A #WobblyModel
 * @uv: A #WobblyVector specifying the unit coordinates relative to
 *      the top right corner of a square to be deformed.
 * @deformed: (out caller-allocates): A #WobblyVector specifying the
 *             absolute position in space that the unit co-ordinate would
 *             be deformed to.
 *
 * Deform texture-coordinates into real space according to the model.
 */
void
wobbly_model_deform_texcoords (WobblyModel  *model,
                               WobblyVector  uv,
                               WobblyVector *deformed)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  g_return_if_fail (deformed != NULL);

  wobbly::Point deformed_point (priv->model->DeformTexcoords (wobbly::Point (uv.x, uv.y)));
  *deformed = {
    wobbly::geometry::dimension::get <0> (deformed_point),
    wobbly::geometry::dimension::get <1> (deformed_point)
  };
}

/**
 * wobbly_model_deform_query_extremes:
 * @model: A #WobblyModel
 * @uv: A #WobblyVector specifying the unit coordinates relative to
 *      the top right corner of a square to be deformed.
 * @top_left: (out caller-allocates): The top left extreme.
 * @top_right: (out caller-allocates): The top right extreme.
 * @bottom_left: (out caller-allocates): The bottom left extreme.
 * @bottom_right: (out caller-allocates): The bottom right extreme.
 *
 * Query the bounding 4-vertex polygon around the model.
 */
void
wobbly_model_query_extremes (WobblyModel  *model,
                             WobblyVector *top_left,
                             WobblyVector *top_right,
                             WobblyVector *bottom_left,
                             WobblyVector *bottom_right)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

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
 * wobbly_model_move_to:
 * @model: A #WobblyModel
 * @position: A #WobblyVector to move the top right corner to.
 *
 * Move the entire model to have its top right corner at @position
 * without causing any deformation.
 */
void
wobbly_model_move_to (WobblyModel *model,
                      WobblyVector position)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->prop_position = position;

  if (priv->model != nullptr)
    priv->model->MoveModelTo (wobbly::Point (position.x, position.y));
}

/**
 * wobbly_model_move_by:
 * @model: A #WobblyModel
 * @delta: A #WobblyVector to move the top right corner by.
 *
 * Move the entire model by @position without causing any deformation.
 */
void
wobbly_model_move_by (WobblyModel *model,
                      WobblyVector delta)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->prop_position.x += delta.x;
  priv->prop_position.y += delta.y;

  if (priv->model != nullptr)
    priv->model->MoveModelBy (wobbly::Point (delta.x, delta.y));
}

/**
 * wobbly_model_resize:
 * @model: A #WobblyModel
 * @size: A #WobblyVector specifying the size in each dimension.
 *
 * Resize the model according to @size.
 */
void
wobbly_model_resize (WobblyModel *model,
                     WobblyVector size)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->prop_size = size;

  if (priv->model != nullptr)
    priv->model->ResizeModel (size.x, size.y);
}

void
wobbly_model_set_spring_k (WobblyModel *model,
                           double       spring_constant)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->settings.springConstant = spring_constant;
}

void
wobbly_model_set_friction (WobblyModel *model,
                           double       friction)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->settings.friction = friction;
}

void
wobbly_model_set_maximum_range (WobblyModel *model,
                                double       range)
{
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->settings.maximumRange = range;
}

static void
wobbly_model_set_property (GObject      *object,
                           guint         prop_id,
                           const GValue *value,
                           GParamSpec   *pspec)
{
  WobblyModel *model = WOBBLY_MODEL (object);

  switch (prop_id)
    {
    case PROP_SPRING_K:
      wobbly_model_set_spring_k (model, g_value_get_double (value));
      break;
    case PROP_FRICTION:
      wobbly_model_set_friction (model, g_value_get_double (value));
      break;
    case PROP_MAXIMUM_RANGE:
      wobbly_model_set_maximum_range (model, g_value_get_double (value));
      break;
    case PROP_POSITION:
      wobbly_model_move_to (model, *(reinterpret_cast <WobblyVector *> (g_value_get_boxed (value))));
      break;
    case PROP_SIZE:
      wobbly_model_resize (model, *(reinterpret_cast <WobblyVector *> (g_value_get_boxed (value))));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
wobbly_model_get_property (GObject    *object,
                           guint       prop_id,
                           GValue     *value,
                           GParamSpec *pspec)
{
  WobblyModel *model = WOBBLY_MODEL (object);
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

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
wobbly_model_finalize (GObject *object)
{
  WobblyModel *model = WOBBLY_MODEL (object);
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  delete priv->model;
  priv->model = nullptr;

  G_OBJECT_CLASS (wobbly_model_parent_class)->finalize (object);
}

static void
wobbly_model_constructed (GObject *object)
{
  WobblyModel *model = WOBBLY_MODEL (object);
  WobblyModelPrivate *priv =
    reinterpret_cast <WobblyModelPrivate *> (wobbly_model_get_instance_private (model));

  priv->model = new wobbly::Model (wobbly::Point (priv->prop_position.x,
                                                  priv->prop_position.y),
                                   priv->prop_size.x,
                                   priv->prop_size.y,
                                   priv->settings);
}

static void
wobbly_model_init (WobblyModel *model)
{
}


static void
wobbly_model_class_init (WobblyModelClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructed = wobbly_model_constructed;
  object_class->get_property = wobbly_model_get_property;
  object_class->set_property = wobbly_model_set_property;
  object_class->finalize = wobbly_model_finalize;

  wobbly_model_props[PROP_SPRING_K] =
    g_param_spec_double ("spring-k",
                         "Spring Constant",
                         "Multiplier for force exerted by springs",
                         0.1,
                         10.0,
                         wobbly::Model::DefaultSpringConstant,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  wobbly_model_props[PROP_FRICTION] =
    g_param_spec_double ("friction",
                         "Friction Constant",
                         "Multiplier for friction exerted by springs",
                         0.1,
                         10.0,
                         wobbly::Model::Friction,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  wobbly_model_props[PROP_MAXIMUM_RANGE] =
    g_param_spec_double ("movement-range",
                         "Object Movement Range",
                         "How far away connected points can be from their rest point",
                         10.0,
                         1000.0,
                         wobbly::Model::DefaultObjectRange,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  wobbly_model_props[PROP_POSITION] =
    g_param_spec_boxed ("position",
                        "Model position",
                        "Where the model is in absolute 2D space",
                        WOBBLY_TYPE_VECTOR,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  wobbly_model_props[PROP_SIZE] =
    g_param_spec_boxed ("size",
                        "Model size",
                        "Where how big the model is in 2D space",
                        WOBBLY_TYPE_VECTOR,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     wobbly_model_props);
}

WobblyModel *
wobbly_model_new (WobblyVector position,
                  WobblyVector size,
                  double       spring_constant,
                  double       friction,
                  double       maximum_range)
{
  return WOBBLY_MODEL (g_object_new (WOBBLY_TYPE_MODEL,
                                     "spring-k", spring_constant,
                                     "friction", friction,
                                     "movement-range", maximum_range,
                                     "position", &position,
                                     "size", &size,
                                     nullptr));
}
