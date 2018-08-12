/*
 * animation-glib/glide/glide.cpp
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
 * GObject implementation for a "glide" animation.
 */

#include <animation/glide/glide.h>

#include <animation-glib/box.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/glide/glide.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace ag = animation::glide;

struct _AnimationGlideAnimation
{
  GObject parent_instance;
};

typedef struct _AnimationGlideAnimationPrivate
{
} AnimationGlideAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationGlideAnimation,
                            animation_glide_animation,
                            ANIMATION_TYPE_TRANSFORM_ANIMATION)

enum {
  PROP_0,
  PROP_INITIAL_DISTANCE,
  PROP_X_ROTATION_ANGLE_DEGREES,
  PROP_Y_ROTATION_ANGLE_DEGREES,
  PROP_X_AXIS_LOCATION_UNIT,
  PROP_Y_AXIS_LOCATION_UNIT,
  PROP_TARGET,
  PROP_LENGTH,
  NPROPS
};

static GParamSpec *animation_glide_animation_props [NPROPS] = { NULL, };

static void
animation_glide_animation_set_property (GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec)
{
  switch (prop_id)
    {
    case PROP_INITIAL_DISTANCE:
    case PROP_X_ROTATION_ANGLE_DEGREES:
    case PROP_Y_ROTATION_ANGLE_DEGREES:
    case PROP_X_AXIS_LOCATION_UNIT:
    case PROP_Y_AXIS_LOCATION_UNIT:
    case PROP_TARGET:
    case PROP_LENGTH:
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_glide_animation_get_property (GObject    *object,
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

static GObject *
animation_glide_animation_constructor (GType                  type,
                                       unsigned int           n_construct_params,
                                       GObjectConstructParam *construct_params)
{
  const char * const wanted_properties[] = {
    "initial-distance",
    "x-rotation-angle-degrees",
    "y-rotation-angle-degrees",
    "x-axis-location-unit",
    "y-axis-location-unit",
    "target",
    "length",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    InterfaceConstructor <ag::GlideAnimation>::construct (
      ForwardFromValueHT (properties_table, g_value_get_float, "initial-distance"),
      ForwardFromValueHT (properties_table, g_value_get_float, "x-rotation-angle-degrees"),
      ForwardFromValueHT (properties_table, g_value_get_float, "y-rotation-angle-degrees"),
      ForwardFromValueHT (properties_table, g_value_get_float, "x-axis-location-unit"),
      ForwardFromValueHT (properties_table, g_value_get_float, "y-axis-location-unit"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "target"),
      ForwardFromValueHT (properties_table, g_value_get_uint, "length")
    );

  unsigned int n_extended_construct_params;
  GObjectConstructParam *extended_construct_params =
    append_interface_prop_to_construct_params (construct_params,
                                               n_construct_params,
                                               G_OBJECT_CLASS (animation_glide_animation_parent_class),
                                               g_steal_pointer (&interface),
                                               &n_extended_construct_params);

  return G_OBJECT_CLASS (animation_glide_animation_parent_class)->constructor (type,
                                                                              n_extended_construct_params,
                                                                              extended_construct_params);
}

static void
animation_glide_animation_init (AnimationGlideAnimation *model)
{
}


static void
animation_glide_animation_class_init (AnimationGlideAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_glide_animation_constructor;
  object_class->get_property = animation_glide_animation_get_property;
  object_class->set_property = animation_glide_animation_set_property;

  animation_glide_animation_props[PROP_INITIAL_DISTANCE] =
    g_param_spec_float ("initial-distance",
                        "Initial Distance",
                        "The initial distance away from the camera",
                        0.1f,
                        1.0f,
                        0.7f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_X_ROTATION_ANGLE_DEGREES] =
    g_param_spec_float ("x-rotation-angle-degrees",
                        "X Rotation Angle Degrees",
                        "Number of degrees on the X axis to rotate",
                        0.0f,
                        360.0f,
                        0.0f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_Y_ROTATION_ANGLE_DEGREES] =
    g_param_spec_float ("y-rotation-angle-degrees",
                        "Y Rotation Angle Degrees",
                        "Number of degrees on the Y axis to rotate",
                        0.0f,
                        360.0f,
                        0.0f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_X_AXIS_LOCATION_UNIT] =
    g_param_spec_float ("x-axis-location-unit",
                        "X Axis Location Unit",
                        "Unit-coordinates of where the X axis is on the surface",
                        0.0f,
                        1.0f,
                        0.5f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_Y_AXIS_LOCATION_UNIT] =
    g_param_spec_float ("y-axis-location-unit",
                        "Y Axis Location Unit",
                        "Unit-coordinates of where the Y axis is on the surface",
                        0.0f,
                        1.0f,
                        0.2f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_TARGET] =
    g_param_spec_pointer ("target",
                          "Target Box",
                          "Box that we are animating to",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_LENGTH] =
    g_param_spec_uint ("length",
                       "Length",
                       "How long the animation lasts",
                       1,
                       5000,
                       300,
                       static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_glide_animation_props);
}

/**
 * animation_glide_new:
 * @initial_distance: Initial distance frm the camera.
 * @x_rotation_angle_degrees: Degrees of rotation towards the X axis.
 * @y_rotation_angle_degrees: Degrees of rotation towards the Y axis.
 * @y_axis_location_unit: Unit-coordinates of where the X axis is on the surface.
 * @x_axis_location_unit: Unit-coordinates of where the Y axis is on the surface.
 * @target_box: The #AnimationBox that we are animating to.
 * @length: The length of the animation.
 *
 * Returns: (transfer full): A new #AnimationGlideAnimation.
 */
AnimationGlideAnimation *
animation_glide_new (float               initial_distance,
                     float               x_rotation_angle_degrees,
                     float               y_rotation_angle_degrees,
                     float               x_axis_location_unit,
                     float               y_axis_location_unit,
                     const AnimationBox *target_box,
                     unsigned int        length)
{
  return ANIMATION_GLIDE_ANIMATION (g_object_new (ANIMATION_TYPE_GLIDE_ANIMATION,
                                                  "initial-distance", initial_distance,
                                                  "x-rotation-angle-degrees", x_rotation_angle_degrees,
                                                  "y-rotation-angle-degrees", y_rotation_angle_degrees,
                                                  "x-axis-location-unit", x_axis_location_unit,
                                                  "y-axis-location-unit", y_axis_location_unit,
                                                  "target", target_box,
                                                  "length", length,
                                                  NULL));
}
