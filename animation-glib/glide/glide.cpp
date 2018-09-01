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
#include <animation-glib/stepper/linear.h>
#include <animation-glib/stepper/stepper.h>
#include <animation-glib/stepper/stepper-wrapper.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace ag = animation::glide;
namespace at = animation::transform;

struct _AnimationGlideAnimation
{
  AnimationTransformAnimation parent_instance;
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
  PROP_SCREEN_WIDTH,
  PROP_TARGET,
  PROP_STEPPER,
  NPROPS
};

static GParamSpec *animation_glide_animation_props [NPROPS] = { NULL, };

void
animation_glide_animation_set_initial_distance (AnimationGlideAnimation *animation,
                                                float                    initial_distance)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->InitialDistance (initial_distance);
}

float
animation_glide_animation_get_initial_distance (AnimationGlideAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->InitialDistance ();
}

void
animation_glide_animation_set_x_rotation_angle_degrees (AnimationGlideAnimation *animation,
                                                        float                    x_rotation_angle_degrees)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->XRotationAngleDegrees (x_rotation_angle_degrees);
}

float
animation_glide_animation_get_x_rotation_angle_degrees (AnimationGlideAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->XRotationAngleDegrees ();
}

void
animation_glide_animation_set_y_rotation_angle_degrees (AnimationGlideAnimation *animation,
                                                        float                    y_rotation_angle_degrees)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->YRotationAngleDegrees (y_rotation_angle_degrees);
}

float
animation_glide_animation_get_y_rotation_angle_degrees (AnimationGlideAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->YRotationAngleDegrees ();
}

void
animation_glide_animation_set_x_axis_location_unit (AnimationGlideAnimation *animation,
                                                    float                    x_axis_location_unit)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->XAxisLocationUnit (x_axis_location_unit);
}

float
animation_glide_animation_get_x_axis_location_unit (AnimationGlideAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->XAxisLocationUnit ();
}

void
animation_glide_animation_set_y_axis_location_unit (AnimationGlideAnimation *animation,
                                                    float                    y_axis_location_unit)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->YAxisLocationUnit (y_axis_location_unit);
}

float
animation_glide_animation_get_y_axis_location_unit (AnimationGlideAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->YAxisLocationUnit ();
}

void
animation_glide_animation_set_stepper (AnimationGlideAnimation *animation,
                                       AnimationStepper        *stepper)
{
  animation::stepper::Stepper *stepper_ptr = nullptr;
  g_object_get (stepper, "stepper", (gpointer) &stepper_ptr, NULL);

  LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->Stepper (*stepper_ptr);
}

/**
 * animation_glide_animation_get_stepper:
 * @animation: An #AnimationGlideAnimation
 *
 * Returns: (transfer full): Get the stepper for this #AnimationGlideAnimation
 */
AnimationStepper *
animation_glide_animation_get_stepper (AnimationGlideAnimation *animation)
{
  auto const &stepper (LookupTypedInterfaceProp <at::TransformAnimation, ag::GlideAnimation> (G_OBJECT (animation))->Stepper ());

  return animation_stepper_wrapper_new ((gpointer) &stepper);
}

static void
animation_glide_animation_set_property (GObject      *object,
                                        guint         prop_id,
                                        const GValue *value,
                                        GParamSpec   *pspec)
{
  AnimationGlideAnimation *glide_animation = ANIMATION_GLIDE_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_INITIAL_DISTANCE:
      animation_glide_animation_set_initial_distance (glide_animation, g_value_get_float (value));
      break;
    case PROP_X_ROTATION_ANGLE_DEGREES:
      animation_glide_animation_set_x_rotation_angle_degrees (glide_animation, g_value_get_float (value));
      break;
    case PROP_Y_ROTATION_ANGLE_DEGREES:
      animation_glide_animation_set_y_rotation_angle_degrees (glide_animation, g_value_get_float (value));
      break;
    case PROP_X_AXIS_LOCATION_UNIT:
      animation_glide_animation_set_x_axis_location_unit (glide_animation, g_value_get_float (value));
      break;
    case PROP_Y_AXIS_LOCATION_UNIT:
      animation_glide_animation_set_y_axis_location_unit (glide_animation, g_value_get_float (value));
      break;
    case PROP_SCREEN_WIDTH:
      /* Not writable, except on construction */
      break;
    case PROP_TARGET:
      /* Not writable, except on construction */
      break;
    case PROP_STEPPER:
      animation_glide_animation_set_stepper (glide_animation,
                                             ANIMATION_STEPPER (g_value_get_object (value)));
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
  AnimationGlideAnimation *glide_animation = ANIMATION_GLIDE_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_INITIAL_DISTANCE:
      g_value_set_float (value, animation_glide_animation_get_initial_distance (glide_animation));
      break;
    case PROP_X_ROTATION_ANGLE_DEGREES:
      g_value_set_float (value, animation_glide_animation_get_x_rotation_angle_degrees (glide_animation));
      break;
    case PROP_Y_ROTATION_ANGLE_DEGREES:
      g_value_set_float (value, animation_glide_animation_get_y_rotation_angle_degrees (glide_animation));
      break;
    case PROP_X_AXIS_LOCATION_UNIT:
      g_value_set_float (value, animation_glide_animation_get_x_axis_location_unit (glide_animation));
      break;
    case PROP_Y_AXIS_LOCATION_UNIT:
      g_value_set_float (value, animation_glide_animation_get_y_axis_location_unit (glide_animation));
      break;
    case PROP_SCREEN_WIDTH:
      /* Not readable */
      break;
    case PROP_TARGET:
      /* Not readable */
      break;
    case PROP_STEPPER:
      g_value_take_object (value, animation_glide_animation_get_stepper (glide_animation));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_glide_animation_constructor (GType                  type,
                                       unsigned int           n_construct_params,
                                       GObjectConstructParam *construct_params)
{
  replace_named_pointer_prop_in_construct_params_if_null (construct_params,
                                                          n_construct_params,
                                                          "stepper",
                                                          g_value_get_object,
                                                          g_value_set_object,
                                                          []() -> gpointer {
                                                              return animation_linear_stepper_new (300);
                                                          });


  const char * const wanted_properties[] = {
    "initial-distance",
    "x-rotation-angle-degrees",
    "y-rotation-angle-degrees",
    "x-axis-location-unit",
    "y-axis-location-unit",
    "screen-width",
    "target",
    "stepper",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    static_cast <at::TransformAnimation *> (InterfaceConstructor <ag::GlideAnimation>::construct (
      ForwardFromValueHT (properties_table, g_value_get_float, "initial-distance"),
      ForwardFromValueHT (properties_table, g_value_get_float, "x-rotation-angle-degrees"),
      ForwardFromValueHT (properties_table, g_value_get_float, "y-rotation-angle-degrees"),
      ForwardFromValueHT (properties_table, g_value_get_float, "x-axis-location-unit"),
      ForwardFromValueHT (properties_table, g_value_get_float, "y-axis-location-unit"),
      ForwardFromValueHT (properties_table, g_value_get_uint, "screen-width"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "target"),
      ForwardFromValueHT (properties_table, animation_stepper_from_gvalue, "stepper")
    ));

  replace_interface_prop_in_construct_params (construct_params,
                                              n_construct_params,
                                              g_steal_pointer (&interface));

  return G_OBJECT_CLASS (animation_glide_animation_parent_class)->constructor (type,
                                                                               n_construct_params,
                                                                               construct_params);
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
                        -1.0f,
                        1.0f,
                        -0.3f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_glide_animation_props[PROP_X_ROTATION_ANGLE_DEGREES] =
    g_param_spec_float ("x-rotation-angle-degrees",
                        "X Rotation Angle Degrees",
                        "Number of degrees on the X axis to rotate",
                        -360.0f,
                        360.0f,
                        0.0f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_glide_animation_props[PROP_Y_ROTATION_ANGLE_DEGREES] =
    g_param_spec_float ("y-rotation-angle-degrees",
                        "Y Rotation Angle Degrees",
                        "Number of degrees on the Y axis to rotate",
                        -360.0f,
                        360.0f,
                        0.0f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_glide_animation_props[PROP_X_AXIS_LOCATION_UNIT] =
    g_param_spec_float ("x-axis-location-unit",
                        "X Axis Location Unit",
                        "Unit-coordinates of where the X axis is on the surface",
                        0.0f,
                        1.0f,
                        0.2f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_glide_animation_props[PROP_Y_AXIS_LOCATION_UNIT] =
    g_param_spec_float ("y-axis-location-unit",
                        "Y Axis Location Unit",
                        "Unit-coordinates of where the Y axis is on the surface",
                        0.0f,
                        1.0f,
                        0.5f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_glide_animation_props[PROP_SCREEN_WIDTH] =
    g_param_spec_uint ("screen-width",
                       "Screen Width",
                       "Width of the screen in pixels",
                       1,
                       G_MAXUINT,
                       1,
                       static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_TARGET] =
    g_param_spec_boxed ("target",
                        "Target Box",
                        "Box that we are animating to",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_glide_animation_props[PROP_STEPPER] =
    g_param_spec_object ("stepper",
                         "Stepper",
                         "Stepper to use to progress the animation",
                         ANIMATION_TYPE_STEPPER,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

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
 * @screen_width: Width of the screen, in pixels.
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
                     unsigned int        screen_width,
                     const AnimationBox *target_box,
                     unsigned int        length)
{
  return ANIMATION_GLIDE_ANIMATION (g_object_new (ANIMATION_TYPE_GLIDE_ANIMATION,
                                                  "initial-distance", initial_distance,
                                                  "x-rotation-angle-degrees", x_rotation_angle_degrees,
                                                  "y-rotation-angle-degrees", y_rotation_angle_degrees,
                                                  "x-axis-location-unit", x_axis_location_unit,
                                                  "y-axis-location-unit", y_axis_location_unit,
                                                  "screen-width", screen_width,
                                                  "target", target_box,
                                                  "length", length,
                                                  NULL));
}
