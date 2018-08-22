/*
 * animation-glib/magiclamp/magiclamp.cpp
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
 * GObject implementation for a "magiclamp" animation.
 */

#include <animation/magiclamp/magiclamp.h>

#include <animation-glib/box.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/grid/grid.h>
#include <animation-glib/magiclamp/magiclamp.h>
#include <animation-glib/stepper/linear.h>
#include <animation-glib/stepper/stepper.h>
#include <animation-glib/stepper/stepper-wrapper.h>
#include <animation-glib/vector.h>

namespace ag = animation::grid;
namespace agd = animation::geometry::dimension;
namespace aml = animation::magiclamp;

struct _AnimationMagicLampAnimation
{
  AnimationGridAnimation parent_instance;
};

typedef struct _AnimationMagicLampAnimationPrivate
{
} AnimationMagicLampAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationMagicLampAnimation,
                            animation_magiclamp_animation,
                            ANIMATION_TYPE_GRID_ANIMATION)

enum {
  PROP_0,
  PROP_SOURCE,
  PROP_TARGET,
  PROP_RESOLUTION,
  PROP_BEND_FACTOR,
  PROP_OFFSET_FACTOR,
  PROP_STRETCH_FACTOR,
  PROP_DEFORM_SPEED_FACTOR,
  PROP_STEPPER,
  NPROPS
};

static GParamSpec *animation_magiclamp_animation_props [NPROPS] = { NULL, };

void
animation_magiclamp_animation_set_source (AnimationMagicLampAnimation *animation,
                                          AnimationBox            box)
{
  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->Source (animation::Box <animation::Point> (animation::Point (box.top_left.x,
                                                                                                                                                             box.top_left.y),
                                                                                                                                           animation::Point (box.bottom_right.x,
                                                                                                                                                             box.bottom_right.y)));
}

/**
 * animation_magiclamp_animation_get_source:
 * @animation: An #AnimationMagicLampAnimation
 * @out_box: (out caller-allocates): Return location for an #AnimationBox to the source
 *
 * Get the source box for this animation.
 */
void
animation_magiclamp_animation_get_source (AnimationMagicLampAnimation *animation,
                                          AnimationBox                *out_box)
{
  g_return_if_fail (out_box != NULL);

  auto box = LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->Source ();

  out_box->top_left.x = agd::get <0> (box.topLeft ());
  out_box->top_left.y = agd::get <1> (box.topLeft ());
  out_box->bottom_right.x = agd::get <0> (box.bottomRight ());
  out_box->bottom_right.y = agd::get <1> (box.bottomRight ());
}


/**
 * animation_magiclamp_animation_get_target:
 * @animation: An #AnimationMagicLampAnimation
 * @out_box: (out caller-allocates): Return location for an #AnimationBox to the target
 *
 * Get the source box for this animation.
 */
void
animation_magiclamp_animation_get_target (AnimationMagicLampAnimation *animation,
                                          AnimationBox                *out_box)
{
  g_return_if_fail (out_box != NULL);

  auto box = LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->Target ();

  out_box->top_left.x = agd::get <0> (box.topLeft ());
  out_box->top_left.y = agd::get <1> (box.topLeft ());
  out_box->bottom_right.x = agd::get <0> (box.bottomRight ());
  out_box->bottom_right.y = agd::get <1> (box.bottomRight ());
}

void
animation_magiclamp_animation_set_bend_factor (AnimationMagicLampAnimation *animation,
                                               float                        bend_factor)
{
  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->BendFactor (bend_factor);
}

float
animation_magiclamp_animation_get_bend_factor (AnimationMagicLampAnimation *animation)
{
  return LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->BendFactor ();
}

void
animation_magiclamp_animation_set_offset_factor (AnimationMagicLampAnimation *animation,
                                                 float                        offset_factor)
{
  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->OffsetFactor (offset_factor);
}

float
animation_magiclamp_animation_get_offset_factor (AnimationMagicLampAnimation *animation)
{
  return LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->OffsetFactor ();
}

void
animation_magiclamp_animation_set_stretch_factor (AnimationMagicLampAnimation *animation,
                                                  float                        stretch_factor)
{
  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->StretchFactor (stretch_factor);
}

float
animation_magiclamp_animation_get_stretch_factor (AnimationMagicLampAnimation *animation)
{
  return LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->StretchFactor ();
}

void
animation_magiclamp_animation_set_deform_speed_factor (AnimationMagicLampAnimation *animation,
                                                       float                        deform_speed_factor)
{
  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->DeformSpeedFactor (deform_speed_factor);
}

float
animation_magiclamp_animation_get_deform_speed_factor (AnimationMagicLampAnimation *animation)
{
  return LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->DeformSpeedFactor ();
}

void
animation_magiclamp_animation_set_stepper (AnimationMagicLampAnimation *animation,
                                           AnimationStepper            *stepper)
{
  animation::stepper::Stepper *stepper_ptr = nullptr;
  g_object_get (stepper, "stepper", (gpointer) &stepper_ptr, NULL);

  LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->Stepper (*stepper_ptr);
}

/**
 * animation_magiclamp_animation_get_stepper:
 * @animation: An #AnimationMagicLampAnimation
 *
 * Returns: (transfer full): Get the stepper for this #AnimationMagicLampAnimation
 */
AnimationStepper *
animation_magiclamp_animation_get_stepper (AnimationMagicLampAnimation *animation)
{
  auto const &stepper (LookupTypedInterfaceProp <ag::GridAnimation, aml::MagicLampAnimation> (G_OBJECT (animation))->Stepper ());

  return animation_stepper_wrapper_new ((gpointer) &stepper);
}

static void
animation_magiclamp_animation_set_property (GObject      *object,
                                            guint         prop_id,
                                            const GValue *value,
                                            GParamSpec   *pspec)
{
  AnimationMagicLampAnimation *magiclamp_animation = ANIMATION_MAGIC_LAMP_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_SOURCE:
      {
        AnimationBox *box = reinterpret_cast <AnimationBox *> (g_value_get_boxed (value));

        if (box != nullptr)
          animation_magiclamp_animation_set_source (magiclamp_animation, *box);
      }
      break;
    case PROP_TARGET:
      /* Not writable, except on construction */
      break;
    case PROP_RESOLUTION:
      /* Not writable, except on construction */
      break;
    case PROP_BEND_FACTOR:
      animation_magiclamp_animation_set_bend_factor (magiclamp_animation, g_value_get_float (value));
      break;
    case PROP_OFFSET_FACTOR:
      animation_magiclamp_animation_set_offset_factor (magiclamp_animation, g_value_get_float (value));
      break;
    case PROP_STRETCH_FACTOR:
      animation_magiclamp_animation_set_stretch_factor (magiclamp_animation, g_value_get_float (value));
      break;
    case PROP_DEFORM_SPEED_FACTOR:
      animation_magiclamp_animation_set_deform_speed_factor (magiclamp_animation, g_value_get_float (value));
      break;
    case PROP_STEPPER:
      animation_magiclamp_animation_set_stepper (magiclamp_animation,
                                                 ANIMATION_STEPPER (g_value_get_object (value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_magiclamp_animation_get_property (GObject    *object,
                                            guint       prop_id,
                                            GValue     *value,
                                            GParamSpec *pspec)
{
  AnimationMagicLampAnimation *magiclamp_animation = ANIMATION_MAGIC_LAMP_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_SOURCE:
      {
        AnimationBox box;

        animation_magiclamp_animation_get_source (magiclamp_animation, &box);
        g_value_set_boxed (value, (gpointer) &box);
      }
      break;
    case PROP_TARGET:
      {
        AnimationBox box;

        animation_magiclamp_animation_get_target (magiclamp_animation, &box);
        g_value_set_boxed (value, (gpointer) &box);
      }
      break;
    case PROP_BEND_FACTOR:
      g_value_set_float (value, animation_magiclamp_animation_get_bend_factor (magiclamp_animation));
      break;
    case PROP_STRETCH_FACTOR:
      g_value_set_float (value, animation_magiclamp_animation_get_stretch_factor (magiclamp_animation));
      break;
    case PROP_OFFSET_FACTOR:
      g_value_set_float (value, animation_magiclamp_animation_get_offset_factor (magiclamp_animation));
      break;
    case PROP_DEFORM_SPEED_FACTOR:
      g_value_set_float (value, animation_magiclamp_animation_get_deform_speed_factor (magiclamp_animation));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_magiclamp_animation_constructor (GType                  type,
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

  /* Need to have a grid resolution */
  replace_named_pointer_prop_in_construct_params_if_null (construct_params,
                                                          n_construct_params,
                                                          "resolution",
                                                          g_value_get_boxed,
                                                          (AnimationConstructorHelpersGValueSetPointerFunc) g_value_set_boxed,
                                                          []() -> gpointer {
                                                              AnimationVector *v = g_new0 (AnimationVector, 1);

                                                              v->x = 10.0;
                                                              v->y = 10.0;

                                                              return (gpointer) v;
                                                          });

  const char * const wanted_properties[] = {
    "source",
    "target",
    "resolution",
    "bend-factor",
    "offset-factor",
    "stretch-factor",
    "deform-speed-factor",
    "stepper",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    static_cast <ag::GridAnimation *> (InterfaceConstructor <aml::MagicLampAnimation>::construct (
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "source"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "target"),
      ForwardFromValueHT (properties_table, animation_point_size_t_from_gvalue, "resolution"),
      ForwardFromValueHT (properties_table, g_value_get_float, "bend-factor"),
      ForwardFromValueHT (properties_table, g_value_get_float, "offset-factor"),
      ForwardFromValueHT (properties_table, g_value_get_float, "stretch-factor"),
      ForwardFromValueHT (properties_table, g_value_get_float, "deform-speed-factor"),
      ForwardFromValueHT (properties_table, animation_stepper_from_gvalue, "stepper")
    ));

  replace_interface_prop_in_construct_params (construct_params,
                                              n_construct_params,
                                              g_steal_pointer (&interface));

  return G_OBJECT_CLASS (animation_magiclamp_animation_parent_class)->constructor (type,
                                                                                   n_construct_params,
                                                                                   construct_params);
}

static void
animation_magiclamp_animation_init (AnimationMagicLampAnimation *model)
{
}


static void
animation_magiclamp_animation_class_init (AnimationMagicLampAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_magiclamp_animation_constructor;
  object_class->get_property = animation_magiclamp_animation_get_property;
  object_class->set_property = animation_magiclamp_animation_set_property;

  animation_magiclamp_animation_props[PROP_SOURCE] =
    g_param_spec_boxed ("source",
                        "Source Box",
                        "Box that we are animating from",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_magiclamp_animation_props[PROP_TARGET] =
    g_param_spec_boxed ("target",
                        "Target Box",
                        "Box that we are animating to",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY));

  animation_magiclamp_animation_props[PROP_RESOLUTION] =
    g_param_spec_boxed ("resolution",
                        "Resolution",
                        "Grid Resolution",
                        ANIMATION_TYPE_VECTOR,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_magiclamp_animation_props[PROP_BEND_FACTOR] =
    g_param_spec_float ("bend-factor",
                        "Bend Factor",
                        "How much the window should bend",
                        1.0,
                        20.0,
                        10.0,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_magiclamp_animation_props[PROP_OFFSET_FACTOR] =
    g_param_spec_float ("offset-factor",
                        "Offset Factor",
                        "How big the curves of the animation should be",
                        0.1,
                        1.0,
                        0.5,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_magiclamp_animation_props[PROP_STRETCH_FACTOR] =
    g_param_spec_float ("stretch-factor",
                        "Stretch Factor",
                        "How much the window should stretch when animating",
                        0.2,
                        1.0,
                        0.45,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_magiclamp_animation_props[PROP_DEFORM_SPEED_FACTOR] =
    g_param_spec_float ("deform-speed-factor",
                        "Deform Speed Factor",
                        "How quickly the deformation phase should happen",
                        1.0,
                        4.0,
                        2.3,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_magiclamp_animation_props[PROP_STEPPER] =
    g_param_spec_object ("stepper",
                         "Stepper",
                         "Stepper to use to progress the animation",
                         ANIMATION_TYPE_STEPPER,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_magiclamp_animation_props);
}

/**
 * animation_magiclamp_new:
 * @source_box: The #AnimationBox that we are animating from.
 * @target_box: The #AnimationBox that we are animating to.
 * @resolution: The #AnimationVector representing the grid resolution
 * @bend_factor: How much the window should bend
 * @offset_factor: How big the curves of the animation should be
 * @deform_speed_factor: How quickly the deformation should complete.
 * @stepper: An #AnimationStepper used for progressing the animation.
 *
 * Returns: (transfer full): A new #AnimationMagicLampAnimation.
 */
AnimationMagicLampAnimation *
animation_magiclamp_new (const AnimationBox    *source_box,
                         const AnimationBox    *target_box,
                         const AnimationVector *resolution,
                         float                  bend_factor,
                         float                  offset_factor,
                         float                  stretch_factor,
                         float                  deform_speed_factor,
                         AnimationStepper      *stepper)
{
  return ANIMATION_MAGIC_LAMP_ANIMATION (g_object_new (ANIMATION_TYPE_MAGIC_LAMP_ANIMATION,
                                                       "source", source_box,
                                                       "target", target_box,
                                                       "resolution", resolution,
                                                       "stepper", stepper,
                                                       NULL));
}
