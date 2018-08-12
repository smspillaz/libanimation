/*
 * animation-glib/bounce/bounce.cpp
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
 * GObject implementation for a "bounce" animation.
 */

#include <animation/bounce/bounce.h>
#include <animation/stepper/linear.h>

#include <animation-glib/box.h>
#include <animation-glib/bounce/bounce.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/stepper/stepper.h>
#include <animation-glib/stepper/linear.h>
#include <animation-glib/stepper/stepper-wrapper.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace ab = animation::bounce;
namespace as = animation::stepper;
namespace at = animation::transform;

struct _AnimationBounceAnimation
{
  AnimationTransformAnimation parent_instance;
};

typedef struct _AnimationBounceAnimationPrivate
{
} AnimationBounceAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationBounceAnimation,
                            animation_bounce_animation,
                            ANIMATION_TYPE_TRANSFORM_ANIMATION)

enum {
  PROP_0,
  PROP_INITIAL_SCALE,
  PROP_MAXIMUM_SCALE,
  PROP_N_BOUNCE,
  PROP_TARGET,
  PROP_STEPPER,
  NPROPS
};

static GParamSpec *animation_bounce_animation_props [NPROPS] = { NULL, };

float
animation_bounce_animation_get_initial_scale (AnimationBounceAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->InitialScale ();
}

void
animation_bounce_animation_set_initial_scale (AnimationBounceAnimation *animation,
                                              float                     initial_scale)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->InitialScale (initial_scale);
}

float
animation_bounce_animation_get_maximum_scale (AnimationBounceAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->MaximumScale ();
}

void
animation_bounce_animation_set_maximum_scale (AnimationBounceAnimation *animation,
                                              float                     maximum_scale)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->MaximumScale (maximum_scale);
}

unsigned int
animation_bounce_animation_get_n_bounce (AnimationBounceAnimation *animation)
{
  return LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->NBounce ();
}

void
animation_bounce_animation_set_n_bounce (AnimationBounceAnimation *animation,
                                         unsigned int              n_bounce)
{
  LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->NBounce (n_bounce);
}

/**
 * animation_bounce_animation_get_target:
 * @animation: An #AnimationBounceAnimation
 * @out_box: (out caller-allocates): Return location for an #AnimationBox
 *
 * Get the box representing the animation target.
 */
void
animation_bounce_animation_get_target (AnimationBounceAnimation *animation,
                                       AnimationBox             *out_box)
{
  g_return_if_fail (out_box != nullptr);

  animation::Box <animation::Point> const &box =
    LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->Target ();

  out_box->top_left.x = agd::get <0> (box.topLeft ());
  out_box->top_left.y = agd::get <1> (box.topLeft ());
  out_box->bottom_right.x = agd::get <0> (box.bottomRight ());
  out_box->bottom_right.y = agd::get <1> (box.bottomRight ());
}

void
animation_bounce_animation_set_stepper (AnimationBounceAnimation *animation,
                                        AnimationStepper         *stepper)
{
  animation::stepper::Stepper *stepper_ptr = nullptr;
  g_object_get (stepper, "stepper", (gpointer) &stepper_ptr, NULL);

  LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->Stepper (*stepper_ptr);
}

/**
 * animation_bounce_animation_get_stepper:
 * @animation: An #AnimationBounceAnimation
 *
 * Returns: (transfer full): Get the stepper for this #AnimationBounceAnimation
 */
AnimationStepper *
animation_bounce_animation_get_stepper (AnimationBounceAnimation *animation)
{
  auto const &stepper (LookupTypedInterfaceProp <at::TransformAnimation, ab::BounceAnimation> (G_OBJECT (animation))->Stepper ());

  return animation_stepper_wrapper_new ((gpointer) &stepper);
}

static void
animation_bounce_animation_set_property (GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec)
{
  switch (prop_id)
    {
    case PROP_INITIAL_SCALE:
      animation_bounce_animation_set_initial_scale (ANIMATION_BOUNCE_ANIMATION (object),
                                                    g_value_get_float (value));
      break;
    case PROP_MAXIMUM_SCALE:
      animation_bounce_animation_set_maximum_scale (ANIMATION_BOUNCE_ANIMATION (object),
                                                    g_value_get_float (value));
      break;
    case PROP_N_BOUNCE:
      animation_bounce_animation_set_n_bounce (ANIMATION_BOUNCE_ANIMATION (object),
                                               g_value_get_uint (value));
      break;
    case PROP_TARGET:
      /* No-op here to handle the constructor */
      break;
    case PROP_STEPPER:
      animation_bounce_animation_set_stepper (ANIMATION_BOUNCE_ANIMATION (object),
                                              ANIMATION_STEPPER (g_value_get_object (value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_bounce_animation_get_property (GObject    *object,
                                         guint       prop_id,
                                         GValue     *value,
                                         GParamSpec *pspec)
{
  AnimationBounceAnimation *bounce_animation = ANIMATION_BOUNCE_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_INITIAL_SCALE:
      g_value_set_float (value, animation_bounce_animation_get_initial_scale (bounce_animation));
      break;
    case PROP_MAXIMUM_SCALE:
      g_value_set_float (value, animation_bounce_animation_get_maximum_scale (bounce_animation));
      break;
    case PROP_N_BOUNCE:
      g_value_set_uint (value, animation_bounce_animation_get_n_bounce (bounce_animation));
      break;
    case PROP_TARGET:
      {
        AnimationBox target;
        animation_bounce_animation_get_target (bounce_animation, &target);

        g_value_set_boxed (value, &target);
        break;
      }
    case PROP_STEPPER:
      g_value_take_object (value, animation_bounce_animation_get_stepper (bounce_animation));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_bounce_animation_constructor (GType                  type,
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
    "initial-scale",
    "maximum-scale",
    "n-bounce",
    "target",
    "stepper",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    static_cast <at::TransformAnimation *> (InterfaceConstructor <ab::BounceAnimation>::construct (
      ForwardFromValueHT (properties_table, g_value_get_float, "initial-scale"),
      ForwardFromValueHT (properties_table, g_value_get_float, "maximum-scale"),
      ForwardFromValueHT (properties_table, g_value_get_uint, "n-bounce"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "target"),
      ForwardFromValueHT (properties_table, animation_stepper_from_gvalue, "stepper")
    ));

  replace_interface_prop_in_construct_params (construct_params,
                                              n_construct_params,
                                              g_steal_pointer (&interface));

  return G_OBJECT_CLASS (animation_bounce_animation_parent_class)->constructor (type,
                                                                                n_construct_params,
                                                                                construct_params);
}

static void
animation_bounce_animation_init (AnimationBounceAnimation *model)
{
}


static void
animation_bounce_animation_class_init (AnimationBounceAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_bounce_animation_constructor;
  object_class->get_property = animation_bounce_animation_get_property;
  object_class->set_property = animation_bounce_animation_set_property;

  animation_bounce_animation_props[PROP_INITIAL_SCALE] =
    g_param_spec_float ("initial-scale",
                        "Initial Scale",
                        "The initial scale of the animation",
                        0.1f,
                        1.0f,
                        0.7f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE |
                                                   G_PARAM_CONSTRUCT));

  animation_bounce_animation_props[PROP_MAXIMUM_SCALE] =
    g_param_spec_float ("maximum-scale",
                        "Maximum Scale",
                        "The maximum scale of the animation",
                        1.0f,
                        3.0f,
                        1.2f,
                        static_cast <GParamFlags> (G_PARAM_READWRITE |
                                                   G_PARAM_CONSTRUCT));

  animation_bounce_animation_props[PROP_N_BOUNCE] =
    g_param_spec_uint ("n-bounce",
                       "Number of Bounces",
                       "The number of bounces in the animation",
                       1,
                       10,
                       1,
                       static_cast <GParamFlags> (G_PARAM_READWRITE |
                                                  G_PARAM_CONSTRUCT));

  animation_bounce_animation_props[PROP_TARGET] =
    g_param_spec_boxed ("target",
                        "Target Box",
                        "Box that we are animating to",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_READWRITE |
                                                   G_PARAM_CONSTRUCT_ONLY));

  animation_bounce_animation_props[PROP_STEPPER] =
    g_param_spec_object ("stepper",
                         "Stepper",
                         "Stepper to use to progress the animation",
                         ANIMATION_TYPE_STEPPER,
                         static_cast <GParamFlags> (G_PARAM_READWRITE |
                                                    G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_bounce_animation_props);
}

/**
 * animation_bounce_new:
 * @initial_scale: Scale factor that the surface will initially have.
 * @maximum_scale: Scale factor that the surface will have at maximum.
 * @n_bounce: Number of bounces.
 * @target: The #AnimationBox that we are animating to.
 * @stepper: The #AnimationStepper of the animation.
 *
 * Returns: (transfer full): A new #AnimationBounceAnimation.
 */
AnimationBounceAnimation *
animation_bounce_new (float               initial_scale,
                      float               maximum_scale,
                      unsigned int        n_bounce,
                      const AnimationBox *target,
                      AnimationStepper   *stepper)
{
  return ANIMATION_BOUNCE_ANIMATION (g_object_new (ANIMATION_TYPE_BOUNCE_ANIMATION,
                                                   "initial-scale", initial_scale,
                                                   "maximum-scale", maximum_scale,
                                                   "n-bounce", n_bounce,
                                                   "target", target,
                                                   "stepper", stepper,
                                                   NULL));
}
