/*
 * animation-glib/zoom/zoom.cpp
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
 * GObject interface for a zoom animation.
 */

#include <animation/zoom/zoom.h>

#include <animation-glib/box.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/stepper/linear.h>
#include <animation-glib/stepper/stepper-wrapper.h>
#include <animation-glib/vector.h>
#include <animation-glib/zoom/zoom.h>

namespace agd = animation::geometry::dimension;
namespace at = animation::transform;
namespace az = animation::zoom;

struct _AnimationZoomAnimation
{
  AnimationTransformAnimation parent_instance;
};

typedef struct _AnimationZoomAnimationPrivate
{
} AnimationZoomAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationZoomAnimation,
                            animation_zoom_animation,
                            ANIMATION_TYPE_TRANSFORM_ANIMATION)

enum {
  PROP_0,
  PROP_FROM,
  PROP_TO,
  PROP_STEPPER,
  NPROPS
};

static GParamSpec *animation_zoom_animation_props [NPROPS] = { NULL, };

void
animation_zoom_animation_set_from (AnimationZoomAnimation *animation,
                                   AnimationBox            box)
{
  LookupTypedInterfaceProp <at::TransformAnimation, az::ZoomAnimation> (G_OBJECT (animation))->From (animation::Box <animation::Point> (animation::Point (box.top_left.x,
                                                      box.top_left.y),
                                    animation::Point (box.bottom_right.x,
                                                      box.bottom_right.y)));
}

/**
 * animation_glide_animation_get_from:
 * @animation: An #AnimationZoomAnimation
 * @out_box: (out caller-allocates): Return location for an #AnimationBox to the source
 *
 * Get the source box for this animation.
 */
void
animation_zoom_animation_get_from (AnimationZoomAnimation *animation,
                                   AnimationBox           *out_box)
{
  g_return_if_fail (out_box != NULL);

  auto box = LookupTypedInterfaceProp <at::TransformAnimation, az::ZoomAnimation> (G_OBJECT (animation))->From ();

  out_box->top_left.x = agd::get <0> (box.topLeft ());
  out_box->top_left.y = agd::get <1> (box.topLeft ());
  out_box->bottom_right.x = agd::get <0> (box.bottomRight ());
  out_box->bottom_right.y = agd::get <1> (box.bottomRight ());
}


/**
 * animation_zoom_animation_get_to:
 * @animation: An #AnimationZoomAnimation
 * @out_box: (out caller-allocates): Return location for an #AnimationBox to the target
 *
 * Get the source box for this animation.
 */
void
animation_zoom_animation_get_to (AnimationZoomAnimation *animation,
                                 AnimationBox             *out_box)
{
  g_return_if_fail (out_box != NULL);

  auto box = LookupTypedInterfaceProp <at::TransformAnimation, az::ZoomAnimation> (G_OBJECT (animation))->To ();

  out_box->top_left.x = agd::get <0> (box.topLeft ());
  out_box->top_left.y = agd::get <1> (box.topLeft ());
  out_box->bottom_right.x = agd::get <0> (box.bottomRight ());
  out_box->bottom_right.y = agd::get <1> (box.bottomRight ());
}

void
animation_zoom_animation_set_stepper (AnimationZoomAnimation *animation,
                                      AnimationStepper       *stepper)
{
  animation::stepper::Stepper *stepper_ptr = nullptr;
  g_object_get (stepper, "stepper", (gpointer) &stepper_ptr, NULL);

  LookupTypedInterfaceProp <at::TransformAnimation, az::ZoomAnimation> (G_OBJECT (animation))->Stepper (*stepper_ptr);
}

/**
 * animation_zoom_animation_get_stepper:
 * @animation: An #AnimationZoomAnimation
 *
 * Returns: (transfer full): Get the stepper for this #AnimationZoomAnimation
 */
AnimationStepper *
animation_zoom_animation_get_stepper (AnimationZoomAnimation *animation)
{
  auto const &stepper (LookupTypedInterfaceProp <at::TransformAnimation, az::ZoomAnimation> (G_OBJECT (animation))->Stepper ());

  return animation_stepper_wrapper_new ((gpointer) &stepper);
}

static void
animation_zoom_animation_set_property (GObject      *object,
                                       guint         prop_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
  AnimationZoomAnimation *zoom_animation = ANIMATION_ZOOM_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_FROM:
      {
        AnimationBox *box = reinterpret_cast <AnimationBox *> (g_value_get_boxed (value));

        if (box != nullptr)
          animation_zoom_animation_set_from (zoom_animation, *box);
      }
      break;
    case PROP_TO:
      break;
    case PROP_STEPPER:
      animation_zoom_animation_set_stepper (zoom_animation, reinterpret_cast <AnimationStepper *> (g_value_get_object (value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_zoom_animation_get_property (GObject    *object,
                                       guint       prop_id,
                                       GValue     *value,
                                       GParamSpec *pspec)
{
  AnimationZoomAnimation *zoom_animation = ANIMATION_ZOOM_ANIMATION (object);

  switch (prop_id)
    {
    case PROP_FROM:
      {
        AnimationBox box;
        animation_zoom_animation_get_from (zoom_animation, &box);

        g_value_set_boxed (value, (gpointer) &box);
      }
      break;
    case PROP_TO:
      {
        AnimationBox box;
        animation_zoom_animation_get_to (zoom_animation, &box);

        g_value_set_boxed (value, (gpointer) &box);
      }
      break;
    case PROP_STEPPER:
      g_value_take_object (value, animation_zoom_animation_get_stepper (zoom_animation));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_zoom_animation_constructor (GType                  type,
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
    "from",
    "to",
    "stepper",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    static_cast <at::TransformAnimation *> (InterfaceConstructor <az::ZoomAnimation>::construct (
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "from"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "to"),
      ForwardFromValueHT (properties_table, animation_stepper_from_gvalue, "stepper")
    ));

  replace_interface_prop_in_construct_params (construct_params,
                                              n_construct_params,
                                              g_steal_pointer (&interface));

  return G_OBJECT_CLASS (animation_zoom_animation_parent_class)->constructor (type,
                                                                              n_construct_params,
                                                                              construct_params);
}

static void
animation_zoom_animation_init (AnimationZoomAnimation *model)
{
}

static void
animation_zoom_animation_class_init (AnimationZoomAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_zoom_animation_constructor;
  object_class->get_property = animation_zoom_animation_get_property;
  object_class->set_property = animation_zoom_animation_set_property;

  animation_zoom_animation_props[PROP_FROM] =
    g_param_spec_boxed ("from",
                        "From Box",
                        "Box that we are animating from",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  animation_zoom_animation_props[PROP_TO] =
    g_param_spec_boxed ("to",
                        "To Box",
                        "Box that we are animating to",
                        ANIMATION_TYPE_BOX,
                        static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY));
  animation_zoom_animation_props[PROP_STEPPER] =
    g_param_spec_object ("stepper",
                         "Stepper",
                         "Stepper to use to progress the animation",
                         ANIMATION_TYPE_STEPPER,
                         static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_zoom_animation_props);
}

/**
 * animation_zoom_new:
 * @from: The #AnimationBox that we are animating from.
 * @to: The #AnimationBox that we are animating to (current location).
 * @stepper: The stepper to use.
 *
 * Returns: (transfer full): A new #AnimationZoomAnimation.
 */
AnimationZoomAnimation *
animation_zoom_new (const AnimationBox *current,
                    const AnimationBox *from,
                    const AnimationBox *to,
                    AnimationStepper   *stepper)
{
  return ANIMATION_ZOOM_ANIMATION (g_object_new (ANIMATION_TYPE_ZOOM_ANIMATION,
                                                 "from", from,
                                                 "to", to,
                                                 "stepper", stepper,
                                                 NULL));
}
