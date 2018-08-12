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

#include <animation-glib/box.h>
#include <animation-glib/bounce/bounce.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace ab = animation::bounce;

struct _AnimationBounceAnimation
{
  GObject parent_instance;
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
  PROP_LENGTH,
  NPROPS
};

static GParamSpec *animation_bounce_animation_props [NPROPS] = { NULL, };

static void
animation_bounce_animation_set_property (GObject      *object,
                                         guint         prop_id,
                                         const GValue *value,
                                         GParamSpec   *pspec)
{
  switch (prop_id)
    {
    case PROP_INITIAL_SCALE:
    case PROP_MAXIMUM_SCALE:
    case PROP_N_BOUNCE:
    case PROP_TARGET:
    case PROP_LENGTH:
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
  switch (prop_id)
    {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_bounce_animation_constructor (GType                  type,
                                        unsigned int           n_construct_params,
                                        GObjectConstructParam *construct_params)
{
  const char * const wanted_properties[] = {
    "initial-scale",
    "maximum-scale",
    "n-bounce",
    "target",
    "length",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    InterfaceConstructor <ab::BounceAnimation>::construct (
      ForwardFromValueHT (properties_table, g_value_get_float, "initial-scale"),
      ForwardFromValueHT (properties_table, g_value_get_float, "maximum-scale"),
      ForwardFromValueHT (properties_table, g_value_get_float, "n-bounce"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "target"),
      ForwardFromValueHT (properties_table, g_value_get_uint, "length")
    );

  unsigned int n_extended_construct_params;
  GObjectConstructParam *extended_construct_params =
    append_interface_prop_to_construct_params (construct_params,
                                               n_construct_params,
                                               G_OBJECT_CLASS (animation_bounce_animation_parent_class),
                                               g_steal_pointer (&interface),
                                               &n_extended_construct_params);

  return G_OBJECT_CLASS (animation_bounce_animation_parent_class)->constructor (type,
                                                                              n_extended_construct_params,
                                                                              extended_construct_params);
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
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_bounce_animation_props[PROP_MAXIMUM_SCALE] =
    g_param_spec_float ("maximum-scale",
                        "Maximum Scale",
                        "The maximum scale of the animation",
                        1.0f,
                        3.0f,
                        1.2f,
                        static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_bounce_animation_props[PROP_N_BOUNCE] =
    g_param_spec_uint ("n-bounce",
                       "Number of Bounces",
                       "The number of bounces in the animation",
                       1,
                       10,
                       1,
                       static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_bounce_animation_props[PROP_TARGET] =
    g_param_spec_pointer ("target",
                          "Target Box",
                          "Box that we are animating to",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  animation_bounce_animation_props[PROP_LENGTH] =
    g_param_spec_uint ("length",
                       "Length",
                       "How long the animation lasts",
                       1,
                       5000,
                       300,
                       static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

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
 * @length: The length of the animation.
 *
 * Returns: (transfer full): A new #AnimationBounceAnimation.
 */
AnimationBounceAnimation *
animation_bounce_new (float              initial_scale,
                      float              maximum_scale,
                      unsigned int       n_bounce,
                      const AnimationBox *target,
                      unsigned int       length)
{
  return ANIMATION_BOUNCE_ANIMATION (g_object_new (ANIMATION_TYPE_BOUNCE_ANIMATION,
                                                   "initial-scale", initial_scale,
                                                   "maximum-scale", maximum_scale,
                                                   "n-bounce", n_bounce,
                                                   "target", target,
                                                   "length", length,
                                                   NULL));
}
