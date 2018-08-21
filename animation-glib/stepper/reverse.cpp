/*
 * animation-glib/stepper/reverse.cpp
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
 * GObject wrapper for reverse stepper.
 */

#include <animation/stepper/reverse.h>
#include <animation/stepper/stepper.h>

#include <animation-glib/constructor-helpers.h>
#include <animation-glib/stepper/reverse.h>
#include <animation-glib/stepper/stepper-holder.h>

namespace as = animation::stepper;

struct _AnimationReverseStepper
{
  GObject parent_instance;
};

typedef struct _AnimationReverseStepperPrivate
{
  AnimationStepper *base_stepper;
} AnimationReverseStepperPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationReverseStepper,
                            animation_reverse_stepper,
                            ANIMATION_TYPE_STEPPER_HOLDER)

enum {
  PROP_0,
  PROP_BASE_STEPPER,
  NPROPS
};

static GParamSpec *animation_reverse_stepper_properties[NPROPS];

static void
animation_reverse_stepper_set_property (GObject      *object,
                                        guint         prop_id,
                                        const GValue *value,
                                        GParamSpec   *pspec)
{
  AnimationReverseStepper *stepper = ANIMATION_REVERSE_STEPPER (object);
  AnimationReverseStepperPrivate *priv =
    reinterpret_cast <AnimationReverseStepperPrivate *> (animation_reverse_stepper_get_instance_private (stepper));

  switch (prop_id)
    {
    case PROP_BASE_STEPPER:
      priv->base_stepper = ANIMATION_STEPPER (g_value_dup_object (value));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_reverse_stepper_get_property (GObject    *object,
                                        guint       prop_id,
                                        GValue     *value,
                                        GParamSpec *pspec)
{
  AnimationReverseStepper *stepper = ANIMATION_REVERSE_STEPPER (object);
  AnimationReverseStepperPrivate *priv =
    reinterpret_cast <AnimationReverseStepperPrivate *> (animation_reverse_stepper_get_instance_private (stepper));

  switch (prop_id)
    {
    case PROP_BASE_STEPPER:
      g_value_set_object (value, G_OBJECT (priv->base_stepper));
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_reverse_stepper_dispose (GObject *object)
{
  AnimationReverseStepper *stepper = ANIMATION_REVERSE_STEPPER (object);
  AnimationReverseStepperPrivate *priv =
    reinterpret_cast <AnimationReverseStepperPrivate *> (animation_reverse_stepper_get_instance_private (stepper));

  g_clear_object (&priv->base_stepper);
}

static GObject *
animation_reverse_stepper_constructor (GType                  type,
                                       unsigned int           n_construct_params,
                                       GObjectConstructParam *construct_params)
{
  const char * const wanted_properties[] = {
    "base-stepper",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto stepper = as::Reverse (ForwardFromValueHT (properties_table,
                                                  animation_stepper_from_gvalue,
                                                  "base-stepper"));

  replace_named_pointer_prop_in_construct_params (construct_params,
                                                  n_construct_params,
                                                  "stepper",
                                                  &stepper);

  return G_OBJECT_CLASS (animation_reverse_stepper_parent_class)->constructor (type,
                                                                               n_construct_params,
                                                                               construct_params);
}

static void
animation_reverse_stepper_init (AnimationReverseStepper *model)
{
}

static void
animation_reverse_stepper_class_init (AnimationReverseStepperClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_reverse_stepper_constructor;
  object_class->get_property = animation_reverse_stepper_get_property;
  object_class->set_property = animation_reverse_stepper_set_property;
  object_class->dispose = animation_reverse_stepper_dispose;

  animation_reverse_stepper_properties[PROP_BASE_STEPPER] =
    g_param_spec_object ("base-stepper",
                         "Base Stepper",
                         "Stepper to reverse",
                         ANIMATION_TYPE_STEPPER,
                         static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_reverse_stepper_properties);
}

/**
 * animation_reverse_stepper_new:
 * @base_stepper: The stepper to reverse.
 *
 * Return a new #AnimationStepper which runs @base_stepper in reverse.
 *
 * Returns: (transfer full): An #AnimationReverseStepper
 *                           implementation of #AnimationStepper
 */
AnimationStepper *
animation_reverse_stepper_new (AnimationStepper *base_stepper)
{
  return ANIMATION_STEPPER (g_object_new (ANIMATION_TYPE_REVERSE_STEPPER,
                                          "base-stepper", base_stepper,
                                          NULL));
}
