/*
 * animation-glib/stepper/linear.cpp
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
 * GObject wrapper for linear stepper.
 */

#include <animation/stepper/linear.h>
#include <animation/stepper/stepper.h>

#include <animation-glib/constructor-helpers.h>
#include <animation-glib/stepper/linear.h>
#include <animation-glib/stepper/stepper-holder.h>

namespace as = animation::stepper;

struct _AnimationLinearStepper
{
  GObject parent_instance;
};

typedef struct _AnimationLinearStepperPrivate
{
} AnimationLinearStepperPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationLinearStepper,
                            animation_linear_stepper,
                            ANIMATION_TYPE_STEPPER_HOLDER)

enum {
  PROP_0,
  PROP_LENGTH,
  NPROPS
};

static GParamSpec *animation_linear_stepper_properties[NPROPS];

static void
animation_linear_stepper_set_property (GObject      *object,
                                       guint         prop_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
  switch (prop_id)
    {
    case PROP_LENGTH:
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_linear_stepper_get_property (GObject    *object,
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
animation_linear_stepper_constructor (GType                  type,
                                      unsigned int           n_construct_params,
                                      GObjectConstructParam *construct_params)
{
  const char * const wanted_properties[] = {
    "length",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto stepper = as::Linear (ForwardFromValueHT (properties_table, g_value_get_uint, "length"));

  replace_named_pointer_prop_in_construct_params (construct_params,
                                                  n_construct_params,
                                                  "stepper",
                                                  &stepper);

  return G_OBJECT_CLASS (animation_linear_stepper_parent_class)->constructor (type,
                                                                              n_construct_params,
                                                                              construct_params);
}

static void
animation_linear_stepper_init (AnimationLinearStepper *model)
{
}

static void
animation_linear_stepper_class_init (AnimationLinearStepperClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_linear_stepper_constructor;
  object_class->get_property = animation_linear_stepper_get_property;
  object_class->set_property = animation_linear_stepper_set_property;

  animation_linear_stepper_properties[PROP_LENGTH] =
    g_param_spec_uint ("length",
                       "Length",
                       "How long the animation lasts",
                       1,
                       5000,
                       300,
                       static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT_ONLY));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_linear_stepper_properties);
}

/**
 * animation_linear_stepper_new:
 * @length: Length of the transition in milliseconds.
 *
 * Return a new #AnimationStepper which linearly increments progress
 * every time the step() method is called on it.
 *
 * Returns: (transfer full): An #AnimationLinearStepper
 *                           implementation of #AnimationStepper
 */
AnimationStepper *
animation_linear_stepper_new (unsigned int length)
{
  return ANIMATION_STEPPER (g_object_new (ANIMATION_TYPE_LINEAR_STEPPER,
                                          "length", length,
                                          NULL));
}
