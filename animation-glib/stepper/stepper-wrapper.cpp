/*
 * animation-glib/stepper/stepper-wrapper.cpp
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
 * GObject base class for steppers. Only observes the stepper passed
 * to it in its construct parameters - not expected to outlive the stepper.
 */

#include <animation/stepper/stepper.h>
#include <animation/stepper/linear.h>

#include <animation-glib/stepper/stepper.h>
#include <animation-glib/stepper/stepper-wrapper.h>

namespace as = animation::stepper;

struct _AnimationStepperWrapper
{
  GObject parent_instance;
};

typedef struct _AnimationStepperWrapperPrivate
{
  as::Stepper *stepper;
} AnimationStepperWrapperPrivate;

static void animation_stepper_iface_init (AnimationStepperInterface *stepper_iface);

G_DEFINE_TYPE_WITH_CODE (AnimationStepperWrapper,
                         animation_stepper_wrapper,
                         G_TYPE_OBJECT,
                         G_IMPLEMENT_INTERFACE (ANIMATION_TYPE_STEPPER, animation_stepper_iface_init)
                         G_ADD_PRIVATE (AnimationStepperWrapper))

enum {
  PROP_0,
  PROP_STEPPER,
  NPROPS
};

static float
animation_stepper_wrapper_step (AnimationStepper *stepper,
                                unsigned int      ms)
{
  AnimationStepperWrapperPrivate *priv =
    reinterpret_cast <AnimationStepperWrapperPrivate *> (animation_stepper_wrapper_get_instance_private (ANIMATION_STEPPER_WRAPPER (stepper)));

  return (*priv->stepper) (ms);
}

static void
animation_stepper_wrapper_set_property (GObject      *object,
                                       guint         prop_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
  AnimationStepperWrapper *holder = ANIMATION_STEPPER_WRAPPER (object);
  AnimationStepperWrapperPrivate *priv =
    reinterpret_cast <AnimationStepperWrapperPrivate *> (animation_stepper_wrapper_get_instance_private (holder));

  switch (prop_id)
    {
    case PROP_STEPPER:
      priv->stepper = reinterpret_cast <as::Stepper *> (g_value_get_pointer (value));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_stepper_wrapper_get_property (GObject    *object,
                                       guint       prop_id,
                                       GValue     *value,
                                       GParamSpec *pspec)
{
  AnimationStepperWrapper *holder = ANIMATION_STEPPER_WRAPPER (object);
  AnimationStepperWrapperPrivate *priv =
    reinterpret_cast <AnimationStepperWrapperPrivate *> (animation_stepper_wrapper_get_instance_private (holder));

  switch (prop_id)
    {
    case PROP_STEPPER:
      g_value_set_pointer (value, reinterpret_cast <gpointer> (priv->stepper));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_stepper_wrapper_finalize (GObject *object)
{
  AnimationStepperWrapper *holder = ANIMATION_STEPPER_WRAPPER (object);
  AnimationStepperWrapperPrivate *priv =
    reinterpret_cast <AnimationStepperWrapperPrivate *> (animation_stepper_wrapper_get_instance_private (holder));

  delete priv->stepper;
  priv->stepper = nullptr;

  G_OBJECT_CLASS (animation_stepper_wrapper_parent_class)->finalize (object);
}

static void
animation_stepper_wrapper_init (AnimationStepperWrapper *stepper_wrapper)
{
}

static void
animation_stepper_iface_init (AnimationStepperInterface *interface)
{
  interface->step = animation_stepper_wrapper_step;
}

static void
animation_stepper_wrapper_class_init (AnimationStepperWrapperClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->get_property = animation_stepper_wrapper_get_property;
  object_class->set_property = animation_stepper_wrapper_set_property;
  object_class->finalize = animation_stepper_wrapper_finalize;

  g_object_class_override_property (object_class,
                                    PROP_STEPPER,
                                    "stepper");
}

/**
 * animation_stepper_wrapper_new: (skip):
 * @interface: A pointer to an interlying stepper implementation.
 *
 * Create a new #AnimationStepperWrapper, an implementation of
 * #AnimationStepper which only observes the underlying stepper when it
 * is constructed (keeping its internal state). However, the wrapper
 * must not outlive the underlying stepper.
 *
 * Returns: (transfer full): A new #AnimationStepper with the underlying
 *                           stepper merely observed.
 */ 
AnimationStepper *
animation_stepper_wrapper_new (gpointer interface)
{
  return ANIMATION_STEPPER (g_object_new (ANIMATION_TYPE_STEPPER_WRAPPER, "stepper", interface, NULL));
}
