/*
 * animation-glib/stepper/stepper-holder.cpp
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
 * GObject base class for steppers. Copies the stepper
 * passed to it in its construct parameters.
 */

#include <animation/stepper/stepper.h>
#include <animation/stepper/linear.h>

#include <animation-glib/stepper/stepper.h>
#include <animation-glib/stepper/stepper-holder.h>

namespace as = animation::stepper;

struct _AnimationStepperHolder
{
  GObject parent_instance;
};

typedef struct _AnimationStepperHolderPrivate
{
  as::Stepper *stepper;
} AnimationStepperHolderPrivate;

static void animation_stepper_iface_init (AnimationStepperInterface *stepper_iface);

G_DEFINE_TYPE_WITH_CODE (AnimationStepperHolder,
                         animation_stepper_holder,
                         G_TYPE_OBJECT,
                         G_IMPLEMENT_INTERFACE (ANIMATION_TYPE_STEPPER, animation_stepper_iface_init)
                         G_ADD_PRIVATE (AnimationStepperHolder))

enum {
  PROP_0,
  PROP_STEPPER,
  NPROPS
};

static float
animation_stepper_holder_step (AnimationStepper *stepper,
                               unsigned int      ms)
{
  AnimationStepperHolderPrivate *priv =
    reinterpret_cast <AnimationStepperHolderPrivate *> (animation_stepper_holder_get_instance_private (ANIMATION_STEPPER_HOLDER (stepper)));

  return (*priv->stepper) (ms);
}

template <typename T>
static T *
copy_ptr_if_set (T *ptr)
{
  if (ptr != nullptr)
    return new T (*ptr);

  return nullptr;
}

static void
animation_stepper_holder_set_property (GObject      *object,
                                       guint         prop_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
  AnimationStepperHolder *holder = ANIMATION_STEPPER_HOLDER (object);
  AnimationStepperHolderPrivate *priv =
    reinterpret_cast <AnimationStepperHolderPrivate *> (animation_stepper_holder_get_instance_private (holder));

  switch (prop_id)
    {
    case PROP_STEPPER:
      /* We need to copy-construct here as the property is
       * not transfer full.  */
      priv->stepper = copy_ptr_if_set (reinterpret_cast <as::Stepper *> (g_value_get_pointer (value)));
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_stepper_holder_get_property (GObject    *object,
                                       guint       prop_id,
                                       GValue     *value,
                                       GParamSpec *pspec)
{
  AnimationStepperHolder *holder = ANIMATION_STEPPER_HOLDER (object);
  AnimationStepperHolderPrivate *priv =
    reinterpret_cast <AnimationStepperHolderPrivate *> (animation_stepper_holder_get_instance_private (holder));

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
animation_stepper_holder_finalize (GObject *object)
{
  AnimationStepperHolder *holder = ANIMATION_STEPPER_HOLDER (object);
  AnimationStepperHolderPrivate *priv =
    reinterpret_cast <AnimationStepperHolderPrivate *> (animation_stepper_holder_get_instance_private (holder));

  delete priv->stepper;
  priv->stepper = nullptr;

  G_OBJECT_CLASS (animation_stepper_holder_parent_class)->finalize (object);
}

static GObject *
animation_stepper_holder_constructor (GType                  type,
                                      unsigned int           n_construct_params,
                                      GObjectConstructParam *construct_params)
{
  auto defaultStepperHolder = as::Linear (300);

  /* Check the passed stepper_holder prop to ensure that it is set. If not,
   * then we need to set it to some sensible default value. */
  for (unsigned int i = 0; i < n_construct_params; ++i)
    {
      if (g_strcmp0 (construct_params[i].pspec->name, "stepper") == 0)
        {
          if (g_value_get_pointer (construct_params[i].value) == nullptr)
            g_value_set_pointer (construct_params[i].value, &defaultStepperHolder);
        }
    }

  return G_OBJECT_CLASS (animation_stepper_holder_parent_class)->constructor (type,
                                                                       n_construct_params,
                                                                       construct_params);
}

static void
animation_stepper_holder_init (AnimationStepperHolder *stepper_holder)
{
}

static void
animation_stepper_iface_init (AnimationStepperInterface *interface)
{
  interface->step = animation_stepper_holder_step;
}

static void
animation_stepper_holder_class_init (AnimationStepperHolderClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_stepper_holder_constructor;
  object_class->get_property = animation_stepper_holder_get_property;
  object_class->set_property = animation_stepper_holder_set_property;
  object_class->finalize = animation_stepper_holder_finalize;

  g_object_class_override_property (object_class,
                                    PROP_STEPPER,
                                    "stepper");
}

/**
 * animation_stepper_holder_new: (skip):
 * @interface: A pointer to an interlying stepper implementation.
 *
 * Create a new #AnimationStepperHolder, an implementation of
 * #AnimationStepper which copies the underlying stepper when it
 * is constructed (thus resetting its internal state, as copying
 * a C++ lambda default-constructs its closure).
 *
 * Returns: (transfer full): A new #AnimationStepper with the underlying
 *                           stepper copied.
 */ 
AnimationStepper *
animation_stepper_holder_new (gpointer interface)
{
  return ANIMATION_STEPPER (g_object_new (ANIMATION_TYPE_STEPPER_HOLDER, "stepper", interface, NULL));
}
