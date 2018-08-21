/*
 * animation-glib/stepper/stepper.cpp
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
 * GObject base interface for steppers.
 */

#include <animation/stepper/stepper.h>
#include <animation-glib/stepper/stepper.h>

namespace as = animation::stepper;


G_DEFINE_INTERFACE (AnimationStepper,
                    animation_stepper,
                    G_TYPE_OBJECT)
float
animation_stepper_step (AnimationStepper *stepper,
                        unsigned int      ms)
{
  g_return_val_if_fail (ANIMATION_IS_STEPPER (stepper), 0.0f);

  return ANIMATION_STEPPER_GET_IFACE (stepper)->step (stepper, ms);
}

static void
animation_stepper_default_init (AnimationStepperInterface *iface)
{
  g_object_interface_install_property ((gpointer) iface,
                                       g_param_spec_pointer ("stepper",
                                                             "Internal Interface",
                                                             "Internal C++ interface that this class wraps",
                                                             static_cast <GParamFlags> (G_PARAM_READWRITE | G_PARAM_CONSTRUCT_ONLY)));
}
