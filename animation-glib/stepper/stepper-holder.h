/*
 * animation-glib/stepper/stepper-holder.h
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
 * GObject base class for steppers, which holds a copy of a stepper.
 */
#pragma once

#include <glib-object.h>

#include <animation-glib/stepper/stepper.h>
#include <animation-glib/vector.h>

G_BEGIN_DECLS

#define ANIMATION_TYPE_STEPPER_HOLDER animation_stepper_holder_get_type ()
G_DECLARE_FINAL_TYPE (AnimationStepperHolder, animation_stepper_holder, ANIMATION, STEPPER_HOLDER, GObject)

AnimationStepper * animation_stepper_holder_new (gpointer interface);

G_END_DECLS
