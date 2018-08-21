/*
 * animation/stepper/linear.h
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
 * Definition for a reverse stepping function. Takes another stepping
 * function and inverts its output.
 */

#include <functional>

#include <animation/stepper/stepper.h>

#pragma once

namespace animation
{
    namespace stepper
    {
        inline std::function <float (unsigned int)> Reverse (Stepper &&step) {
            Stepper localStepper (std::move (step));

            return [=](unsigned int ms) mutable -> float {
                return 1.0f - localStepper (ms);
            };
        }
    }
}
