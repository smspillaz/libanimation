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
 * Definition for a linear stepping function - just adds to progress
 * according to how long the animation is.
 */

#include <functional>

#pragma once

namespace animation
{
    namespace stepper
    {
        inline std::function <float (unsigned int)> Linear (float length) {
            float progress = 0.0;

            return [=](unsigned int ms) mutable -> float {
                progress += ms / length;
                return progress;
            };
        }
    }
}
