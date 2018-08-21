/*
 * animation/stepper/stepper.h
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
 * Definition for a stepping function, taking an unsigned int of
 * milliseconds passed and returning a floating point number
 * between 0.0 and 1.0.
 */

#include <functional>

#pragma once

namespace animation
{
    namespace stepper
    {
        typedef std::function <float (unsigned int)> Stepper;
    }
}
