/*
 * animation/grid/grid.h
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
 * Interface definition for animations that perform
 * a 2D grid based deformation of surface unit co-ordinates.
 */

#include <array>

#include <animation/geometry.h>

#pragma once

namespace animation
{
    namespace grid
    {
        class GridAnimation
        {
            public:

                virtual ~GridAnimation() {};
                virtual animation::Point DeformUVToModelSpace (animation::Point const &) const = 0;
                virtual animation::geometry::PointModel <size_t> Resolution () const = 0;
                virtual float Progress () const = 0;
                virtual bool Step (unsigned int ms) = 0;
                virtual std::array <animation::Vector4D, 4> Extremes (std::array<animation::Point, 4> const &corners) const = 0;
        };
    }
}
