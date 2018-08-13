/*
 * animation/box_calculation.h
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
 * Utility functions to calculate points and box offsets.
 */
#pragma once

#include <animation/geometry.h>

namespace animation
{
    namespace box_calculation
    {
        inline animation::Point ComputeBoxCenterOffset (animation::Box <animation::Point> const &box)
        {
            namespace agd = animation::geometry::dimension;

            return animation::Point ((agd::get <0> (box.bottomRight ()) -
                                      agd::get <0> (box.topLeft ())) / 2.0,
                                     (agd::get <1> (box.bottomRight ()) -
                                      agd::get <1> (box.topLeft ())) / 2.0);
        }

        inline animation::Point ComputeBoxCenter (animation::Box <animation::Point> const &box)
        {
            namespace agd = animation::geometry::dimension;

            animation::Point topLeft (box.topLeft ());
            agd::pointwise_add (topLeft, ComputeBoxCenterOffset (box));
            return topLeft;
        }

        inline animation::Point ComputeRotationAxisOffset (animation::Box <animation::Point> const &box,
                                                           float                                   u,
                                                          float                                   v)
        {
            namespace agd = animation::geometry::dimension;

            return animation::Point ((agd::get <0> (box.bottomRight ()) -
                                      agd::get <0> (box.topLeft ())) * u,
                                     (agd::get <1> (box.bottomRight ()) -
                                      agd::get <1> (box.topLeft ())) * v);
        }

        inline animation::Point ComputeRotationAxis (animation::Box <animation::Point> const &box,
                                                     float                                   u,
                                                     float                                   v)
        {
            namespace agd = animation::geometry::dimension;

            animation::Point topLeft (box.topLeft ());
            agd::pointwise_add (topLeft, ComputeRotationAxisOffset (box, u, v));
            return topLeft;
        }
    }
}
