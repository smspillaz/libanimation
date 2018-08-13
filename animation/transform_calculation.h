/*
 * animation/transform_calculation.h
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
 * Utility functions with some helpful matrix transforms.
 */
#pragma once

#include <animation/geometry.h>

#include <glm/glm.hpp>

namespace animation
{
    namespace transform_calculation
    {
        /* This function does not take into account depth, since
         * it won't project the vector - it only throws away the
         * depth component. Thus, it isn't very useful for finding
         * bounding boxes where a projection would be required. */
        inline animation::Point TransformFlattened2DPointBy3DMatrix (animation::Point const &p,
                                                                     glm::mat4        const &matrix)
        {
            namespace agd = animation::geometry::dimension;

            glm::vec4 t (matrix * glm::vec4 (agd::get <0> (p), agd::get <1> (p), 0.0, 1.0));
            return animation::Point (t[0], t[1]);
        }
    }
}
