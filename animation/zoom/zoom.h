/*
 * animation/zoom/zoom.h
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
 * Animation that causes a surface to zoom from one rectangle to another.
 */

#include <array>
#include <memory>

#include <animation/geometry.h>
#include <animation/property.h>
#include <animation/stepper/stepper.h>
#include <animation/transform/transform.h>

#pragma once

namespace animation
{
    namespace zoom
    {
        class ZoomAnimation :
            public animation::transform::TransformAnimation
        {
            public:

                ZoomAnimation (animation::Box <animation::Point> const &from,
                               animation::Box <animation::Point> const &to,
                               animation::stepper::Stepper       const &stepper);
                ~ZoomAnimation ();

                float * const Matrix () const;
                float Progress () const;
                bool Step (unsigned int ms);
                std::array<animation::Vector4D, 4> Extremes (std::array<animation::Point, 4> const &corners) const;

                ANIMATION_DECLARE_PROPERTY (ZoomAnimation, From, animation::Box <animation::Point>)
                ANIMATION_DECLARE_READONLY_PROPERTY (ZoomAnimation, To, animation::Box <animation::Point>)
                ANIMATION_DECLARE_PROPERTY (ZoomAnimation, Stepper, animation::stepper::Stepper)

            private:

                struct Private;
                std::unique_ptr <Private> priv;
        };
    }
}
