/*
 * animation/bounce/bounce.h
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
 * An animation that causes a surface to bounce onto screen, gently
 * following an attenuating sine wave.
 */

#include <array>
#include <memory>

#include <animation/geometry.h>

#pragma once

namespace animation
{
    namespace bounce
    {
        class BounceAnimation
        {
            public:

                BounceAnimation (float                                   initialScale,
                                 float                                   maximumScale,
                                 unsigned int                            nBounce,
                                 animation::Box <animation::Point> const &target,
                                 unsigned int                            length);
                ~BounceAnimation ();

                float * const Matrix () const;
                bool Step (unsigned int ms);
                std::array<animation::Point, 4> Extremes (std::array<animation::Point, 4> const &corners) const;

            private:

                struct Private;
                std::unique_ptr <Private> priv;
        };
    }
}
