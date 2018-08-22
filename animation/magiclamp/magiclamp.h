/*
 * animation/magiclamp/magiclamp.h
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
#include <animation/grid/grid.h>
#include <animation/property.h>
#include <animation/stepper/stepper.h>

#pragma once

namespace animation
{
    namespace magiclamp
    {
        class MagicLampAnimation :
            public animation::grid::GridAnimation
        {
            public:

                MagicLampAnimation (animation::Box <animation::Point>        const &source,
                                    animation::Box <animation::Point>        const &target,
                                    animation::geometry::PointModel <size_t> const &resolution,
                                    float                                           bendFactor,
                                    float                                           offsetFactor,
                                    float                                           stretchFactor,
                                    float                                           deformSpeedFactor,
                                    animation::stepper::Stepper              const &stepper);
                ~MagicLampAnimation ();

                animation::Point DeformUVToModelSpace (animation::Point const &) const;
                animation::geometry::PointModel <size_t> Resolution () const;
                float Progress () const;
                bool Step (unsigned int ms);
                std::array<animation::Vector4D, 4> Extremes (std::array<animation::Point, 4> const &corners) const;

                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, Source, animation::Box <animation::Point>)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, Target, animation::Box <animation::Point>)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, GridResolution, animation::geometry::PointModel <size_t>)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, BendFactor, float)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, OffsetFactor, float)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, StretchFactor, float)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, DeformSpeedFactor, float)
                ANIMATION_DECLARE_PROPERTY (MagicLampAnimation, Stepper, animation::stepper::Stepper)

            private:

                struct Private;
                std::unique_ptr <Private> priv;
        };
    }
}
