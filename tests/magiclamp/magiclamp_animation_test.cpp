/*
 * tests/wobbly/magiclamp_animation_test.cpp
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
 * Tests for the "wobbly" spring model.
 */
#include <cstddef>                      // for size_t
#include <functional>                   // for bind, __bind, _1

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <gmock/gmock-cardinalities.h>  // for AtLeast
#include <gmock/gmock-generated-function-mockers.h>  // for FunctionMocker, etc
#include <gmock/gmock-matchers.h>       // for AnythingMatcher, etc
#include <gmock/gmock-spec-builders.h>  // for EXPECT_CALL, etc
#include <gtest/gtest.h>                // for TEST_F, Test, Types, etc

#include <glm_ostream_operators.h>
#include <mathematical_model_matcher.h>
#include <ostream_point_operator.h>

#include <animation/geometry.h>
#include <animation/stepper/linear.h>
#include <animation/stepper/reverse.h>
#include <animation/magiclamp/magiclamp.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Eq;
using ::testing::Test;

namespace am = animation::matchers;
namespace aml = animation::magiclamp;
namespace as = animation::stepper;

namespace animation
{
    namespace matchers
    {
        template <>
        struct AlmostEqTrait <animation::Point>
        {
            typedef double ValueType;

            template <class Comparator>
            static bool apply (animation::Point const &lhs,
                               animation::Point const &rhs,
                               Comparator             &&comparator)
            {
                return comparator (agd::get <0> (lhs), agd::get <0> (rhs)) &&
                       comparator (agd::get <1> (lhs), agd::get <1> (rhs));
            }
        };
    }
}

namespace
{
    static animation::geometry::PointModel <size_t> const Resolution (4, 4);
    static double const BendFactor = 10;
    static double const OffsetFactor = 0.5;
    static double const StretchFactor = 0.45;
    static double const DeformationSpeedFactor = 2.3;

    TEST (MagicLampAnimation, AnimationIncompleteBeforeLengthTimesteps)
    {
        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (100, 100),
                                                                         animation::Point (150, 150)),
                                      animation::Box <animation::Point> (animation::Point (0, 0),
                                                                         animation::Point (100, 100)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Linear (200));

        EXPECT_TRUE (anim.Step (199));
    }

    TEST (MagicLampAnimation, AnimationCompleteAfterLengthTimesteps)
    {
        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (100, 100),
                                                                         animation::Point (150, 150)),
                                      animation::Box <animation::Point> (animation::Point (0, 0),
                                                                         animation::Point (100, 100)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Linear (200));

        EXPECT_FALSE (anim.Step (200));
    }

    TEST (MagicLampAnimation, StartsAtCorrectPosition)
    {
        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (100, 100),
                                                                         animation::Point (150, 150)),
                                      animation::Box <animation::Point> (animation::Point (0, 0),
                                                                         animation::Point (100, 100)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Linear (200));

        std::array <animation::Point, 4> corners = {{
            animation::Point (0, 0),
            animation::Point (100, 0),
            animation::Point (0, 100),
            animation::Point (100, 100)
        }};
        std::array <animation::Vector4D, 4> extremes = anim.Extremes (corners);

        EXPECT_THAT (extremes[0], am::Eq (animation::Vector4D (100, 100, 0, 1)));
        EXPECT_THAT (extremes[1], am::Eq (animation::Vector4D (150, 100, 0, 1)));
        EXPECT_THAT (extremes[2], am::Eq (animation::Vector4D (100, 150, 0, 1)));
        EXPECT_THAT (extremes[3], am::Eq (animation::Vector4D (150, 150, 0, 1)));
    }

    TEST (MagicLampAnimation, CorrectLinearInterpolatedVertexDeformation)
    {
        std::vector <animation::Point> clientGrid;
        clientGrid.reserve(128);

        double const x = 0;
        double const y = 0;
        double const width = 100;
        double const height = 100;

        double const iconX = 100;
        double const iconY = 100;
        double const iconWidth = 50;
        double const iconHeight = 50;

        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (iconX, iconY),
                                                                         animation::Point (iconX + iconWidth, iconY + iconHeight)),
                                      animation::Box <animation::Point> (animation::Point (x, y),
                                                                         animation::Point (width, height)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Linear (200));

        size_t const gridWidth = 4;
        size_t const gridHeight = 100;

        for (size_t i = 0; i < gridWidth; ++i)
        {
            for (size_t j = 0; j < gridHeight; ++j)
            {
                EXPECT_THAT (anim.DeformUVToModelSpace (animation::Point (i / static_cast <double> (gridWidth - 1),
                                                                          j / static_cast <double> (gridHeight - 1))),
                             am::AlmostEq (animation::Point (iconX + (iconWidth / (gridWidth - 1)) * i,
                                                             iconY + (iconHeight / (gridHeight - 1)) * j),
                                           0.002));
            }
        }
    }

    TEST (MagicLampAnimation, AnimatesToCorrectPositionAfterLengthTimesteps)
    {
        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (100, 100),
                                                                         animation::Point (150, 150)),
                                      animation::Box <animation::Point> (animation::Point (0, 0),
                                                                         animation::Point (100, 100)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Linear (200));
        anim.Step (200);

        std::array <animation::Point, 4> corners = {{
            animation::Point (0, 0),
            animation::Point (100, 0),
            animation::Point (0, 100),
            animation::Point (100, 100)
        }};
        std::array <animation::Vector4D, 4> extremes = anim.Extremes (corners);

        EXPECT_THAT (extremes[0], am::Eq (animation::Vector4D (0, 0, 0, 1)));
        EXPECT_THAT (extremes[1], am::Eq (animation::Vector4D (100, 0, 0, 1)));
        EXPECT_THAT (extremes[2], am::Eq (animation::Vector4D (0, 100, 0, 1)));
        EXPECT_THAT (extremes[3], am::Eq (animation::Vector4D (100, 100, 0, 1)));
    }

    TEST (MagicLampAnimation, AnimatesToCorrectPositionAfterLengthTimestepsReverse)
    {
        aml::MagicLampAnimation anim (animation::Box <animation::Point> (animation::Point (100, 100),
                                                                         animation::Point (150, 150)),
                                      animation::Box <animation::Point> (animation::Point (0, 0),
                                                                         animation::Point (100, 100)),
                                      Resolution,
                                      BendFactor,
                                      OffsetFactor,
                                      StretchFactor,
                                      DeformationSpeedFactor,
                                      as::Reverse (as::Linear (200)));
        anim.Step (200);

        std::array <animation::Point, 4> corners = {{
            animation::Point (0, 0),
            animation::Point (100, 0),
            animation::Point (0, 100),
            animation::Point (100, 100)
        }};
        std::array <animation::Vector4D, 4> extremes = anim.Extremes (corners);

        EXPECT_THAT (extremes[0], am::Eq (animation::Vector4D (100, 100, 0, 1)));
        EXPECT_THAT (extremes[1], am::Eq (animation::Vector4D (150, 100, 0, 1)));
        EXPECT_THAT (extremes[2], am::Eq (animation::Vector4D (100, 150, 0, 1)));
        EXPECT_THAT (extremes[3], am::Eq (animation::Vector4D (150, 150, 0, 1)));
    }
}
