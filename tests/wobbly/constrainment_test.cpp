/*
 * tests/wobbly/constrainment_test.cpp
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
#include <array>                        // for array, operator==
#include <functional>                   // for __base
#include <ostream>                      // for operator<<, char_traits, etc
#include <vector>                       // for vector

#include <math.h>                       // for fabs
#include <stddef.h>                     // for size_t

#include <gmock/gmock-matchers.h>       // for Matcher, ElementsAreArray, etc
#include <gmock/gmock.h>                // IWYU pragma: keep
#include <gtest/gtest-param-test.h>     // for Values, Combine, ValuesIn, etc
#include <gtest/gtest.h>                // for AssertHelper, TEST_F, etc

#include <animation/wobbly/wobbly.h>    // for Point, PointView
#include <animation/wobbly/wobbly_internal.h>            // for TargetMesh, etc

#include <mathematical_model_matcher.h>  // for EqDispatchHelper, Eq, etc
#include "ostream_point_operator.h"     // for operator<<

using ::testing::ElementsAreArray;
using ::testing::Matcher;
using ::testing::Not;
using ::testing::Test;
using ::testing::ValuesIn;
using ::testing::WithParamInterface;

using ::wobbly::matchers::Eq;

namespace
{
    namespace agd = animation::geometry::dimension;

    double TileWidth (double width)
    {
        return width / (wobbly::config::Width - 1);
    }

    double TileHeight (double height)
    {
        return height / (wobbly::config::Height - 1);
    }

    void
    InitializePositionsWithDimensions (wobbly::MeshArray &positions,
                                       double            width,
                                       double            height)
    {
        double const meshWidth = wobbly::config::Width;
        double const meshHeight = wobbly::config::Height;
        double const tileWidth = TileWidth (width);
        double const tileHeight = TileHeight (height);

        for (unsigned int i = 0; i < meshHeight; ++i)
        {
            for (unsigned int j = 0; j < meshWidth; ++j)
            {
                animation::PointView <double> pv (positions,
                                                  i * meshWidth + j);
                agd::assign (pv,
                             animation::Point (tileWidth * j,
                                               tileHeight * i));
            }
        }
    }

    constexpr double TextureWidth = 50.0f;
    constexpr double TextureHeight = 100.0f;
    animation::Point const TextureCenter = animation::Point (TextureWidth / 2,
                                                             TextureHeight / 2);

    class ConstrainmentStep :
        public Test
    {
        public:

            ConstrainmentStep () :
                range (10),
                width (TextureWidth),
                height (TextureHeight),
                targets ([this](wobbly::MeshArray &mesh){
                    InitializePositionsWithDimensions (mesh,
                                                       TextureWidth,
                                                       TextureHeight);
                }),
                constrainment (range, targets)
            {
                InitializePositionsWithDimensions (positions,
                                                   TextureWidth,
                                                   TextureHeight);
            }

            double                    range;
            double                    width;
            double                    height;

            wobbly::TargetMesh targets;
            wobbly::MeshArray positions;
            wobbly::AnchorArray anchors;

            wobbly::ConstrainmentStep constrainment;
    };

    TEST_F (ConstrainmentStep, PointsNotAffectedWhereNoAnchorGrabbed)
    {
        /* Make a separate copy of the array and test against it later */
        wobbly::MeshArray expectedPositions;

        InitializePositionsWithDimensions (expectedPositions,
                                           TextureWidth,
                                           TextureHeight);

        constrainment (positions, anchors);

        EXPECT_EQ (expectedPositions, positions);
    }

    TEST_F (ConstrainmentStep, ReturnsTrueWhereConstrainmentTookPlace)
    {
        auto handleOwner (targets.Activate ());
        animation::PointView <double> pv (positions, 1);
        agd::pointwise_add (pv, animation::Point (range * 2, range * 2));

        EXPECT_TRUE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ReturnsFalseWhereNoConstrainmentTookPlace)
    {
        auto handleOwner (targets.Activate ());
        animation::PointView <double> pv (positions, 1);

        /* Not enough to cause constrainment */
        agd::pointwise_add (pv, animation::Point (range / 2, 0));

        EXPECT_FALSE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ConstrainedToPointsOnTargetMesh)
    {
        animation::Point const movement (range * 2, 0);
        auto handleOwner (targets.Activate ());
        wobbly::MoveOnly <wobbly::TargetMesh::Move> &handleWrap (handleOwner);
        wobbly::TargetMesh::Move const &moveBy (handleWrap);

        moveBy (movement);

        /* Move the anchored point right by range * 2, the result should
         * be that every other point is p.x + range */
        constrainment (positions, anchors);

        wobbly::MeshArray expectedPositions;

        InitializePositionsWithDimensions (expectedPositions,
                                           TextureWidth,
                                           TextureHeight);

        /* Add range.x to each point */
        for (size_t i = 0; i < positions.size () / 2; ++i)
        {
            animation::PointView <double> pv (expectedPositions, i);
            agd::pointwise_add (pv, animation::Point (range, 0));
        }

        std::vector <Matcher <double>> matchers;
        for (double expected : expectedPositions)
            matchers.push_back (::testing::DoubleEq (expected));

        EXPECT_THAT (positions, ElementsAreArray (&matchers[0],
                                                  matchers.size ()));
    }

    typedef std::tuple <size_t, double> ConstrainmentStepPositionsParam;

    class ConstrainmentStepPositions :
        public ConstrainmentStep,
        public WithParamInterface <ConstrainmentStepPositionsParam>
    {
        public:

            ConstrainmentStepPositions () :
                handle (targets.Activate ()),
                index (std::get <0> (GetParam ())),
                ratio (std::get <1> (GetParam ()))
            {
                anchors.Lock ((wobbly::config::Width *
                               wobbly::config::Height) / 2);
            }

            wobbly::TargetMesh::Hnd handle;
            size_t index;
            double ratio;
    };

    TEST_P (ConstrainmentStepPositions, NotAffectedWhereWithinRadiusOfRange)
    {
        double const absratio = std::fabs (ratio);

        /* Prevent zero-division. Treat zero as positive */
        double const sign = ratio / (absratio + (absratio == 0.0));
        double const radiusInRange = range - (range / 2);

        animation::PointView <double> pv (positions, index);
        animation::Point expected;

        agd::pointwise_add (pv,
                            animation::Point (radiusInRange * ratio,
                                              radiusInRange * (1 - absratio) * sign));

        /* Expected point is the modified point here, before constrainment */
        agd::pointwise_add (expected, pv);

        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    TEST_P (ConstrainmentStepPositions, AffectedWhereOutsideRadiusOfRange)
    {
        double absratio = std::fabs (ratio);

        /* Prevent zero-division. Treat zero as positive */
        double const sign = ratio / (absratio + (absratio == 0.0));
        double const radiusOutOfRange = range * range;

        animation::Point outOfRange (radiusOutOfRange * ratio,
                                     radiusOutOfRange * (1 - absratio) * sign);
        animation::Point inRange (range * ratio,
                                  range * (1 - absratio) * sign);

        animation::PointView <double> pv (positions, index);
        animation::Point expected;

        /* Expected point is the actual grid point, but at its maximum range */
        agd::assign (expected, pv);
        agd::pointwise_add (expected, inRange);

        agd::pointwise_add (pv, outOfRange);
        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    void ScalePositionMesh (wobbly::MeshArray       &array,
                            animation::Point const  &origin,
                            animation::Vector const &scaleFactor)
    {
        for (size_t i = 0; i < wobbly::config::TotalIndices; ++i)
        {
            animation::PointView <double> p (array, i);
            agd::pointwise_subtract (p, origin);
            agd::pointwise_scale (p, scaleFactor);
            agd::pointwise_add (p, origin);
        }
    }

    TEST_P (ConstrainmentStepPositions, NotAffectedWhereBothMeshesResizeEvenly)
    {
        animation::Vector scaleFactor (range * 4, range * 4);
        ScalePositionMesh (positions,
                           animation::Point (positions[0], positions[1]),
                           scaleFactor);
        ScalePositionMesh (targets.PointArray (),
                           animation::Point (targets.PointArray ()[0],
                                             targets.PointArray ()[1]),
                           scaleFactor);

        animation::PointView <double> pv (positions, index);
        animation::Point expected;

        /* Expected point is the same point here, before constrainment */
        agd::assign (expected, pv);

        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    TEST_P (ConstrainmentStepPositions, AffectedWhereBothMeshesResizeUnevenly)
    {
        animation::Vector scaleFactor (range * 4, range * 4);
        ScalePositionMesh (positions,
                           animation::Point (wobbly::config::Width / 2,
                                             wobbly::config::Height / 2),
                           scaleFactor);

        animation::PointView <double> pv (positions, index);
        animation::Point expected;

        /* Not expecting the same point before constrainment */
        agd::assign (expected, pv);

        constrainment (positions, anchors);

        EXPECT_THAT (pv, Not (Eq (expected)));
    }

    ConstrainmentStepPositionsParam const constrainmentStepParams[] =
    {
        ConstrainmentStepPositionsParam (0, -1.0),
        ConstrainmentStepPositionsParam (wobbly::config::Width - 1, 1.0),
        ConstrainmentStepPositionsParam ((wobbly::config::Height - 1) *
                                         wobbly::config::Width, 0.0),
        ConstrainmentStepPositionsParam (wobbly::config::Width *
                                         wobbly::config::Height - 1, 0.0)
    };

    INSTANTIATE_TEST_CASE_P (Extremes, ConstrainmentStepPositions,
                             ValuesIn (constrainmentStepParams));
}
