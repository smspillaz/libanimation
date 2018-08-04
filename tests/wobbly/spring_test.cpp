/*
 * tests/spring_test.cpp
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
#include <array>                        // for array
#include <functional>                   // for function, __base
#include <memory>                       // for unique_ptr
#include <sstream>                      // for char_traits, etc
#include <type_traits>                  // for move

#include <stdlib.h>                     // for exit

#include <gmock/gmock-matchers.h>       // for EXPECT_THAT, etc
#include <gtest/gtest-death-test.h>     // for DeathTest, ExitedWithCode, etc
#include <gtest/gtest.h>                // for AssertHelper, TEST_F, etc
#include <gmock/gmock.h>                // IWYU pragma: keep

#include <animation/wobbly/wobbly.h>    // for PointView, Point, Vector
#include <animation/wobbly/wobbly_internal.h>            // for Spring, etc

#include <mathematical_model_matcher.h>  // for Eq, EqDispatchHelper, etc
#include "ostream_point_operator.h"     // for operator<<

using ::testing::ExitedWithCode;
using ::testing::Not;
using ::testing::Test;

using ::animation::matchers::Eq;
using ::animation::matchers::SatisfiesModel;

using ::animation::models::Linear;

namespace
{
    namespace agd = animation::geometry::dimension;

    class SingleObjectStorage
    {
        public:

            SingleObjectStorage ()
            {
                storage.fill (0);
            }

            animation::PointView <double> Position ()
            {
                return animation::PointView <double> (storage, 0);
            }

            animation::PointView <double> Velocity ()
            {
                return animation::PointView <double> (storage, 1);
            }

            animation::PointView <double> Force ()
            {
                return animation::PointView <double> (storage, 2);
            }

        private:

            std::array <double, 6> storage;
    };

    class SingleObjectStorageView
    {
        public:

            SingleObjectStorageView (SingleObjectStorage &storage) :
                position (storage.Position ()),
                velocity (storage.Velocity ()),
                force (storage.Force ())
            {
            }

            animation::PointView <double> position;
            animation::PointView <double> velocity;
            animation::PointView <double> force;
    };

    constexpr double FirstPositionX = 50.0f;
    constexpr double FirstPositionY = 50.0f;

    constexpr double SecondPositionX = 100.0f;
    constexpr double SecondPositionY = 100.0f;

    constexpr double SpringConstant = 0.5f;

    TEST (Spring, MoveConstructorNoExcept)
    {
        SingleObjectStorage storageA, storageB;

        EXPECT_NO_THROW ({
            wobbly::Spring a (storageA.Force (),
                              storageB.Force (),
                              storageA.Position (),
                              storageB.Position (),
                              animation::Vector (0, 0));
            wobbly::Spring b (std::move (a));
        });
    }

    class Springs :
        public ::testing::Test
    {
        public:

            Springs ():
                desiredDistance (SecondPositionX - FirstPositionX,
                                 SecondPositionY - FirstPositionY),
                first (firstStorage),
                second (secondStorage),
                spring (firstStorage.Force (),
                        secondStorage.Force (),
                        firstStorage.Position (),
                        secondStorage.Position (),
                        desiredDistance)
            {
                agd::assign (first.position,
                             animation::Point (FirstPositionX,
                                               FirstPositionY));
                agd::assign (second.position,
                             animation::Point (SecondPositionX,
                                               SecondPositionY));
            }

        protected:

            animation::Vector desiredDistance;

        private:

            SingleObjectStorage firstStorage;
            SingleObjectStorage secondStorage;

        protected:

            SingleObjectStorageView first;
            SingleObjectStorageView second;
            wobbly::Spring spring;
    };

    TEST_F (Springs, NoForceAppliedWhenNoDeltaFromDesired)
    {
        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force,
                     Eq (animation::Vector (0, 0)));
        EXPECT_THAT (second.force,
                     Eq (animation::Vector (0, 0)));
    }

    template <typename Position>
    animation::Vector
    ForceForSpring (Position          const &first,
                    Position          const &second,
                    animation::Vector const &desired)
    {
        animation::Vector expectedForce;
        agd::assign (expectedForce, second);
        agd::pointwise_subtract (expectedForce, first);
        agd::pointwise_subtract (expectedForce, desired);
        agd::scale (expectedForce, SpringConstant);
        agd::scale (expectedForce, 1 / 2.0);

        return expectedForce;
    }

    TEST_F (Springs, ForceAppliedToFirstObjectProportianalToPositiveDistanceSK)
    {
        agd::assign (first.position,
                     animation::Vector (FirstPositionX - 10.0f,
                                        FirstPositionY - 10.0f));
        animation::Vector expectedForce (ForceForSpring (first.position,
                                                         second.position,
                                                         desiredDistance));

        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (expectedForce));
    }

    TEST_F (Springs, ForceAppliedToSecondObjectProportionalToNegativeDistanceSK)
    {
        agd::assign (first.position, animation::Vector (FirstPositionX - 10.0f,
                                                        FirstPositionY - 10.0f));
        animation::Vector negativeDistance (desiredDistance);
        agd::scale (negativeDistance, -1.0f);

        animation::Vector expectedForce (ForceForSpring (second.position,
                                                         first.position,
                                                         negativeDistance));

        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (second.force, Eq (expectedForce));
    }

    TEST_F (Springs, ForceAccumulatesWithApplications)
    {
        agd::assign (first.position,
                     animation::Vector (FirstPositionX - 10.0f,
                                        FirstPositionY - 10.0f));
        animation::Vector expectedForce (ForceForSpring (first.position,
                                                         second.position,
                                                         desiredDistance));

        /* Scalar for single spring */
        unsigned int const nApplications = 3;
        agd::scale (expectedForce, nApplications);

        for (unsigned int i = 0; i < nApplications; ++i)
            spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (expectedForce));
    }

    TEST_F (Springs, ForceRelationshipIsLinearlyRelatedToDistance)
    {
        std::function <double (int)> forceByDistanceFunction =
            [this](int delta) -> double {
                agd::assign (first.force, animation::Vector (0, 0));
                agd::assign (second.force, animation::Vector (0, 0));
                agd::assign (first.position,
                             animation::Vector (FirstPositionX - delta,
                                                FirstPositionY));
                agd::assign (second.position,
                             animation::Vector (SecondPositionX + delta,
                                                SecondPositionY));

                spring.ApplyForces (SpringConstant);

                return agd::get <0> (first.force);
            };

        EXPECT_THAT (forceByDistanceFunction,
                     SatisfiesModel (Linear <double> ()));
    }

    TEST_F (Springs, ForceClippedWithVerySmallDelta)
    {
        double const justBelowThreshold = FirstPositionX -
                                          wobbly::Spring::ClipThreshold * 1.1;

        agd::assign (first.position,
                     animation::Vector (justBelowThreshold, FirstPositionY));
        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (animation::Vector (0, 0)));
    }

    TEST_F (Springs, ApplyForcesReturnsTrueIfForceRemaining)
    {
        /* Change the position of one object, that will cause forces
         * to be exerted
         */
        agd::assign (first.position,
                     animation::Vector (FirstPositionX - 10,
                                        FirstPositionY));

        EXPECT_TRUE (spring.ApplyForces (SpringConstant));
    }

    TEST_F (Springs, ApplyForcesReturnsFalseIfNoForceRemaining)
    {
        /* Where there is no delta, there is no force */
        EXPECT_FALSE (spring.ApplyForces (0.0f));
    }

    double const SpringScaleFactor = 2.0f;

    TEST_F (Springs, ForceExistsAfterLengthScaled)
    {
        spring.ScaleLength (animation::Vector (SpringScaleFactor,
                                               SpringScaleFactor));
        EXPECT_TRUE (spring.ApplyForces (SpringConstant));
    }

    TEST_F (Springs, NoForceAfterLengthScaledAndObjectsMoved)
    {
        /* Calculate distance between first and second, then adjust
         * second object's position to be distance * scaleFactor */
        animation::Vector distance;
        agd::assign (distance, second.position);
        agd::pointwise_subtract (distance, first.position);
        agd::pointwise_subtract (second.position, distance);
        agd::scale (distance, SpringScaleFactor);
        agd::pointwise_add (second.position, distance);

        spring.ScaleLength (animation::Vector (SpringScaleFactor,
                                               SpringScaleFactor));
        EXPECT_FALSE (spring.ApplyForces (SpringConstant));
    }
}
