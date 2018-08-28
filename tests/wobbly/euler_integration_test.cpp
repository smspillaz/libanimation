/*
 * tests/wobbly/euler_integration_test.cpp
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
#include <sstream>                      // for basic_stringbuf<>::int_type, etc
#include <type_traits>                  // for move

#include <cmath>                        // for pow

#include <gmock/gmock-matchers.h>       // for ElementsAreArray, etc
#include <gmock/gmock.h>                // IWYU pragma: keep
#include <gtest/gtest.h>                // for AssertHelper, TEST_F, etc

#include <animation/wobbly/wobbly.h>    // for PointView, Point, Vector
#include <animation/wobbly/wobbly_internal.h>            // for EulerIntegrate

#include <mathematical_model_matcher.h>  // for SatisfiesModel, Linear, etc

using ::testing::Test;

using ::animation::matchers::Eq;
using ::animation::matchers::SatisfiesModel;
using ::animation::matchers::WithSamples;
using ::animation::matchers::WithTolerance;

using ::animation::models::Linear;
using ::animation::models::Parabolic;

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

    class EulerIntegration :
        public Test
    {
        public:

            EulerIntegration () :
                view (storage)
            {
            }

            typedef animation::PointView <double const> DCPV;

            SingleObjectStorage storage;
            SingleObjectStorageView view;
    };

    TEST_F (EulerIntegration, ContinueStepWhenObjectsHaveVelocity)
    {
        agd::set <0> (view.velocity, 1.0);

        /* Integrate without any friction */
        EXPECT_TRUE (wobbly::EulerIntegrate (1,
                                             0,
                                             1,
                                             std::move (view.position),
                                             std::move (view.velocity),
                                             DCPV (view.force)));
    }

    TEST_F (EulerIntegration, NoFurtherStepWhenObjectsHaveNoVelocity)
    {
        agd::set <0> (view.velocity, 0.0);

        /* Integrate without any friction */
        EXPECT_FALSE (wobbly::EulerIntegrate (1,
                                              0,
                                              1,
                                              std::move (view.position),
                                              std::move (view.velocity),
                                              DCPV (view.force)));
    }

    TEST_F (EulerIntegration, VelocityIsParabolicWithFriction)
    {
        std::function <double (int)> horizontalVelocityFunction =
            [this](int timestep) -> double {
                agd::assign (view.position, animation::Point (1, 1));
                agd::assign (view.velocity, animation::Point (1, 0));

                wobbly::EulerIntegrate (timestep,
                                        1,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return agd::get <0> (view.velocity);
            };

        EXPECT_THAT (horizontalVelocityFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    TEST_F (EulerIntegration, VelocityIncreasesLinearlyWithConstantForce)
    {
        std::function <double (int)> frictionlessHorizontalVelocityFunction =
            [this](int timestep) -> double {
                /* Reset velocity and force */
                agd::assign (view.velocity, animation::Vector (0, 0));
                agd::assign (view.force, animation::Vector (1.0f, 0));

                wobbly::EulerIntegrate (timestep,
                                        0,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return agd::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionlessHorizontalVelocityFunction,
                     SatisfiesModel (Linear <double> ()));
    }

    TEST_F (EulerIntegration, LinearDecreaseInVelocityWithFriction)
    {
        unsigned int nSamples = 10;
        int          range = std::pow (2, nSamples);

        std::function <double (int)> frictionToVelocityFunction =
            [this, range](int frictionAmount) -> double {
                /* Reset velocity and force */
                agd::assign (view.position, animation::Point (0, 0));
                agd::assign (view.velocity, animation::Vector (0, 0));
                agd::assign (view.force, animation::Vector (1.0f, 0));

                double frictionProportion =
                    frictionAmount / static_cast <double> (range);

                /* Step once, but with different frictions, linearly
                 * interpolate between frictionAmount and nSamples
                 * and scale by the default friction value to get
                 * our samples */
                wobbly::EulerIntegrate (16,
                                        frictionProportion,
                                        1.0,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return agd::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionToVelocityFunction,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples)));
    }
}
