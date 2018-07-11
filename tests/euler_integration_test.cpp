/*
 * tests/wobbly_test.cpp
 *
 * Copyright 2018 Endless Mobile, Inc.
 *
 * libwobbly is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * libwobbly is distributed in the hope that it will be useful,
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

#include <wobbly/wobbly.h>    // for PointView, Point, Vector
#include <wobbly/wobbly_internal.h>            // for EulerIntegrate

#include <mathematical_model_matcher.h>  // for SatisfiesModel, Linear, etc

using ::testing::Test;

using ::wobbly::matchers::Eq;
using ::wobbly::matchers::SatisfiesModel;
using ::wobbly::matchers::WithSamples;
using ::wobbly::matchers::WithTolerance;

using ::wobbly::models::Linear;
using ::wobbly::models::Parabolic;

namespace
{
    namespace wgd = wobbly::geometry::dimension;

    class SingleObjectStorage
    {
        public:

            SingleObjectStorage ()
            {
                storage.fill (0);
            }

            wobbly::PointView <double> Position ()
            {
                return wobbly::PointView <double> (storage, 0);
            }

            wobbly::PointView <double> Velocity ()
            {
                return wobbly::PointView <double> (storage, 1);
            }

            wobbly::PointView <double> Force ()
            {
                return wobbly::PointView <double> (storage, 2);
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

            wobbly::PointView <double> position;
            wobbly::PointView <double> velocity;
            wobbly::PointView <double> force;
    };

    class EulerIntegration :
        public Test
    {
        public:

            EulerIntegration () :
                view (storage)
            {
            }

            typedef wobbly::PointView <double const> DCPV;

            SingleObjectStorage storage;
            SingleObjectStorageView view;
    };

    TEST_F (EulerIntegration, ContinueStepWhenObjectsHaveVelocity)
    {
        wgd::set <0> (view.velocity, 1.0);

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
        wgd::set <0> (view.velocity, 0.0);

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
                wgd::assign (view.position, wobbly::Point (1, 1));
                wgd::assign (view.velocity, wobbly::Point (1, 0));

                wobbly::EulerIntegrate (timestep,
                                        1,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return wgd::get <0> (view.velocity);
            };

        EXPECT_THAT (horizontalVelocityFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    TEST_F (EulerIntegration, VelocityIncreasesLinearlyWithConstantForce)
    {
        std::function <double (int)> frictionlessHorizontalVelocityFunction =
            [this](int timestep) -> double {
                /* Reset velocity and force */
                wgd::assign (view.velocity, wobbly::Vector (0, 0));
                wgd::assign (view.force, wobbly::Vector (1.0f, 0));

                wobbly::EulerIntegrate (timestep,
                                        0,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return wgd::get <0> (view.velocity);
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
                wgd::assign (view.position, wobbly::Point (0, 0));
                wgd::assign (view.velocity, wobbly::Vector (0, 0));
                wgd::assign (view.force, wobbly::Vector (1.0f, 0));

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

                return wgd::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionToVelocityFunction,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples)));
    }
}
