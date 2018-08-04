/*
 * tests/wobbly_test.cpp
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
#include <ostream>                      // for operator<<, char_traits, etc

#include <stddef.h>                     // for size_t

#include <gtest/gtest.h>                // for AssertHelper, EXPECT_EQ, etc
#include <gtest/gtest-param-test.h>     // for Values, Combine, etc

#include <animation/wobbly/wobbly.h>    // for PointView

using ::testing::Test;
using ::testing::Values;
using ::testing::WithParamInterface;

namespace
{
    namespace wgd = wobbly::geometry::dimension;

    class DoublePointView :
        public Test,
        public WithParamInterface <size_t>
    {
        public:

            DoublePointView () :
                pointOffset (GetParam ()),
                arrayOffset (GetParam () * 2)
            {
                array.fill (0);
            }

            std::array <double, 4> array;
            size_t                 pointOffset;
            size_t                 arrayOffset;
    };

    TEST_P (DoublePointView, WriteXWithOffset)
    {
        wobbly::PointView <double> pv (array, pointOffset);
        wgd::set <0> (pv, 1.0);

        EXPECT_EQ (array[arrayOffset], 1.0);
    }

    TEST_P (DoublePointView, WriteYWithOffset)
    {
        wobbly::PointView <double> pv (array, pointOffset);
        wgd::set <1> (pv, 1.0);

        EXPECT_EQ (array[arrayOffset + 1], 1.0);
    }

    INSTANTIATE_TEST_CASE_P (DuplexMeshArray, DoublePointView,
                             Values (0, 1));

    TEST (DoublePointView, MutableToConstConversionOperatorCopiesSafely)
    {
        std::array <double, 8> array = {
                                           { 1, 2, 3, 4, 5, 6, 7, 8 }
                                       };
        wobbly::PointView <double> mutpv (array, 1);
        wobbly::PointView <double const> constpv (mutpv);

        EXPECT_TRUE (wgd::equals (mutpv, constpv));
    }
}
