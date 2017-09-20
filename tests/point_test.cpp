/*
 * tests/wobbly_test.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <array>                        // for array
#include <ostream>                      // for operator<<, char_traits, etc

#include <stddef.h>                     // for size_t

#include <gtest/gtest.h>                // for AssertHelper, EXPECT_EQ, etc
#include <gtest/gtest-param-test.h>     // for Values, Combine, etc

#include <wobbly/wobbly.h>    // for PointView

using ::testing::Test;
using ::testing::Values;
using ::testing::WithParamInterface;

namespace bg = boost::geometry;

namespace
{
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
        bg::set <0> (pv, 1);

        EXPECT_EQ (array[arrayOffset], 1);
    }

    TEST_P (DoublePointView, WriteYWithOffset)
    {
        wobbly::PointView <double> pv (array, pointOffset);
        bg::set <1> (pv, 1);

        EXPECT_EQ (array[arrayOffset + 1], 1);
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

        EXPECT_TRUE (bg::equals (mutpv, constpv));
    }
}
