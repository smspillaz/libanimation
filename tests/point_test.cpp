/*
 * tests/wobbly_test.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <functional>
#include <gmock/gmock.h>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/algorithms/for_each.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <mathematical_model_matcher.h>

#include <smspillaz/wobbly/wobbly.h>
#include <wobbly_internal.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Combine;
using ::testing::ElementsAreArray;
using ::testing::ExitedWithCode;
using ::testing::Invoke;
using ::testing::MakeMatcher;
using ::testing::MakePolymorphicMatcher;
using ::testing::Matcher;
using ::testing::MatcherCast;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;
using ::testing::Not;
using ::testing::PolymorphicMatcher;
using ::testing::Test;
using ::testing::Types;
using ::testing::Values;
using ::testing::ValuesIn;
using ::testing::WithParamInterface;

using ::wobbly::matchers::Eq;
using ::wobbly::matchers::SatisfiesModel;
using ::wobbly::matchers::WithSamples;
using ::wobbly::matchers::WithTolerance;

using ::wobbly::models::ExponentialDecayTowards;
using ::wobbly::models::Linear;
using ::wobbly::models::Parabolic;

namespace bg = boost::geometry;

namespace boost
{
    namespace geometry
    {
        namespace model
        {
            template <typename C, std::size_t D, typename S>
            std::ostream &
            operator<< (std::ostream &lhs, point <C, D, S> const &p)
            {
                return lhs << std::setprecision (10)
                           << "x: "
                           << bg::get <0> (p)
                           << " y: "
                           << bg::get <1> (p);
            }
        }
    }
}

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
}
