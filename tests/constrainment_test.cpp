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

#include <ostream_point_operator.h>

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
                wobbly::PointView <double> pv (positions,
                                               i * meshWidth + j);
                bg::assign_point (pv, wobbly::Point (tileWidth * j,
                                                     tileHeight * i));
            }
        }
    }

    constexpr double TextureWidth = 50.0f;
    constexpr double TextureHeight = 100.0f;
    wobbly::Point const TextureCenter = wobbly::Point (TextureWidth / 2,
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
        wobbly::PointView <double> pv (positions, 1);
        bg::add_point (pv, wobbly::Point (range * 2, range * 2));

        EXPECT_TRUE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ReturnsFalseWhereNoConstrainmentTookPlace)
    {
        auto handleOwner (targets.Activate ());
        wobbly::PointView <double> pv (positions, 1);

        /* Not enough to cause constrainment */
        bg::add_point (pv, wobbly::Point (range / 2, 0));

        EXPECT_FALSE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ConstrainedToPointsOnTargetMesh)
    {
        wobbly::Point const movement (range * 2, 0);
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
            wobbly::PointView <double> pv (expectedPositions, i);
            bg::add_point (pv, wobbly::Point (range, 0));
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

        wobbly::PointView <double> pv (positions, index);
        wobbly::Point expected;

        bg::add_point (pv,
                       wobbly::Point (radiusInRange * ratio,
                                      radiusInRange * (1 - absratio) * sign));

        /* Expected point is the modified point here, before constrainment */
        bg::assign (expected, pv);

        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    TEST_P (ConstrainmentStepPositions, AffectedWhereOutsideRadiusOfRange)
    {
        double absratio = std::fabs (ratio);

        /* Prevent zero-division. Treat zero as positive */
        double const sign = ratio / (absratio + (absratio == 0.0));
        double const radiusOutOfRange = range * range;

        wobbly::Point outOfRange (radiusOutOfRange * ratio,
                                  radiusOutOfRange * (1 - absratio) * sign);
        wobbly::Point inRange (range * ratio,
                               range * (1 - absratio) * sign);

        wobbly::PointView <double> pv (positions, index);
        wobbly::Point expected;

        /* Expected point is the actual grid point, but at its maximum range */
        bg::assign (expected, pv);
        bg::add_point (expected, inRange);

        bg::add_point (pv, outOfRange);
        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
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
