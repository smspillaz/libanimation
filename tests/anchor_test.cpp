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
    class TrackedAnchors :
        public Test
    {
        public:

            wobbly::TrackedAnchors <3> anchors;
    };

    class MockAnchorAction
    {
        public:

            MockAnchorAction ()
            {
                EXPECT_CALL (*this, Action (_)).Times (AtLeast (0));
            }

            MOCK_METHOD1 (Action, void (size_t));
    };

    TEST_F (TrackedAnchors, WithFirstGrabbedDisabledWithNoAnchorsLocked)
    {
        using namespace std::placeholders;

        MockAnchorAction action;
        EXPECT_CALL (action, Action (_)).Times (0);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, LockingAnchorEnablesWithFirstGrabbed)
    {
        using namespace std::placeholders;

        size_t lockIndex = 1;

        anchors.Lock (lockIndex);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (lockIndex)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, FirstGrabbedTakesPriorityAsMoreLocked)
    {
        using namespace std::placeholders;

        size_t lockIndex = 1;

        anchors.Lock (lockIndex);
        anchors.Lock (lockIndex + 1);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (lockIndex)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, MoveToNextHeavisestWhereFirstGrabbedUnlocked)
    {
        using namespace std::placeholders;

        anchors.Lock (0);

        for (size_t i = 0; i < 2; ++i)
            anchors.Lock (1);

        for (size_t i = 0; i < 3; ++i)
            anchors.Lock (2);

        anchors.Unlock (0);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (2)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, UnsetWhenAllUnlocked)
    {
        using namespace std::placeholders;

        for (size_t i = 0; i < 3; ++i)
            anchors.Lock (i);

        for (size_t i = 0; i < 3; ++i)
            anchors.Unlock (i);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (_)).Times (0);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }
}
