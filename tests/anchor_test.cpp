/*
 * tests/wobbly_test.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <cstddef>                      // for size_t
#include <functional>                   // for bind, __bind, _1

#include <gmock/gmock-cardinalities.h>  // for AtLeast
#include <gmock/gmock-generated-function-mockers.h>  // for FunctionMocker, etc
#include <gmock/gmock-matchers.h>       // for AnythingMatcher, etc
#include <gmock/gmock-spec-builders.h>  // for EXPECT_CALL, etc
#include <gtest/gtest.h>                // for TEST_F, Test, Types, etc

#include <wobbly_internal.h>            // for TrackedAnchors

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Test;

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
