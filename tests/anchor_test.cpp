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
#include <cstddef>                      // for size_t
#include <functional>                   // for bind, __bind, _1

#include <gmock/gmock-cardinalities.h>  // for AtLeast
#include <gmock/gmock-generated-function-mockers.h>  // for FunctionMocker, etc
#include <gmock/gmock-matchers.h>       // for AnythingMatcher, etc
#include <gmock/gmock-spec-builders.h>  // for EXPECT_CALL, etc
#include <gtest/gtest.h>                // for TEST_F, Test, Types, etc

#include <wobbly/wobbly_internal.h>            // for TrackedAnchors

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
