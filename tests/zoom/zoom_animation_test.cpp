/*
 * tests/wobbly/zoom_animation_test.cpp
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
#include <cstddef>                      // for size_t
#include <functional>                   // for bind, __bind, _1

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <gmock/gmock-cardinalities.h>  // for AtLeast
#include <gmock/gmock-generated-function-mockers.h>  // for FunctionMocker, etc
#include <gmock/gmock-matchers.h>       // for AnythingMatcher, etc
#include <gmock/gmock-spec-builders.h>  // for EXPECT_CALL, etc
#include <gtest/gtest.h>                // for TEST_F, Test, Types, etc

#include <animation/geometry.h>
#include <animation/zoom/zoom.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::Eq;
using ::testing::Test;

namespace az = animation::zoom;

namespace
{
    TEST (ZoomAnimation, AnimationIncompleteBeforeLengthTimesteps)
    {
        az::ZoomAnimation anim (animation::Box <animation::Point> (animation::Point (0, 0),
                                                                   animation::Point (100, 100)),
                                animation::Box <animation::Point> (animation::Point (100, 100),
                                                                   animation::Point (200, 200)),
                                200);

        EXPECT_TRUE (anim.Step (199));
    }

    TEST (ZoomAnimation, AnimationCompleteAfterLengthTimesteps)
    {
        az::ZoomAnimation anim (animation::Box <animation::Point> (animation::Point (0, 0),
                                                                   animation::Point (100, 100)),
                                animation::Box <animation::Point> (animation::Point (100, 100),
                                                                   animation::Point (200, 200)),
                                200);

        EXPECT_FALSE (anim.Step (200));
    }

    TEST (ZoomAnimation, AnimatesToCorrectPositionAfterLengthTimesteps)
    {
        az::ZoomAnimation anim (animation::Box <animation::Point> (animation::Point (0, 0),
                                                                   animation::Point (100, 100)),
                                animation::Box <animation::Point> (animation::Point (100, 100),
                                                                   animation::Point (200, 200)),
                                200);

        anim.Step (200);

        /* Apply to a shape at (0, 0) which is translated to (100, 100) */
        glm::mat4 translation (glm::translate (glm::mat4 (1.0), glm::vec3 (100, 100, 0)));
        glm::vec4 tl (translation * glm::vec4 (0, 0, 0, 1));
        glm::vec4 br (translation * glm::vec4 (100, 100, 0, 1));

        /* Apply transformation matrix to vectors */
        glm::mat4 transformation (glm::make_mat4x4 (anim.Matrix ()));

        /* Should be no change, as animation is complete */
        EXPECT_THAT (transformation * tl, Eq (tl));
        EXPECT_THAT (transformation * br, Eq (br));
    }

    TEST (ZoomAnimation, StartsAtCorrectPositionAtZeroTimesteps)
    {
        az::ZoomAnimation anim (animation::Box <animation::Point> (animation::Point (0, 0),
                                                                   animation::Point (100, 100)),
                                animation::Box <animation::Point> (animation::Point (100, 100),
                                                                   animation::Point (200, 200)),
                                200);

        /* Apply to a shape at (0, 0) which is translated to (100, 100) */
        glm::mat4 translation (glm::translate (glm::mat4 (1.0), glm::vec3 (100, 100, 0)));
        glm::vec4 tl (translation * glm::vec4 (0, 0, 0, 1));
        glm::vec4 br (translation * glm::vec4 (100, 100, 0, 1));

        /* Apply transformation matrix to vectors */
        glm::mat4 transformation (glm::make_mat4x4 (anim.Matrix ()));

        /* Should start at expected box */
        EXPECT_THAT (transformation * tl, Eq (glm::vec4 (0, 0, 0, 1)));
        EXPECT_THAT (transformation * br, Eq (glm::vec4 (100, 100, 0, 1)));
    }

    TEST (ZoomAnimation, AtCorrectPositionAfterHalfwayPoint)
    {
        az::ZoomAnimation anim (animation::Box <animation::Point> (animation::Point (0, 0),
                                                                   animation::Point (100, 100)),
                                animation::Box <animation::Point> (animation::Point (100, 100),
                                                                   animation::Point (200, 200)),
                                200);

        anim.Step (100);

        /* Apply to a shape at (0, 0) which is translated to (100, 100) */
        glm::mat4 translation (glm::translate (glm::mat4 (1.0), glm::vec3 (100, 100, 0)));
        glm::vec4 tl (translation * glm::vec4 (0, 0, 0, 1));
        glm::vec4 br (translation * glm::vec4 (100, 100, 0, 1));

        /* Apply transformation matrix to vectors */
        glm::mat4 transformation (glm::make_mat4x4 (anim.Matrix ()));

        /* Should be about halfway */
        EXPECT_THAT (transformation * tl, Eq (glm::vec4 (50, 50, 0, 1)));
        EXPECT_THAT (transformation * br, Eq (glm::vec4 (150, 150, 0, 1)));
    }
}
