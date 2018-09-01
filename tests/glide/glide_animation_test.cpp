/*
 * tests/wobbly/glide_animation_test.cpp
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
 * Tests for the "glide" animation.
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
#include <animation/glide/glide.h>
#include <animation/stepper/linear.h>

#include <glm_ostream_operators.h>
#include <mathematical_model_matcher.h>

using ::animation::matchers::AlmostEq;
using ::testing::_;
using ::testing::AtLeast;
using ::testing::Eq;
using ::testing::Not;
using ::testing::Test;

namespace agd = animation::geometry::dimension;
namespace ag = animation::glide;
namespace as = animation::stepper;

namespace animation
{
    namespace matchers
    {
        template <>
        struct AlmostEqTrait <glm::vec4>
        {
            typedef float ValueType;

            template <class Comparator>
            static bool apply (glm::vec4 const &lhs,
                               glm::vec4 const &rhs,
                               Comparator      &&comparator)
            {
                return comparator (lhs[0], rhs[0]) &&
                       comparator (lhs[1], rhs[1]) &&
                       comparator (lhs[2], rhs[2]) &&
                       comparator (lhs[3], rhs[3]);
            }
        };
    }
}

namespace
{
    static const unsigned int MockScreenWidth = 1000;

    TEST (GlideAnimation, AnimationIncompleteBeforeLengthTimesteps)
    {
        ag::GlideAnimation anim (0.5f,
                                 0.0f,
                                 20.0f,
                                 0.5f,
                                 0.0f,
                                 1000,
                                 animation::Box <animation::Point> (animation::Point (0, 0),
                                                                    animation::Point (100, 100)),
                                 as::Linear (200));

        EXPECT_TRUE (anim.Step (199));
    }

    TEST (GlideAnimation, AnimationCompleteAtLengthTimesteps)
    {
        ag::GlideAnimation anim (0.5f,
                                 0.0f,
                                 20.0f,
                                 0.5f,
                                 0.0f,
                                 1000,
                                 animation::Box <animation::Point> (animation::Point (0, 0),
                                                                    animation::Point (100, 100)),
                                 as::Linear (200));

        EXPECT_FALSE (anim.Step (200));
    }

    TEST (GlideAnimation, AnimationStopsAtTargetBox)
    {
        auto target = animation::Box <animation::Point> (animation::Point (100, 100),
                                                         animation::Point (200, 200));
        ag::GlideAnimation anim (0.5f,
                                 0.0f,
                                 20.0f,
                                 0.5f,
                                 0.0f,
                                 1000,
                                 target,
                                 as::Linear (200));

        anim.Step (200);

        /* Apply to a shape at (0, 0) which is translated to (100, 100) */
        glm::mat4 translation (glm::translate (glm::mat4 (1.0), glm::vec3 (100, 100, 0)));
        glm::vec4 tl (translation * glm::vec4 (0, 0, 0, 1));
        glm::vec4 br (translation * glm::vec4 (100, 100, 0, 1));

        /* Apply transformation matrix to vectors */
        glm::mat4 transformation (glm::make_mat4x4 (anim.Matrix ()));

        /* Should finish at target box */
        EXPECT_THAT (transformation * tl,
                     AlmostEq (glm::vec4 (agd::get <0> (target.topLeft ()),
                                          agd::get <1> (target.topLeft ()),
                                          0,
                                          1),
                               0.002));
        EXPECT_THAT (transformation * br,
                     AlmostEq (glm::vec4 (agd::get <0> (target.bottomRight ()),
                                          agd::get <1> (target.bottomRight ()),
                                          0,
                                          1),
                               0.002));
    }

    TEST (GlideAnimation, AnimationDoesNotStartAtTargetBox)
    {
        auto target = animation::Box <animation::Point> (animation::Point (100, 100),
                                                         animation::Point (200, 200));
        ag::GlideAnimation anim (0.5f,
                                 0.0f,
                                 20.0f,
                                 0.5f,
                                 0.0f,
                                 1000,
                                 target,
                                 as::Linear (200));

        /* Apply to a shape at (0, 0) which is translated to (100, 100) */
        glm::mat4 translation (glm::translate (glm::mat4 (1.0), glm::vec3 (100, 100, 0)));
        glm::vec4 tl (translation * glm::vec4 (0, 0, 0, 1));
        glm::vec4 br (translation * glm::vec4 (100, 100, 0, 1));

        /* Apply transformation matrix to vectors */
        glm::mat4 transformation (glm::make_mat4x4 (anim.Matrix ()));

        /* Should finish at target box */
        EXPECT_THAT (transformation * tl,
                     Not (Eq (glm::vec4 (agd::get <0> (target.topLeft ()),
                                         agd::get <1> (target.topLeft ()),
                                         0,
                                         1))));
        EXPECT_THAT (transformation * br,
                     Not (Eq (glm::vec4 (agd::get <0> (target.bottomRight ()),
                                         agd::get <1> (target.bottomRight ()),
                                         0,
                                         1))));
    }
}
