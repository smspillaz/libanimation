/*
 * tests/wobbly/glib_api_test.cpp
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
 * Tests for the "wobbly" spring model, GObject API.
 */

#include <iomanip>                      // for operator<<, setprecision
#include <iosfwd>                       // for ostream
#include <ostream>                      // for basic_ostream, char_traits, etc

#include <animation-glib/wobbly/anchor.h>
#include <animation-glib/wobbly/model.h>
#include <animation-glib/wobbly/vector.h>

#include <mathematical_model_matcher.h>  // for Eq, EqDispatchHelper, etc
#include <within_geometry_matcher.h>     // for WithinGeometry, etc

#include <gmock/gmock-cardinalities.h>  // for AtLeast
#include <gmock/gmock-generated-function-mockers.h>  // for FunctionMocker, etc
#include <gmock/gmock-matchers.h>       // for AnythingMatcher, etc
#include <gmock/gmock-spec-builders.h>  // for EXPECT_CALL, etc
#include <gtest/gtest.h>                // for TEST_F, Test, Types, etc

#include "ostream_point_operator.h"      // for operator<<, etc

using ::testing::ElementsAreArray;
using ::testing::Eq;
using ::testing::Matcher;
using ::testing::Not;
using ::testing::Test;

using ::wobbly::matchers::WithinGeometry;

bool operator== (AnimationVector const &lhs,
                 AnimationVector const &rhs)
{
  return lhs.x == rhs.x && lhs.y == rhs.y;
}

inline std::ostream &
operator<< (std::ostream &lhs, AnimationVector const &v)
{
    return lhs << std::setprecision (10)
               << "x: "
               << v.x
               << " y: "
               << v.y;
}

namespace wobbly
{
    namespace geometry
    {
        namespace dimension
        {
            template <>
            struct Dimension <AnimationVector>
            {
                typedef double data_type;
                static const size_t dimensions = 2;
            };

            template <>
            struct DimensionAccess <AnimationVector, 0>
            {
                static inline double get (AnimationVector const &p)
                {
                    return p.x;
                }

                static inline void
                set (AnimationVector &p, double const &value)
                {
                    p.x = value;
                }
            };

            template <>
            struct DimensionAccess <AnimationVector, 1>
            {
                static inline double get (AnimationVector const &p)
                {
                    return p.y;
                }

                static inline void
                set (AnimationVector &p, double const &value)
                {
                    p.y = value;
                }
            };
        }
    }
}

namespace
{
    namespace detail
    {
        template <typename T, typename U, typename Visitor, size_t N, size_t I>
        struct ArrayMapInto
        {
            static void apply (std::array <T, N>       &dst,
                               std::array <U, N> const &src,
                               Visitor                 &&visitor)
            {
                dst[I] = visitor (src[I]);
                ArrayMapInto <T, U, Visitor, N, I - 1>::apply (dst,
                                                               src,
                                                               std::forward <Visitor> (visitor));
            }
        };

        template <typename T, typename U, typename Visitor, size_t N>
        struct ArrayMapInto <T, U, Visitor, N, 0>
        {
            static void apply (std::array <T, N>       &dst,
                               std::array <U, N> const &src,
                               Visitor                 &&visitor)
            {
                dst[0] = visitor (src[0]);
            }
        };
    }

    template <typename T, typename U, typename Visitor, size_t N>
    void array_map_into (std::array <T, N>       &dst,
                         std::array <U, N> const &src,
                         Visitor                 &&visitor)
    {
        detail::ArrayMapInto <T, U, Visitor, N, N - 1>::apply (dst,
                                                               src,
                                                               std::forward <Visitor> (visitor));
    }

    typedef wobbly::Box <wobbly::Point> PointBox;

    TEST (WobblyGLibAPI, ConstructModel)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
    }

    TEST (WobblyGLibAPI, MoveModelChangesExtremes)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);

        std::array <AnimationVector, 4> extremes;

        animation_wobbly_model_move_to (model, { 1.0, 1.0 });
        animation_wobbly_model_query_extremes (model,
                                               &extremes[0],
                                               &extremes[1],
                                               &extremes[2],
                                               &extremes[3]);

        std::array <AnimationVector, 4> expected = {
            {
                { 1.0, 1.0 },
                { 101.0, 1.0 },
                { 1.0, 101.0 },
                { 101.0, 101.0 }
            }
        };
        std::array <Matcher <AnimationVector const &>, 4> textureEdges;
        array_map_into (textureEdges,
                        expected,
                        [](auto const &vector) -> decltype(auto) {
                            return Eq (vector);
                        });

        EXPECT_THAT (extremes, ElementsAreArray(textureEdges)); 
    }

    TEST (WobblyGLibAPI, ResizeModelChangesExtremes)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);

        std::array <AnimationVector, 4> extremes;

        animation_wobbly_model_resize (model, { 200.0, 200.0 });
        animation_wobbly_model_query_extremes (model,
                                               &extremes[0],
                                               &extremes[1],
                                               &extremes[2],
                                               &extremes[3]);

        std::array <AnimationVector, 4> expected = {
            {
                { 0.0, 0.0 },
                { 200.0, 0.0 },
                { 0.0, 200.0 },
                { 200.0, 200.0 }
            }
        };
        std::array <Matcher <AnimationVector const &>, 4> textureEdges;
        array_map_into (textureEdges,
                        expected,
                        [](auto const &vector) -> decltype(auto) {
                            return Eq (vector);
                        });

        EXPECT_THAT (extremes, ElementsAreArray(textureEdges)); 
    }

    TEST (WobblyGLibAPI, GrabCorrectIndex)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_grab_anchor (model, { 100.0, 0.0 });

        animation_wobbly_anchor_move_by (anchor, { 10.0, 10.0 });

        AnimationVector topRightExtreme;
        AnimationVector expectedTopRightExtreme = { 110.0, 0.0 };

        animation_wobbly_model_query_extremes (model,
                                               nullptr,
                                               &topRightExtreme,
                                               nullptr,
                                               nullptr);

        EXPECT_THAT (topRightExtreme, Eq (expectedTopRightExtreme));
    }

    TEST (WobblyGLibAPI, ModelSettlesAfterMovingAnchor)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_grab_anchor (model, { 100.0, 0.0 });

        /* Move anchor and settle */
        animation_wobbly_anchor_move_by (anchor, { 10.0, 10.0 });
        while (animation_wobbly_model_step (model, 1));

        std::array <AnimationVector, 4> extremes;
        animation_wobbly_model_query_extremes (model,
                                               &extremes[0],
                                               &extremes[1],
                                               &extremes[2],
                                               &extremes[3]);

        std::array <AnimationVector, 4> expected = {
            {
                { 10.0, 10.0 },
                { 110.0, 10.0 },
                { 10.0, 110.0 },
                { 110.0, 110.0 }
            }
        };
        std::array <Matcher <AnimationVector const &>, 4> textureEdges;
        array_map_into (textureEdges,
                        expected,
                        [](auto const &vector) -> decltype(auto) {
                            return Eq (vector);
                        });

        EXPECT_THAT (extremes, ElementsAreArray(textureEdges)); 
    }

    TEST (WobblyGLibAPI, DeformedWithGrabbedAnchor)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_grab_anchor (model, { 100.0, 0.0 });
        AnimationVector deformed;
        AnimationVector center = { 50.0, 50.0 };

        animation_wobbly_anchor_move_by (anchor, { 10.0, 10.0 });
        animation_wobbly_model_deform_texcoords (model, { 0.5, 0.5 }, &deformed);

        EXPECT_THAT (deformed, Not (Eq (center)));
    }

    TEST (WobblyGLibAPI, DeformedWithInsertAnchor)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_insert_anchor (model, { 70.0, 0.0 });
        AnimationVector deformed;
        AnimationVector center = { 50.0, 50.0 };

        animation_wobbly_anchor_move_by (anchor, { 10.0, 10.0 });
        animation_wobbly_model_step (model, 1);
        animation_wobbly_model_deform_texcoords (model, { 0.5, 0.5 }, &deformed);

        EXPECT_THAT (deformed, Not (Eq (center)));
    }

    TEST (WobblyGLibAPI, NoDeformationNoAnchorMove)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_grab_anchor (model, { 100.0, 0.0 });
        AnimationVector deformed;
        AnimationVector center = { 50.0, 50.0 };

        animation_wobbly_model_deform_texcoords (model, { 0.5, 0.5 }, &deformed);

        EXPECT_THAT (deformed, Eq (center));
    }

    TEST (WobblyGLibAPI, ReleaseAnchorOnModel)
    {
        g_autoptr(AnimationWobblyModel) model = animation_wobbly_model_new ({ 0.0, 0.0 },
                                                                            { 100.0, 100.0 },
                                                                            8.0,
                                                                            5.0,
                                                                            500.0);
        g_autoptr(AnimationWobblyAnchor) anchor = animation_wobbly_model_grab_anchor (model, { 100.0, 0.0 });

        /* Temporarily grab another anchor and move the first one */
        {
            g_autoptr(AnimationWobblyAnchor) otherAnchor = animation_wobbly_model_grab_anchor (model, { 0.0, 0.0 });
            animation_wobbly_anchor_move_by (anchor, { 10.0, 10.0 });
        }

        /* Anchor is now released. Settle model */
        while (animation_wobbly_model_step (model, 1));

        std::array <AnimationVector, 4> extremes;
        animation_wobbly_model_query_extremes (model,
                                               &extremes[0],
                                               &extremes[1],
                                               &extremes[2],
                                               &extremes[3]);

        std::array <AnimationVector, 4> expected = {
            {
                { 10.0, 10.0 },
                { 110.0, 10.0 },
                { 10.0, 110.0 },
                { 110.0, 110.0 }
            }
        };
        std::array <Matcher <AnimationVector const &>, 4> textureEdges;
        array_map_into (textureEdges,
                        expected,
                        [](auto const &vector) -> decltype(auto) {
                            return WithinGeometry (PointBox (wobbly::Point (vector.x - 3.0,
                                                                            vector.y - 3.0),
                                                             wobbly::Point (vector.x + 3.0,
                                                                            vector.y + 3.0)));
                        });

        EXPECT_THAT (extremes, ElementsAreArray(textureEdges)); 
    }
}
