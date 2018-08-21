/*
 * animation-glib/vector4d-internal.h
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
 * GObject boxed type 4D vector implementation.
 */

#include <animation/geometry_traits.h>

namespace animation
{
    namespace geometry
    {
        namespace dimension
        {
            template <>
            struct Dimension <AnimationVector4D>
            {
                typedef double data_type;
                static const size_t dimensions = 4;
            };

            template <>
            struct DimensionAccess <AnimationVector4D, 0>
            {
                static inline double get (AnimationVector4D const &p)
                {
                    return p.x;
                }

                static inline void
                set (AnimationVector4D &p, double const &value)
                {
                    p.x = value;
                }
            };

            template <>
            struct DimensionAccess <AnimationVector4D, 1>
            {
                static inline double get (AnimationVector4D const &p)
                {
                    return p.y;
                }

                static inline void
                set (AnimationVector4D &p, double const &value)
                {
                    p.y = value;
                }
            };

            template <>
            struct DimensionAccess <AnimationVector4D, 2>
            {
                static inline double get (AnimationVector4D const &p)
                {
                    return p.z;
                }

                static inline void
                set (AnimationVector4D &p, double const &value)
                {
                    p.z = value;
                }
            };

            template <>
            struct DimensionAccess <AnimationVector4D, 3>
            {
                static inline double get (AnimationVector4D const &p)
                {
                    return p.w;
                }

                static inline void
                set (AnimationVector4D &p, double const &value)
                {
                    p.w = value;
                }
            };
        }
    }
}
