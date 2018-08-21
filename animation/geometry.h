/*
 * animation/wobbly/geometry.h
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
 * Provides geometric types and trait overloads
 * for 2D points and boxes.
 */
#pragma once

#include <animation/geometry_traits.h>

namespace animation
{
    namespace geometry
    {
        namespace detail
        {
            struct CopyablePV
            {
                public:

                    CopyablePV (CopyablePV const &copy) = default;
                    CopyablePV & operator= (CopyablePV const &copy) = default;
            };

            struct NoncopyablePV
            {
                public:

                    NoncopyablePV (NoncopyablePV const &copy) = delete;
                    NoncopyablePV & operator= (NoncopyablePV const &copy) = delete;
            };

            template <typename NumericType>
            class PointViewCopyabilityBase
            {
                public:

                    typedef NumericType NT;
                    typedef std::is_const <NumericType> IC;
                    typedef std::conditional <IC::value,
                                              CopyablePV,
                                              NoncopyablePV> type;
            };
        }

        /**
         * PointView<NumericType>:
         *
         * A view of a single point into an array of two dimensional
         * NumericTypes.
         *
         * This allows us to use "structure of arrays" and exploit cache
         * locality when updating lots of forces, velocities or positions
         * at the same time.
         *
         * PointView implements the Dimension trait, meaning that it
         * can be used with functions in the animation::geometry::dimension
         * namespace.
         */ 
        template <typename NumericType>
        class PointView :
            public detail::PointViewCopyabilityBase <NumericType>::type
        {
            public:

                typedef NumericType NT;
                typedef typename std::remove_const <NumericType>::type NTM;
                typedef typename std::add_const <NumericType>::type NTC;

                typedef typename std::is_const <NT> IC;

                template <std::size_t N>
                PointView (std::array <NTM, N> &points,
                           std::size_t         index) :
                    array (points.data ()),
                    offset (index * 2)
                {
                }

                template <std::size_t N,
                          typename = typename std::enable_if <IC::value && N>::type>
                PointView (std::array <NTM, N> const &points,
                           std::size_t               index) :
                    array (points.data ()),
                    offset (index * 2)
                {
                }

                PointView (NumericType *points,
                           std::size_t index) :
                    array (points),
                    offset (index * 2)
                {
                }

                PointView (PointView <NumericType> &&view) :
                    array (view.array),
                    offset (view.offset)
                {
                }

                PointView &
                operator= (PointView <NumericType> &&view) noexcept
                {
                    if (this == &view)
                        return *this;

                    array = view.array;
                    offset = view.offset;

                    return *this;
                }

                /* Copy constructor and assignment operator enabled only if the view
                 * is truly just observing and has no write access to the point,
                 * see detail::PointViewCopyabilityBase */
                PointView (PointView <NumericType> const &p) :
                    array (p.array),
                    offset (p.offset)
                {
                }

                friend void swap (PointView <NumericType> &first,
                                  PointView <NumericType> &second)
                {
                    std::swap (first.array, second.array);
                    std::swap (first.offset, second.offset);
                }

                PointView &
                operator= (PointView <NumericType> const &p)
                {
                    PointView <NumericType> copy (p);
                    swap (*this, copy);
                    return *this;
                }

                /* cppcheck thinks this is a constructor, so we need to suppress
                 * a constructor warning here.
                 */
                // cppcheck-suppress uninitMemberVar
                operator PointView <NTC> ()
                {
                    /* The stored offset is an array offset, but the constructor
                     * requires a point offset. Divide by two */
                    return PointView <NTC> (array, offset / 2);
                }

                template <size_t Offset>
                NumericType const & get () const
                {
                    static_assert (Offset < 2, "Offset must be < 2");
                    return array[offset + Offset];
                }

                template <size_t Offset>
                typename std::enable_if <!(IC::value) && (Offset + 1), void>::type
                set (NumericType value)
                {
                    static_assert (Offset < 2, "Offset must be < 2");
                    array[offset + Offset] = value;
                }

            private:

                NumericType *array;
                size_t      offset;
        };

        namespace dimension
        {
            /* Overloads for animation::PointView */
            template <typename U>
            struct Dimension <PointView <U> >
            {
                typedef double data_type;
                static const size_t dimensions = 2;
            };

            template <typename T, size_t D>
            struct DimensionAccess <PointView <T>, D>
            {
                static inline T get (PointView <T> const &p)
                {
                    return p.template get <D> ();
                }

                static inline void
                set (PointView <T> &p, T const &value)
                {
                    p.template set <D> (value);
                }
            };
        }

        /**
         * PointModel<T>:
         *
         * A detached 2D point or vector in space for a given data
         * type T. This is a structure of two values.
         *
         * PointModel implements the Dimension trait, meaning that it
         * can be used with functions in the animation::geometry::dimension
         * namespace.
         */
        template <typename T>
        struct PointModel
        {
            PointModel (T x, T y) noexcept :
                x (x),
                y (y)
            {
            }

            PointModel () noexcept :
                x (0),
                y (0)
            {
            }

            PointModel (PointModel const &p) noexcept :
                x (p.x),
                y (p.y)
            {
            }

            void swap (PointModel &a, PointModel &b) noexcept
            {
                std::swap (a.x, b.x);
                std::swap (a.y, b.y);
            }

            PointModel & operator= (PointModel other) noexcept
            {
                swap (*this, other);

                return *this;
            }

            T x;
            T y;
        };

        typedef PointModel <double> Point;
        typedef Point Vector;

        namespace dimension
        {
            template <typename T>
            struct Dimension <PointModel <T> >
            {
                typedef T data_type;
                static const size_t dimensions = 2;
            };

            template <typename T>
            struct DimensionAccess <PointModel <T>, 0>
            {
                static inline T get (PointModel <T> const &p)
                {
                    return p.x;
                }

                static inline void
                set (PointModel <T> &p, T const &value)
                {
                    p.x = value;
                }
            };

            template <typename T>
            struct DimensionAccess <PointModel <T>, 1>
            {
                static inline T get (PointModel <T> const &p)
                {
                    return p.y;
                }

                static inline void
                set (PointModel <T> &p, T const &value)
                {
                    p.y = value;
                }
            };
        }

        /**
         * Box<T>
         *
         * A box of two points of type T, specifying the
         * top left and bottom right corners of a box.
         *
         * The contains() method tells the user if a
         * 2D point satisfying the Dimension trait is
         * inclusively contained within the box bounds.
         */
        template <typename T>
        class Box
        {
            public:

                Box (T const &tl, T const &br) :
                    tl (tl),
                    br (br)
                {
                }

                template <typename P>
                bool contains (P const &p) const;

                template <typename V>
                void apply_visitor (V &&visitor) const
                {
                    visitor (tl);
                    visitor (br);
                }

                template <typename V>
                void apply_visitor (V &&visitor)
                {
                    (const_cast <Box const &> (*this))->apply_visitor (visitor);
                }

                T const & topLeft () const
                {
                    return tl;
                }

                T const & bottomRight () const
                {
                    return br;
                }

            private:

                T tl;
                T br;
        };

        template <typename T>
        template <typename U>
        bool Box <T>::contains (U const &p) const
        {
            auto const x1 = dimension::get <0> (tl);
            auto const x2 = dimension::get <0> (br);
            auto const y1 = dimension::get <1> (tl);
            auto const y2 = dimension::get <1> (br);

            return (dimension::get <0> (p) >= x1 &&
                    dimension::get <0> (p) <= x2 &&
                    dimension::get <1> (p) >= y1 &&
                    dimension::get <1> (p) <= y2);
        }

        /**
         * Vector4DModel<T>:
         *
         * A detached 2D point or vector in space for a given data
         * type T. This is a structure of two values.
         *
         * PointModel implements the Dimension trait, meaning that it
         * can be used with functions in the animation::geometry::dimension
         * namespace.
         */
        template <typename T>
        struct Vector4DModel
        {
            Vector4DModel (T x, T y, T z, T w) noexcept :
                x (x),
                y (y),
                z (z),
                w (w)
            {
            }

            Vector4DModel () noexcept :
                x (0),
                y (0),
                z (0),
                w (0)
            {
            }

            Vector4DModel (Vector4DModel const &v) noexcept :
                x (v.x),
                y (v.y),
                z (v.z),
                w (v.w)
            {
            }

            void swap (Vector4DModel &a, Vector4DModel &b) noexcept
            {
                std::swap (a.x, b.x);
                std::swap (a.y, b.y);
                std::swap (a.z, b.w);
                std::swap (a.w, b.z);
            }

            Vector4DModel & operator= (Vector4DModel other) noexcept
            {
                swap (*this, other);

                return *this;
            }

            T x;
            T y;
            T z;
            T w;
        };

        typedef Vector4DModel <double> Vector4D;

        namespace dimension
        {
            template <typename T>
            struct Dimension <Vector4DModel <T> >
            {
                typedef T data_type;
                static const size_t dimensions = 4;
            };

            template <typename T>
            struct DimensionAccess <Vector4DModel <T>, 0>
            {
                static inline T get (Vector4DModel <T> const &p)
                {
                    return p.x;
                }

                static inline void
                set (Vector4DModel <T> &p, T const &value)
                {
                    p.x = value;
                }
            };

            template <typename T>
            struct DimensionAccess <Vector4DModel <T>, 1>
            {
                static inline T get (Vector4DModel <T> const &p)
                {
                    return p.y;
                }

                static inline void
                set (Vector4DModel <T> &p, T const &value)
                {
                    p.y = value;
                }
            };

            template <typename T>
            struct DimensionAccess <Vector4DModel <T>, 2>
            {
                static inline T get (Vector4DModel <T> const &p)
                {
                    return p.z;
                }

                static inline void
                set (Vector4DModel <T> &p, T const &value)
                {
                    p.z = value;
                }
            };

            template <typename T>
            struct DimensionAccess <Vector4DModel <T>, 3>
            {
                static inline T get (Vector4DModel <T> const &p)
                {
                    return p.w;
                }

                static inline void
                set (Vector4DModel <T> &p, T const &value)
                {
                    p.w = value;
                }
            };
        }
    }

    /* Import animation::geometry::Point types into
     * animation namespace for compatibility. */
    typedef animation::geometry::Point Point;
    typedef animation::geometry::Vector Vector;
    typedef animation::geometry::Vector4D Vector4D;

    template <typename NumericType>
    using PointView = animation::geometry::PointView <NumericType>;

    template <typename NumericType>
    using PointModel = animation::geometry::PointModel <NumericType>;

    template <typename PointType>
    using Box = animation::geometry::Box <PointType>;
}
