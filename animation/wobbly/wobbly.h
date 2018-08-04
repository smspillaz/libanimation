/*
 * animation/wobbly/wobbly.h
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
 * C++ Interface for "wobbly" textures
 *
 * Implicitly depends on:
 *  - std::array
 */
#ifndef WOBBLY_H
#define WOBBLY_H

#include <cstddef>

#include <array>                        // for array, swap
#include <memory>
#include <stdexcept>                    // for runtime_error
#include <type_traits>
#include <vector>

#include <animation/geometry.h>
#include <animation/geometry_traits.h>

/* std::swap is explicitly specialized for numerous other types we don't care
 * about */
// IWYU pragma: no_include <__split_buffer>
// IWYU pragma: no_include <__bit_reference>
// IWYU pragma: no_include <__mutex_base>
// IWYU pragma: no_include <__tree>
// IWYU pragma: no_include <deque>
// IWYU pragma: no_include <functional>
// IWYU pragma: no_include <list>
// IWYU pragma: no_include <map>
// IWYU pragma: no_include <set>
// IWYU pragma: no_include <sstream>
// IWYU pragma: no_include <stack>
// IWYU pragma: no_include <string>
// IWYU pragma: no_include <tuple>
// IWYU pragma: no_include <utility>
// IWYU pragma: no_forward_declare wobbly::Anchor::MovableAnchor
// IWYU pragma: no_forward_declare wobbly::Model::Private

namespace wobbly
{
    /* Import wobbly::geometry::Point types into
     * wobbly namespace for compatibility. */
    typedef animation::geometry::Point Point;
    typedef animation::geometry::Vector Vector;

    template <typename NumericType>
    using PointView = animation::geometry::PointView <NumericType>;

    template <typename NumericType>
    using PointModel = animation::geometry::PointModel <NumericType>;

    template <typename PointType>
    using Box = animation::geometry::Box <PointType>;

    class Anchor
    {
        public:

            Anchor ();
            Anchor (Anchor &&) noexcept = default;
            Anchor & operator= (Anchor &&) noexcept = default;
            ~Anchor ();

            void MoveBy (Vector const &delta) noexcept;

            class MovableAnchor;
            struct MovableAnchorDeleter
            {
                void operator () (MovableAnchor *);
            };

            typedef std::unique_ptr <MovableAnchor, MovableAnchorDeleter> Impl;

            static Anchor Create (Impl &&imp);

        protected:

            Anchor (Anchor const &) = delete;
            Anchor & operator= (Anchor const &) = delete;

            Impl priv;
    };

    class Model
    {
        public:

            struct Settings
            {
                double springConstant;
                double friction;
                double maximumRange;
            };

            Model (Point const &initialPosition,
                   double width,
                   double height,
                   Settings const &settings);
            Model (Point const &initialPosition,
                   double width,
                   double height);
            Model (Model const &other);
            ~Model ();

            /* This function will cause a point on the spring mesh closest
             * to grab in absolute terms to become immobile in the mesh.
             *
             * The resulting point can be moved freely by the returned
             * object. Integrating the model after the point has been moved
             * will effectively cause force to be exerted on all the other
             * points. */
            wobbly::Anchor
            GrabAnchor (Point const &grab) noexcept (false);

            /* This function will insert a new point in the spring
             * mesh which is immobile, with springs from it to its
             * two nearest neighbours.
             *
             * The resulting point can be moved freely by the returned object.
             * As above, integrating the model after moving the point will
             * effectively cause force to be exerted on all other non-immobile
             * points in the mesh */
            wobbly::Anchor
            InsertAnchor (Point const &grab) noexcept (false);

            /* Performs a single integration per 16 ms in millisecondsDelta */
            bool Step (unsigned int millisecondsDelta);

            /* Takes a normalized texture co-ordinate from 0 to 1 and returns
             * an absolute-position on-screen for that texture co-ordinate
             * as deformed by the model */
            Point DeformTexcoords (Point const &normalized) const;

            /* Bounding box for the model */
            std::array <Point, 4> const Extremes () const;

            /* These functions will attempt to move and resize
             * the model relative to its target position, however,
             * caution should be exercised when using them.
             *
             * A full integration until the model has reached equillibrium
             * may need to be performed in order to determine the target
             * position and given the nature of the calculations, there may
             * be some error in determining that target position. That may
             * affect the result of these operations to a slight degree.
             *
             * Moving and resizing a model will also move any attached anchors
             * relative to the change in the model's mesh. You might want to
             * compensate for this by applying movementto those anchors
             * if they are to stay in the same position.
             *
             * If a precise position is required, then the recommended course
             * of action is to destroy and re-create the model. */
            void MoveModelTo (Point const &point);
            void MoveModelBy (Point const &delta);
            void ResizeModel (double width, double height);

            static constexpr double DefaultSpringConstant = 8.0;
            static constexpr double DefaultObjectRange = 500.0f;
            static constexpr double Mass = 15.0f;
            static constexpr double Friction = 3.0f;

            static Settings DefaultSettings;

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };
}
#endif
