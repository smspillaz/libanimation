/*
 * include/boost_geometry.h
 *
 * Workarounds for boost::geometry
 *
 * Implicitly depends on:
 *  - boost::geometry
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_BOOST_GEOMETRY_H
#define WOBBLY_BOOST_GEOMETRY_H

// IWYU pragma: begin_exports
#include <boost/geometry/algorithms/assign.hpp>  // for assign
/* boost/geometry/strategies/cartesian/distance_pythagoras depends on
 * boost/numeric/conversion/cast for boost::numeric_cast but does not
 * include it. See Boost bug #10177 */
#include <boost/numeric/conversion/cast.hpp>
#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>  // for add_point, etc
#include <boost/geometry/core/access.hpp>  // for get, set
#include <boost/geometry/core/coordinate_type.hpp>  // for coordinate_type
#include <boost/geometry/core/tag.hpp>  // for traits
#include <boost/geometry/core/tags.hpp>  // for geometry
#include <boost/geometry/util/for_each_coordinate.hpp>
// IWYU pragma: end_exports

#include "wobbly_concept_assert.h"

namespace boost
{
    namespace geometry
    {
        namespace concept
        {
            template <typename Geometry> class ConstPoint;
            template <typename Geometry> class Point;
        }

        /* There is a bug in boost where a concept check was used on
         * the second point only, making it impossible to add from
         * a const type
         */
        namespace fixups
        {
            template <typename Point1, typename Point2>
            inline void assign_point (Point1 &p1, Point2 const &p2)
            {
                WOBBLY_CONCEPT_ASSERT ((concept::Point <Point1>));
                WOBBLY_CONCEPT_ASSERT ((concept::ConstPoint <Point2>));

                for_each_coordinate (p1,
                                     detail::point_assignment <Point2> (p2));
            }

            template <typename Point1, typename Point2>
            inline void subtract_point (Point1 &p1, Point2 const &p2)
            {
                WOBBLY_CONCEPT_ASSERT ((concept::Point <Point1>));
                WOBBLY_CONCEPT_ASSERT ((concept::ConstPoint <Point2>));

                for_each_coordinate (p1,
                                     detail::point_operation <Point2,
                                                              std::minus> (p2));
            }

            /* Don't want include-what-you-use to suggest including
             * boost/geometry/algorithms/detail/distance/interface.hpp
             * because its not available in newer boost versions. Just
             * call through our wrapper here where IWYU is not run */
            template <typename G1, typename G2>
            inline typename default_distance_result <G1, G2>::type
            distance (G1 const &g1, G2 const &g2)
            {
                return boost::geometry::distance (g1, g2);
            }
        }
    }
}

#endif
