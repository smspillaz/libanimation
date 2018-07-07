/*
 * tests/ostream_point_operator.h
 *
 * A simple helper to output the contents of wobbly::Point and
 * wobbly::PointView when tests fail.
 *
 * See LICENCE.md for Copyright information.
 */
#ifndef WOBBLY_TESTS_OSTREAM_POINT_OPERATOR_H
#define WOBBLY_TESTS_OSTREAM_POINT_OPERATOR_H

#include <wobbly/wobbly.h>    // for Point
#include <iomanip>                      // for operator<<, setprecision
#include <iosfwd>                       // for ostream
#include <ostream>                      // for basic_ostream, char_traits, etc

namespace wobbly
{
    namespace geometry
    {
        inline std::ostream &
        operator<< (std::ostream &lhs, Point const &p)
        {
            namespace wgd = wobbly::geometry::dimension;

            return lhs << std::setprecision (10)
                       << "x: "
                       << wgd::get <0> (p)
                       << " y: "
                       << wgd::get <1> (p);
        }

        template <typename NumericType>
        inline std::ostream &
        operator<< (std::ostream                                    &lhs,
                    wobbly::geometry::PointView <NumericType> const &p)
        {
            namespace wgd = wobbly::geometry::dimension;

            Point point;
            wgd::assign (point, p);
            return lhs << point;
        }
    }
}
#endif
