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
#include <boost/geometry/algorithms/assign.hpp>  // for assign
#include <boost/geometry/core/access.hpp>  // for get
#include <iomanip>                      // for operator<<, setprecision
#include <iosfwd>                       // for ostream
#include <ostream>                      // for basic_ostream, char_traits, etc

namespace wobbly
{
    inline std::ostream &
    operator<< (std::ostream &lhs, Point const &p)
    {
        return lhs << std::setprecision (10)
                   << "x: "
                   << bg::get <0> (p)
                   << " y: "
                   << bg::get <1> (p);
    }

    template <typename NumericType>
    inline std::ostream &
    operator<< (std::ostream &lhs, PointView <NumericType> const &p)
    {
        Point point;
        bg::assign (point, p);
        return lhs << point;
    }
}
#endif
