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

#include <iosfwd>

#include <smspillaz/wobbly/wobbly.h>

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
