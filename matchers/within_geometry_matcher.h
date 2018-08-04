/*
 * matchers/within_geometry_matcher.h
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
 * Provides utilities to match types satisfying the
 * Dimension trait to ensure they are within
 * a PointBox.
 */

#ifndef WOBBLY_WITHIN_GEOMETRY_MATCHER_H
#define WOBBLY_WITHIN_GEOMETRY_MATCHER_H

#include <iomanip>                      // for operator<<, setprecision
#include <ostream>                      // for ostream, operator<<, etc

#include <animation/geometry_traits.h>

#include <gmock/gmock.h>       // IWYU pragma: keep

namespace wobbly
{
    namespace matchers
    {
        namespace t = ::testing;
        namespace agd = ::animation::geometry::dimension;

        template <typename ParentGeometry>
        class WithinGeometryMatcher
        {
            public:

                WithinGeometryMatcher (ParentGeometry const &parent) :
                    parent (parent)
                {
                }

                template <typename ChildGeometry>
                bool MatchAndExplain (ChildGeometry const    &child,
                                      t::MatchResultListener *listener) const
                {
                    return parent.contains (child);
                }

                void DescribeTo (std::ostream *os) const
                {
                    *os << "is";
                    Describe (*os);
                }

                void DescribeNegationTo (std::ostream *os) const
                {
                    *os << "is not";
                    Describe (*os);
                }

            private:

                void Describe (std::ostream &os) const
                {
                    os << " within :" << std::endl;
                    agd::for_each_point (parent, [&os](auto const &p) {
                        os << " - " << p << std::endl;
                    });
                }

                ParentGeometry parent;
        };

        template <typename ParentGeometry>
        inline t::PolymorphicMatcher <WithinGeometryMatcher <ParentGeometry> >
        WithinGeometry (ParentGeometry const &parent)
        {
            WithinGeometryMatcher <ParentGeometry> matcher (parent);
            return t::MakePolymorphicMatcher (matcher);
        }
    }
}

#endif
