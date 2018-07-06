/*
 * matchers/within_geometry_matcher.h
 *
 * Provides utilities to match types satisfying the
 * Dimension trait to ensure they are within
 * a PointBox.
 *
 * See LICENCE.md for Copyright information
 */

#ifndef WOBBLY_WITHIN_GEOMETRY_MATCHER_H
#define WOBBLY_WITHIN_GEOMETRY_MATCHER_H

#include <iomanip>                      // for operator<<, setprecision
#include <ostream>                      // for ostream, operator<<, etc

#include <wobbly/geometry_traits.h>

#include <gmock/gmock.h>       // IWYU pragma: keep

namespace wobbly
{
    namespace matchers
    {
        namespace t = ::testing;
        namespace wgd = ::wobbly::geometry::dimension;

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
                    wgd::for_each_point (parent, [&os](auto const &p) {
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
