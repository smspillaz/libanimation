/*
 * tests/glm_ostream_operators.h
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
 * A simple helper to output the contents of glm types when
 * tests fail.
 */
#pragma once

#include <glm/glm.hpp>
#include <iomanip>                      // for operator<<, setprecision
#include <iosfwd>                       // for ostream
#include <ostream>                      // for basic_ostream, char_traits, etc

namespace
{
    namespace detail
    {
        template <typename T, int D>
        struct ArrayDimensionPrinter
        {
            static std::ostream & Apply (std::ostream &lhs,
                                         T      const &array)
            {
                ArrayDimensionPrinter <T, D - 1>::Apply (lhs, array);
                return lhs << ", " << array[D];
            }
        };

        template <typename T>
        struct ArrayDimensionPrinter <T, 0>
        {
            static std::ostream & Apply (std::ostream &lhs,
                                         T      const &array)
            {
                return lhs << array[0];
            }
        };
    }
}

namespace glm
{
    inline std::ostream & operator<< (std::ostream    &lhs,
                                      glm::vec4 const &vector)
    {
        lhs << "vec4(";
        ::detail::ArrayDimensionPrinter <glm::vec4, 3>::Apply (lhs, vector);
        return lhs << ")";
    }
}
