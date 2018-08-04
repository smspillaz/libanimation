/*
 * animation/wobbly/wobbly_concept_assert.h
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
 * Workarounds for -Wunused-local-typedef warnings on BOOST_CONCEPT_ASSERT
 *
 * Implicitly depends on:
 *  - boost::concept
 */
#ifndef WOBBLY_CONCEPT_ASSERT_H
#define WOBBLY_CONCEPT_ASSERT_H

/* Work around compiler warnings when using BOOST_CONCEPT_ASSERT - reimplement
 * BOOST_CONCEPT_ASSERT ourselves and add __attribute__ (unused) to specify
 * that the typedef will be unsued */
#define WOBBLY_CONCEPT_ASSERT_FN( ModelFnPtr ) \
    typedef ::boost::concepts::detail::instantiate < \
        &::boost::concepts::requirement_<ModelFnPtr>::failed> \
    BOOST_PP_CAT(boost_concept_check,__LINE__) \
    __attribute__((unused))

#define WOBBLY_CONCEPT_ASSERT(ModelInParens) \
    WOBBLY_CONCEPT_ASSERT_FN(void(*)ModelInParens)

#endif
