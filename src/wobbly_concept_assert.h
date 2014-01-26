/*
 * include/wobbly_concept_assert.h
 *
 * Workarounds for -Wunused-local-typedef warnings on BOOST_CONCEPT_ASSERT
 *
 * Implicitly depends on:
 *  - boost::concept
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_CONCEPT_ASSERT_H
#define WOBBLY_CONCEPT_ASSERT_H

#include <boost/concept/assert.hpp>     // for BOOST_CONCEPT_ASSERT
#include <boost/concept/usage.hpp> 

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
