/*
 * matchers/mathematical_model_matcher.cpp
 *
 * Provides utilities to match functions producing single floating
 * point output values for single integer input values to arbitrary
 * mathematical models, for instance, asserting that a function
 * produces values in a linear sequence, or an exponential sequence
 * with a certain (low) error tolerance.
 *
 * See LICENCE.md for Copyright information
 */
#include <gmock/gmock.h>

#include "mathematical_model_matcher.h"
