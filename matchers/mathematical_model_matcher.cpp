/*
 * matchers/mathematical_model_matcher.cpp
 *
 * Copyright 2018 Endless Mobile, Inc.
 *
 * libwobbly is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 2.1 of the
 * License, or (at your option) any later version.
 *
 * libwobbly is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with eos-companion-app-service.  If not, see
 * <http://www.gnu.org/licenses/>.
 *
 * Provides utilities to match functions producing single floating
 * point output values for single integer input values to arbitrary
 * mathematical models, for instance, asserting that a function
 * produces values in a linear sequence, or an exponential sequence
 * with a certain (low) error tolerance.
 */
#include "mathematical_model_matcher.h"
