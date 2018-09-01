/*
 * animation/property.h
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
 * Properties that can be get and set, without a bunch of boilerplate.
 */

#pragma once

#define ANIMATION_DECLARE_READONLY_PROPERTY(klass, name, type) \
    type const & name () const ;

#define ANIMATION_DEFINE_READONLY_PROPERTY(qualifier, name, type, location) \
    type const & qualifier::name() const \
    { \
        return location; \
    }

#define ANIMATION_DECLARE_PROPERTY(klass, name, type) \
    ANIMATION_DECLARE_READONLY_PROPERTY(klass, name, type) \
    klass & name (type const &v);

#define ANIMATION_DEFINE_PROPERTY(qualifier, name, type, location) \
    ANIMATION_DEFINE_READONLY_PROPERTY(qualifier, name, type, location) \
    qualifier & qualifier::name(type const &v) \
    { \
        location = v; \
        return *this; \
    }

