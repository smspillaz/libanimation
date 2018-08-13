/*
 * animation-glib/constructor-helpers.cpp
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
 * Helpers for constructing C++ objects directly from GObject properties
 * in a GObject constructor.
 */

#pragma once

#include <glib.h>
#include <glib-object.h>

#include <array>
#include <type_traits>
#include <utility>

#include <animation/geometry.h>
#include <animation-glib/box.h>

G_BEGIN_DECLS

inline GValue *
lookup_gvalue (GHashTable *ht, const char *key)
{
  return reinterpret_cast <GValue *> (g_hash_table_lookup (ht, key));
}

GHashTable *
static_hash_table_of_values_for_specs (const char * const    *wanted_properties,
                                       GObjectConstructParam *construct_params,
                                       unsigned int           n_construct_params);

GObjectConstructParam *
append_construct_param_copy (GObjectConstructParam *construct_params,
                             unsigned int           n_construct_params,
                             GParamSpec            *append_pspec,
                             GValue                *append_value,
                             unsigned int          *out_extended_n_construct_params);

GObjectConstructParam *
append_interface_prop_to_construct_params (GObjectConstructParam *construct_params,
                                           unsigned int           n_construct_params,
                                           GObjectClass          *klass,
                                           gpointer               interface,
                                           unsigned int          *out_extended_n_construct_params);

G_END_DECLS

#ifdef __cplusplus
inline animation::Box <animation::Point>
animation_box_from_gvalue (GValue *value)
{
  AnimationBox target_box =
    *(reinterpret_cast <AnimationBox *> (g_value_get_boxed (value)));
  return animation::Box <animation::Point> (animation::Point (target_box.top_left.x,
                                                              target_box.top_left.y),
                                            animation::Point (target_box.bottom_right.x,
                                                              target_box.bottom_right.y));
}

template <typename Marshaller>
typename std::result_of <Marshaller(GValue *)>::type ForwardFromValueHT (GHashTable  *ht,
                                                                         Marshaller &&m,
                                                                         const char  *name)
{
  return m (lookup_gvalue (ht, name));
}

template <class Interface>
struct InterfaceConstructor
{
    template <typename... ArgTypes>
    static Interface * construct (ArgTypes&&... args)
    {
        return new Interface (args...);
    }
};
#endif
