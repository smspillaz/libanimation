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

#include <glib.h>

#include "constructor-helpers.h"

GHashTable *
static_hash_table_of_values_for_specs (const char * const    *wanted_properties,
                                       GObjectConstructParam *construct_params,
                                       unsigned int           n_construct_params)
{
  g_autoptr(GHashTable) ht = g_hash_table_new (g_str_hash, g_str_equal);

  /* Insert all the properties we intend to
   * have in the hash table, with NULL as
   * their value. */
  for (const char * const *property_iter = wanted_properties;
       *property_iter != NULL;
       ++property_iter)
    g_hash_table_insert (ht,
                         (gpointer) (*property_iter),
                         NULL);

  /* Now go through all the construct params and insert
   * their value pointer into the hash table. */
  for (unsigned int i = 0; i < n_construct_params; ++i)
    {
      GParamSpec *pspec = construct_params[i].pspec;
      GValue     *value = construct_params[i].value;

      if (g_hash_table_contains (ht, pspec->name))
        g_hash_table_insert (ht,
                             (gpointer) pspec->name,
                             value);
    }

  return reinterpret_cast <GHashTable *> (g_steal_pointer (&ht));
}

GObjectConstructParam *
append_construct_param_copy (GObjectConstructParam *construct_params,
                             unsigned int           n_construct_params,
                             GParamSpec            *append_pspec,
                             GValue                *append_value,
                             unsigned int          *out_extended_n_construct_params)
{
  g_assert (out_extended_n_construct_params != NULL);

  unsigned int extended_n_construct_params = n_construct_params + 1;
  GObjectConstructParam *extended_construct_params = g_new0 (GObjectConstructParam,
                                                             extended_n_construct_params);
  unsigned int i = 0;

  for (; i < n_construct_params; ++i)
    {
      extended_construct_params[i].pspec = construct_params[i].pspec;
      extended_construct_params[i].value = construct_params[i].value;
    }

  extended_construct_params[i].pspec = append_pspec;
  extended_construct_params[i].value = append_value;

  return extended_construct_params;
}

GObjectConstructParam *
append_interface_prop_to_construct_params (GObjectConstructParam *construct_params,
                                           unsigned int           n_construct_params,
                                           GObjectClass          *klass,
                                           gpointer               interface,
                                           unsigned int          *out_extended_n_construct_params)
{
  g_assert (out_extended_n_construct_params != NULL);

  GValue interface_value = G_VALUE_INIT;
  g_value_init (&interface_value, G_TYPE_POINTER);

  return append_construct_param_copy (construct_params,
                                      n_construct_params,
                                      g_object_class_find_property (klass, "interface"),
                                      &interface_value,
                                      out_extended_n_construct_params);
}
