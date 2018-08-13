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
  for (const char * const *iter = wanted_properties; *iter != NULL; ++iter)
    g_hash_table_insert (ht,
                         (gpointer) (*iter),
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

/**
 * replace_construct_param:
 * @construct_params: (array length=n_construct_params) An array of #GObjectConstructParam
 * @n_construct_params: Number of elements in @construct_params.
 * @prop_name: The name of the construct prop to replace the value of.
 * @initialize_func: A function which sets the GValue to something sensible.
 *
 * Replace a construction parameter @prop_name in the
 * passed @construct_params by using the passed @initialize_func.
 *
 * This function must always replace one construct parameter, it is
 * an error to pass a @prop_name that is not in the @construct_params.
 */
void
replace_construct_param (GObjectConstructParam                          *construct_params,
                         unsigned int                                    n_construct_params,
                         const char                                     *prop_name,
                         AnimationConstructorHelpersInitializeValueFunc  initialize_func,
                         gpointer                                        initialize_func_data)
{
  /* The prop should always be found in the array so that we can replace
   * it, this function doesn't support appending the prop. That means
   * that the relevant prop must always G_PARAM_CONSTRUCT or
   * G_PARAM_CONSTRUCT_ONLY. */
  for (unsigned int i = 0; i < n_construct_params; ++i)
    {
      if (g_strcmp0 (construct_params[i].pspec->name, prop_name) == 0)
        {
          g_value_unset (construct_params[i].value);
          initialize_func (construct_params[i].value, initialize_func_data);
          return;
        }
    }

  g_assert_not_reached ();
}

template <typename FunctionType, typename ...Args>
static typename std::result_of <FunctionType(Args...)>::type
invoke_function_thunk (Args... args, gpointer lambda)
{
  FunctionType *f = reinterpret_cast <FunctionType *> (lambda);

  return (*f)(args...);
}


void
replace_named_pointer_prop_in_construct_params (GObjectConstructParam *construct_params,
                                                unsigned int           n_construct_params,
                                                const char            *prop_name,
                                                gpointer               ptr)
{
  auto set_value = [ptr](GValue *value) {
    g_value_init (value, G_TYPE_POINTER);
    g_value_set_pointer (value, ptr);
  };
  replace_construct_param (construct_params,
                           n_construct_params,
                           prop_name,
                           (AnimationConstructorHelpersInitializeValueFunc) invoke_function_thunk <decltype (set_value), GValue *>,
                           &set_value);
}

void
replace_interface_prop_in_construct_params (GObjectConstructParam *construct_params,
                                            unsigned int           n_construct_params,
                                            gpointer               interface)
{
  replace_named_pointer_prop_in_construct_params (construct_params,
                                                  n_construct_params,
                                                  "interface",
                                                  interface);
}

void
replace_named_pointer_prop_in_construct_params_if_null (GObjectConstructParam                                *construct_params,
                                                        unsigned int                                          n_construct_params,
                                                        const char                                           *prop_name,
                                                        AnimationConstructorHelpersGValueGetPointerFunc       get_func,
                                                        AnimationConstructorHelpersGValueSetPointerFunc       set_func,
                                                        AnimationConstructorHelpersConstructDefaultValueFunc  construct_func)
{
  /* The prop should always be found in the array so that we can replace
   * it, this function doesn't support appending the prop. That means
   * that the relevant prop must always G_PARAM_CONSTRUCT or
   * G_PARAM_CONSTRUCT_ONLY. */
  for (unsigned int i = 0; i < n_construct_params; ++i)
    {
      if (g_strcmp0 (construct_params[i].pspec->name, prop_name) == 0)
        {
          if (get_func (construct_params[i].value) == nullptr)
            set_func (construct_params[i].value, construct_func ());

          return;
        }
    }

  g_assert_not_reached ();
}
