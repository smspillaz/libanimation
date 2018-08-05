/*
 * animation-glib/zoom/zoom.cpp
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
 * GObject Interface for "wobbly" textures, Anchor type.
 *
 * An anchor is an object type privately holding an
 * anchor. The owner can release the anchor, which happens
 * implicitly when its ref-count drops to zero or when
 * the release() method is called.
 *
 * GObject base class for affine transform based animations.
 */

#include <animation/zoom/zoom.h>

#include <animation-glib/box.h>
#include <animation-glib/zoom/zoom.h>
#include <animation-glib/constructor-helpers.h>
#include <animation-glib/vector.h>

namespace agd = animation::geometry::dimension;
namespace az = animation::zoom;

struct _AnimationZoomAnimation
{
  GObject parent_instance;
};

typedef struct _AnimationZoomAnimationPrivate
{
} AnimationZoomAnimationPrivate;

G_DEFINE_TYPE_WITH_PRIVATE (AnimationZoomAnimation,
                            animation_zoom_animation,
                            ANIMATION_TYPE_TRANSFORM_ANIMATION)

enum {
  PROP_0,
  PROP_FROM,
  PROP_TO,
  PROP_LENGTH,
  NPROPS
};

static GParamSpec *animation_zoom_animation_props [NPROPS] = { NULL, };

static void
animation_zoom_animation_set_property (GObject      *object,
                                       guint         prop_id,
                                       const GValue *value,
                                       GParamSpec   *pspec)
{
  switch (prop_id)
    {
    case PROP_FROM:
      break;
    case PROP_TO:
      break;
    case PROP_LENGTH:
      break;
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static void
animation_zoom_animation_get_property (GObject    *object,
                                       guint       prop_id,
                                       GValue     *value,
                                       GParamSpec *pspec)
{
  switch (prop_id)
    {
    default:
      G_OBJECT_WARN_INVALID_PROPERTY_ID (object, prop_id, pspec);
    }
}

static GObject *
animation_zoom_animation_constructor (GType                  type,
                                      unsigned int           n_construct_params,
                                      GObjectConstructParam *construct_params)
{
  const char * const wanted_properties[] = {
    "from",
    "to",
    "length",
    NULL
  };
  g_autoptr(GHashTable) properties_table =
    static_hash_table_of_values_for_specs (wanted_properties,
                                           construct_params,
                                           n_construct_params);

  auto *interface =
    InterfaceConstructor <az::ZoomAnimation>::construct (
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "from"),
      ForwardFromValueHT (properties_table, animation_box_from_gvalue, "to"),
      ForwardFromValueHT (properties_table, g_value_get_uint, "length")
    );

  unsigned int n_extended_construct_params;
  GObjectConstructParam *extended_construct_params =
    append_interface_prop_to_construct_params (construct_params,
                                               n_construct_params,
                                               G_OBJECT_CLASS (animation_zoom_animation_parent_class),
                                               g_steal_pointer (&interface),
                                               &n_extended_construct_params);

  return G_OBJECT_CLASS (animation_zoom_animation_parent_class)->constructor (type,
                                                                              n_extended_construct_params,
                                                                              extended_construct_params);
}

static void
animation_zoom_animation_init (AnimationZoomAnimation *model)
{
}


static void
animation_zoom_animation_class_init (AnimationZoomAnimationClass *klass)
{
  GObjectClass *object_class = G_OBJECT_CLASS (klass);

  object_class->constructor = animation_zoom_animation_constructor;
  object_class->get_property = animation_zoom_animation_get_property;
  object_class->set_property = animation_zoom_animation_set_property;

  animation_zoom_animation_props[PROP_FROM] =
    g_param_spec_pointer ("from",
                          "From Box",
                          "Box that we are animating from",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  animation_zoom_animation_props[PROP_TO] =
    g_param_spec_pointer ("to",
                          "To Box",
                          "Box that we are animating to",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  animation_zoom_animation_props[PROP_LENGTH] =
    g_param_spec_pointer ("length",
                          "Length",
                          "How long the animation lasts",
                          static_cast <GParamFlags> (G_PARAM_WRITABLE | G_PARAM_CONSTRUCT));

  g_object_class_install_properties (object_class,
                                     NPROPS,
                                     animation_zoom_animation_props);
}

/**
 * animation_zoom_new:
 * @from: The #AnimationBox that we are animating from.
 * @to: The #AnimationBox that we are animating to.
 * @length: The length of the animation.
 *
 * Returns: (transfer full): A new #AnimationZoomAnimation.
 */
AnimationZoomAnimation *
animation_zoom_new (const AnimationBox *from,
                    const AnimationBox *to,
                    unsigned int        length)
{
  return ANIMATION_ZOOM_ANIMATION (g_object_new (ANIMATION_TYPE_ZOOM_ANIMATION,
                                                 "from", from,
                                                 "to", to,
                                                 "length", length,
                                                 NULL));
}
