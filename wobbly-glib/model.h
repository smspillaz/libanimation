/*
 * wobbly-glib/model.h
 *
 * GObject Interface for "wobbly" textures
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_GLIB_MODEL_H
#define WOBBLY_GLIB_MODEL_H

#include <glib-object.h>

#include <wobbly-glib/anchor.h>
#include <wobbly-glib/vector.h>

G_BEGIN_DECLS

#define WOBBLY_TYPE_MODEL wobbly_model_get_type ()
G_DECLARE_FINAL_TYPE (WobblyModel, wobbly_model, WOBBLY, MODEL, GObject)

WobblyModel * wobbly_model_new (WobblyVector  position,
                                WobblyVector  size,
                                double        spring_constant,
                                double        friction,
                                double        maximum_range);

WobblyAnchor * wobbly_model_grab_anchor (WobblyModel *model,
                                         WobblyVector position);

WobblyAnchor * wobbly_model_insert_anchor (WobblyModel *model,
                                           WobblyVector position);

gboolean wobbly_model_step (WobblyModel  *model,
                            unsigned int  ms);

void wobbly_model_deform_texcoords (WobblyModel  *model,
                                    WobblyVector  uv,
                                    WobblyVector *deformed);

void wobbly_model_query_extremes (WobblyModel  *model,
                                  WobblyVector *top_left,
                                  WobblyVector *top_right,
                                  WobblyVector *bottom_left,
                                  WobblyVector *bottom_right);

void wobbly_model_move_to (WobblyModel *model,
                           WobblyVector position);
void wobbly_model_move_by (WobblyModel *model,
                           WobblyVector delta);

void wobbly_model_resize (WobblyModel *model,
                          WobblyVector size);

void wobbly_model_set_spring_k (WobblyModel *model, double spring_constant);

void wobbly_model_set_friction (WobblyModel *model, double friction);

void wobbly_model_set_maximum_range (WobblyModel *model, double range);

G_END_DECLS

#endif
