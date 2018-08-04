/*
 * animation-glib/wobbly/model.h
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
 * GObject Interface for "wobbly" textures.
 */
#ifndef WOBBLY_GLIB_MODEL_H
#define WOBBLY_GLIB_MODEL_H

#include <glib-object.h>

#include <animation-glib/wobbly/anchor.h>
#include <animation-glib/vector.h>

G_BEGIN_DECLS

#define ANIMATION_WOBBLY_TYPE_MODEL animation_wobbly_model_get_type ()
G_DECLARE_FINAL_TYPE (AnimationWobblyModel, animation_wobbly_model, ANIMATION, WOBBLY_MODEL, GObject)

AnimationWobblyModel * animation_wobbly_model_new (AnimationVector  position,
                                                   AnimationVector  size,
                                                   double        spring_constant,
                                                   double        friction,
                                                   double        maximum_range);

AnimationWobblyAnchor * animation_wobbly_model_grab_anchor (AnimationWobblyModel *model,
                                                            AnimationVector position);

AnimationWobblyAnchor * animation_wobbly_model_insert_anchor (AnimationWobblyModel *model,
                                                              AnimationVector position);

gboolean animation_wobbly_model_step (AnimationWobblyModel  *model,
                                      unsigned int  ms);

void animation_wobbly_model_deform_texcoords (AnimationWobblyModel  *model,
                                              AnimationVector  uv,
                                              AnimationVector *deformed);

void animation_wobbly_model_query_extremes (AnimationWobblyModel  *model,
                                            AnimationVector *top_left,
                                            AnimationVector *top_right,
                                            AnimationVector *bottom_left,
                                            AnimationVector *bottom_right);

void animation_wobbly_model_move_to (AnimationWobblyModel *model,
                                     AnimationVector position);
void animation_wobbly_model_move_by (AnimationWobblyModel *model,
                                     AnimationVector delta);

void animation_wobbly_model_resize (AnimationWobblyModel *model,
                                    AnimationVector size);

void animation_wobbly_model_set_spring_k (AnimationWobblyModel *model, double spring_constant);

void animation_wobbly_model_set_friction (AnimationWobblyModel *model, double friction);

void animation_wobbly_model_set_maximum_range (AnimationWobblyModel *model, double range);

G_END_DECLS

#endif
