/*
 * src/emscripten.cpp
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
 * "wobbly" textures in C++, emscripten bindings.
 */

#include <emscripten/bind.h>
#include <wobbly/wobbly.h>

namespace em = emscripten;

static void SetSpringConstant (double springK)
{
    wobbly::Model::DefaultSettings.springConstant = springK;
}

static void SetFriction (double friction)
{
    wobbly::Model::DefaultSettings.friction = friction;
}

static void SetMaximumRange (double maximumRange)
{
    wobbly::Model::DefaultSettings.maximumRange = maximumRange;
}

EMSCRIPTEN_BINDINGS (wobbly)
{
    em::function ("SetSpringConstant", &SetSpringConstant);
    em::function ("SetFriction", &SetFriction);
    em::function ("SetMaximumRange", &SetMaximumRange);

    em::value_object <wobbly::Point> ("WobblyPoint")
        .field ("x", &wobbly::Point::x)
        .field ("y", &wobbly::Point::y);

    em::value_object <wobbly::Model::Settings> ("WobblyModelSettings")
        .field ("springConstant", &wobbly::Model::Settings::springConstant)
        .field ("friction", &wobbly::Model::Settings::friction)
        .field ("maximumRange", &wobbly::Model::Settings::maximumRange);

    em::class_ <wobbly::Anchor> ("WobblyAnchor")
        .function ("MoveBy",
                   &wobbly::Anchor::MoveBy);

    em::class_ <wobbly::Model> ("WobblyModel")
        .constructor <wobbly::Point,
                      float,
                      float> ()
        .function ("MoveModelBy", &wobbly::Model::MoveModelBy)
        .function ("MoveModelTo", &wobbly::Model::MoveModelTo)
        .function ("Step", &wobbly::Model::Step)
        .function ("DeformTexcoords", &wobbly::Model::DeformTexcoords)
        .function ("InsertAnchor", &wobbly::Model::InsertAnchor);
}
