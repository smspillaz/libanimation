/*
 * src/emscripten.cpp
 *
 * "wobbly" textures in C++, emscripten bindings.
 *
 * See LICENCE.md for Copyright information
 */

#include <emscripten/bind.h>
#include <smspillaz/wobbly/wobbly.h>

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
