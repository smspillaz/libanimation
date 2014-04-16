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

EMSCRIPTEN_BINDINGS (wobbly)
{
    em::value_object <wobbly::Point> ("WobblyPoint")
        .field ("x", &wobbly::Point::x)
        .field ("y", &wobbly::Point::y);

    em::class_ <wobbly::Object::AnchorGrab> ("WobblyAnchor")
        .constructor <wobbly::ImmediatelyMovablePosition &, unsigned int> ()
        .function ("MoveBy", &wobbly::Object::AnchorGrab::MoveBy);

    em::class_ <wobbly::Model> ("WobblyModel")
        .constructor <wobbly::Point, float, float> ()
        .function ("MoveModelBy", &wobbly::Model::MoveModelBy)
        .function ("MoveModelTo", &wobbly::Model::MoveModelTo)
        .function ("StepModel", &wobbly::Model::StepModel)
        .function ("DeformTexcoords", &wobbly::Model::DeformTexcoords)
        .function ("GrabAnchor", &wobbly::Model::GrabAnchor);
}
