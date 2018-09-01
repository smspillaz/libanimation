/*
 * animation/glide/glide.cpp
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
 * An animation that causes a surface to glide onto screen, gently
 * following an attenuating sine wave.
 */

#include <algorithm>
#include <array>

#include <cmath>

#include <animation/box_calculation.h>
#include <animation/geometry.h>
#include <animation/glide/glide.h>
#include <animation/math.h>
#include <animation/property.h>
#include <animation/stepper/stepper.h>
#include <animation/transform_calculation.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace agd = animation::geometry::dimension;
namespace abc = animation::box_calculation;
namespace ag = animation::glide;
namespace am = animation::math;
namespace atc = animation::transform_calculation;

namespace
{
    inline float DegreesToRadians (float degrees)
    {
        return (M_PI / 180.0f) * degrees;
    }

    /* This undoes perspective distortion */
    glm::mat4 CreatePerspectiveDistortionMatrix (unsigned int screenWidth)
    {
        float v = -1.0 / screenWidth;

        return glm::mat4 (glm::vec4 (1.0, 0.0, 0.0, 0.0),
                          glm::vec4 (0.0, 1.0, 0.0, 0.0),
                          glm::vec4 (0.0, 0.0, 0.0, 0.0),
                          glm::vec4 (0.0, 0.0, v, 1.0));
    }

    /* This animation rotates the surface from some rotated position
     * towards a resting position where its rotation angle is
     * 0, 0, 0 on all three axes. The axis that the surface can rotate
     * on is configurable in unit-cordinates. Rotation on 0.5, 0.5 rotates
     * at the center of the surface, whereas rotating from 0, 0 would rotate
     * from the top left corner. */
    void ComputeGlideTransform (glm::mat4                               &transform,
                                float                                   progress,
                                float                                   initialDistance,
                                float                                   xRotationAngleDegrees,
                                float                                   yRotationAngleDegrees,
                                float                                   xAxisLocationUnit,
                                float                                   yAxisLocationUnit,
                                unsigned int                            screenWidth,
                                animation::Box <animation::Point> const &targetBox)
    {
        animation::Point rotationAxis (abc::ComputeRotationAxisOffset (targetBox,
                                                                       xAxisLocationUnit,
                                                                       yAxisLocationUnit));

        auto centerMat = glm::translate (glm::mat4 (1.0f),
                                         glm::vec3 (-1.0f * agd::get <0> (rotationAxis),
                                                    -1.0f * agd::get <1> (rotationAxis),
                                                    0.0f));
        auto xRotationMat = glm::rotate (glm::mat4 (1.0f),
                                         DegreesToRadians (xRotationAngleDegrees) * (1.0f - progress),
                                         glm::vec3 (1.0f, 0.0f, 0.0f));
        auto yRotationMat = glm::rotate (glm::mat4 (1.0f),
                                         DegreesToRadians (yRotationAngleDegrees) * (1.0f - progress),
                                         glm::vec3 (0.0f, 1.0f, 0.0f));
        auto translationMat = glm::translate (glm::mat4 (1.0f),
                                              glm::vec3 (0.0f,
                                                         0.0f,
                                                         -1.0f * initialDistance * (1.0f - progress)));
        auto invCenterMat = glm::translate (glm::mat4 (1.0f),
                                            glm::vec3 (agd::get <0> (rotationAxis),
                                                       agd::get <1> (rotationAxis),
                                                       0.0f));

        transform = invCenterMat *
                    translationMat *
                    xRotationMat *
                    yRotationMat *
                    CreatePerspectiveDistortionMatrix (screenWidth) *
                    centerMat;
    }
}

namespace animation
{
    namespace glide
    {
        struct GlideAnimation::Private
        {
            Private (float                                   initialDistance,
                     float                                   xRotationAngleDegrees,
                     float                                   yRotationAngleDegrees,
                     float                                   xAxisLocationUnit,
                     float                                   yAxisLocationUnit,
                     unsigned int                            screenWidth,
                     animation::Box <animation::Point> const &target,
                     animation::stepper::Stepper       const &stepper);

            float initialDistance;
            float xRotationAngleDegrees;
            float yRotationAngleDegrees;
            float xAxisLocationUnit;
            float yAxisLocationUnit;
            unsigned int screenWidth;
            animation::Box <animation::Point> target;

            glm::mat4 transform;
            float progress;

            animation::stepper::Stepper stepper;
        };

        GlideAnimation::Private::Private (float                                   initialDistance,
                                          float                                   xRotationAngleDegrees,
                                          float                                   yRotationAngleDegrees,
                                          float                                   xAxisLocationUnit,
                                          float                                   yAxisLocationUnit,
                                          unsigned int                            screenWidth,
                                          animation::Box <animation::Point> const &target,
                                          animation::stepper::Stepper       const &stepper) :
            initialDistance (initialDistance),
            xRotationAngleDegrees (xRotationAngleDegrees),
            yRotationAngleDegrees (yRotationAngleDegrees),
            xAxisLocationUnit (xAxisLocationUnit),
            yAxisLocationUnit (yAxisLocationUnit),
            screenWidth (screenWidth),
            target (target),
            transform (glm::mat4 (1.0)),
            progress (stepper (0)),
            stepper (stepper)
        {
            ComputeGlideTransform (transform,
                                   progress,
                                   initialDistance,
                                   xRotationAngleDegrees,
                                   yRotationAngleDegrees,
                                   xAxisLocationUnit,
                                   yAxisLocationUnit,
                                   screenWidth,
                                   target);
        }
    }
}

std::array <animation::Vector4D, 4>
ag::GlideAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Vector4D, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

float
ag::GlideAnimation::Progress () const
{
    return priv->progress;
}

bool
ag::GlideAnimation::Step (unsigned int ms)
{
    priv->progress = am::clamp (priv->stepper (ms), 0.0f, 1.0f);

    ComputeGlideTransform (priv->transform,
                           priv->progress,
                           priv->initialDistance,
                           priv->xRotationAngleDegrees,
                           priv->yRotationAngleDegrees,
                           priv->xAxisLocationUnit,
                           priv->yAxisLocationUnit,
                           priv->screenWidth,
                           priv->target);

    return priv->progress != 0.0f && priv->progress != 1.0f;
}

float * const
ag::GlideAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, InitialDistance, float, priv->initialDistance)
ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, XRotationAngleDegrees, float, priv->xRotationAngleDegrees)
ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, YRotationAngleDegrees, float, priv->yRotationAngleDegrees)
ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, XAxisLocationUnit, float, priv->xAxisLocationUnit)
ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, YAxisLocationUnit, float, priv->yAxisLocationUnit)
ANIMATION_DEFINE_PROPERTY (ag::GlideAnimation, Stepper, animation::stepper::Stepper, priv->stepper)

ag::GlideAnimation::GlideAnimation (float                                   initialDistance,
                                    float                                   xRotationAngleDegrees,
                                    float                                   yRotationAngleDegrees,
                                    float                                   xAxisLocationUnit,
                                    float                                   yAxisLocationUnit,
                                    unsigned int                            screenWidth,
                                    animation::Box <animation::Point> const &target,
                                    animation::stepper::Stepper       const &stepper) :
    priv (new ag::GlideAnimation::Private (initialDistance,
                                           xRotationAngleDegrees,
                                           yRotationAngleDegrees,
                                           xAxisLocationUnit,
                                           yAxisLocationUnit,
                                           screenWidth,
                                           target,
                                           stepper))
{
}

ag::GlideAnimation::~GlideAnimation ()
{
}
