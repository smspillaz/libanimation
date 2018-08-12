/*
 * animation/bounce/bounce.cpp
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
 * An animation that causes a surface to bounce onto screen, gently
 * following an attenuating sine wave.
 */

#include <algorithm>
#include <array>

#include <cmath>

#include <animation/bounce/bounce.h>
#include <animation/box_calculation.h>
#include <animation/geometry.h>
#include <animation/math.h>
#include <animation/property.h>
#include <animation/stepper/linear.h>
#include <animation/stepper/stepper.h>
#include <animation/transform_calculation.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace agd = animation::geometry::dimension;
namespace ab = animation::bounce;
namespace abc = animation::box_calculation;
namespace am = animation::math;
namespace atc = animation::transform_calculation;

namespace
{
    float SampleSineWave (float        progress,
                          unsigned int nBounces)
    {
        return ::sin (progress * M_PI * nBounces);
    }

    /* This animation simulates a gentle bounce on a sine wave from
     * the center of the window outwards.
     *
     * The best way to think of this animation is to think of an attenuating
     * sine wave squeezed inwards by two bounds that converge on a single point.
     *
     * In thise case those lines are running from the "initialScale" to 1.0
     * and the "maximumScale" down to 1.0. We run the sine wave for 2pi * nBounce
     * iterations (scaling it it to fit within the time range), but the effect
     * of the naimation is scaled according to where we are on the bunds.
     *
     * Now, rotate the attenuating sine wave so that it is facing towards you
     * on the z-axis (OpenGL co-ordinates). This is essentially what the animation
     * is. */
    float ComputeScaleFactorFromProgressParameters (float        progress,
                                                    float        initialScale,
                                                    float        maximumScale,
                                                    unsigned int nBounces)
    {
        /* Squeeze the sine wave into place by applying a linear
         * interpolation to it */
        float const targetScale = 1.0f;
        float sampledSine = SampleSineWave (progress, nBounces);
        float range = (maximumScale - initialScale) * (1.0f - progress);
        float scaleFloor = initialScale + (targetScale - initialScale) * progress;

        return scaleFloor + range * sampledSine;
    }

    void ComputeBounceTransform (glm::mat4                               &transform,
                                 float                                   progress,
                                 float                                   initialScale,
                                 float                                   maximumScale,
                                 unsigned int                            nBounces,
                                 animation::Box <animation::Point> const &targetBox)
    {
        animation::Point boxCenterOffset (abc::ComputeBoxCenterOffset (targetBox));

        float const scaleFactor = ComputeScaleFactorFromProgressParameters (progress,
                                                                            initialScale,
                                                                            maximumScale,
                                                                            nBounces);
        auto centerMat = glm::translate (glm::mat4 (1.0),
                                         glm::vec3 (-1 * agd::get <0> (boxCenterOffset),
                                                    -1 *agd::get <1> (boxCenterOffset),
                                                    0.0));
        auto scaleMat = glm::scale (glm::mat4 (1.0),
                                    glm::vec3 (scaleFactor,
                                               scaleFactor,
                                               1.0));
        auto invCenterMat = glm::translate (glm::mat4 (1.0),
                                            glm::vec3 (agd::get <0> (boxCenterOffset),
                                                       agd::get <1> (boxCenterOffset),
                                                       0.0));

        transform = invCenterMat * scaleMat * centerMat;
    }
}

namespace animation
{
    namespace bounce
    {
        struct BounceAnimation::Private
        {
            Private (float                                   initialScale,
                     float                                   maximumScale,
                     unsigned int                            nBounce,
                     animation::Box <animation::Point> const &target,
                     animation::stepper::Stepper       const &stepper);

            float initialScale;
            float maximumScale;
            unsigned int nBounce;
            animation::Box <animation::Point> target;

            glm::mat4 transform;
            float progress;
            
            animation::stepper::Stepper stepper;
        };

        BounceAnimation::Private::Private (float                                   initialScale,
                                           float                                   maximumScale,
                                           unsigned int                            nBounce,
                                           animation::Box <animation::Point> const &target,
                                           animation::stepper::Stepper       const &stepper) :
            initialScale (initialScale),
            maximumScale (maximumScale),
            nBounce (nBounce),
            target (target),
            transform (glm::mat4 (1.0)),
            progress (stepper (0)),
            stepper (stepper)
        {
            ComputeBounceTransform (transform,
                                    progress,
                                    initialScale,
                                    maximumScale,
                                    nBounce,
                                    target);
        }
    }
}

std::array <animation::Vector4D, 4>
ab::BounceAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Vector4D, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

float
ab::BounceAnimation::Progress () const
{
    return priv->progress;
}

bool
ab::BounceAnimation::Step (unsigned int ms)
{
    priv->progress = am::clamp (priv->stepper (ms), 0.0f, 1.0f);

    ComputeBounceTransform (priv->transform,
                            priv->progress,
                            priv->initialScale,
                            priv->maximumScale,
                            priv->nBounce,
                            priv->target);

    return priv->progress != 0.0f && priv->progress != 1.0f;
}

float * const
ab::BounceAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

ANIMATION_DEFINE_PROPERTY (ab::BounceAnimation, InitialScale, float, priv->initialScale)
ANIMATION_DEFINE_PROPERTY (ab::BounceAnimation, MaximumScale, float, priv->maximumScale)
ANIMATION_DEFINE_PROPERTY (ab::BounceAnimation, NBounce, unsigned int, priv->nBounce)
ANIMATION_DEFINE_PROPERTY (ab::BounceAnimation, Stepper, animation::stepper::Stepper, priv->stepper)
ANIMATION_DEFINE_READONLY_PROPERTY (ab::BounceAnimation, Target, animation::Box <animation::Point>, priv->target)

ab::BounceAnimation::BounceAnimation (float                                   initialScale,
                                      float                                   maximumScale,
                                      unsigned int                            nBounce,
                                      animation::Box <animation::Point> const &target,
                                      animation::stepper::Stepper       const &stepper) :
    priv (new ab::BounceAnimation::Private (initialScale,
                                            maximumScale,
                                            nBounce,
                                            target,
                                            stepper))
{
}

ab::BounceAnimation::~BounceAnimation ()
{
}
