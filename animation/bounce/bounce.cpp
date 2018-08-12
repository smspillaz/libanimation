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
        animation::Point boxCenter (abc::ComputeBoxCenter (targetBox));

        float const scaleFactor = ComputeScaleFactorFromProgressParameters (progress,
                                                                            initialScale,
                                                                            maximumScale,
                                                                            nBounces);
        transform = 
            glm::translate (
                glm::scale (
                    glm::translate (glm::mat4 (1.0),
                                    glm::vec3 (agd::get <0> (boxCenter),
                                               agd::get <1> (boxCenter),
                                               0.0f)),
                    glm::vec3 (scaleFactor, scaleFactor, 1.0f)
                ),
                glm::vec3 (-1 * agd::get <0> (boxCenter),
                           -1 * agd::get <1> (boxCenter),
                           0.0f)
            );
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
                     unsigned int                            length);

            float                             initialScale;
            float                             maximumScale;
            unsigned int                      nBounce;
            animation::Box <animation::Point> target;

            glm::mat4 transform;
            float progress;

            /* We store length as a float as it is the
             * divisor to work out progress. */
            float length;
        };

        BounceAnimation::Private::Private (float                                   initialScale,
                                           float                                   maximumScale,
                                           unsigned int                            nBounce,
                                           animation::Box <animation::Point> const &target,
                                           unsigned int                            length) :
            initialScale (initialScale),
            maximumScale (maximumScale),
            nBounce (nBounce),
            target (target),
            transform (glm::mat4 (1.0)),
            progress (0.0),
            length (static_cast <float> (length))
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

std::array <animation::Point, 4>
ab::BounceAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Point, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

bool
ab::BounceAnimation::Step (unsigned int ms)
{
    priv->progress += ms / priv->length;
    float clamped = am::clamp (priv->progress, 0.0f, 1.0f);

    ComputeBounceTransform (priv->transform,
                            priv->progress,
                            priv->initialScale,
                            priv->maximumScale,
                            priv->nBounce,
                            priv->target);

    return clamped != 0.0f && clamped != 1.0f;
}

float * const
ab::BounceAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

ab::BounceAnimation::BounceAnimation (float                                   initialScale,
                                      float                                   maximumScale,
                                      unsigned int                            nBounce,
                                      animation::Box <animation::Point> const &target,
                                      unsigned int                            length) :
    priv (new ab::BounceAnimation::Private (initialScale,
                                            maximumScale,
                                            nBounce,
                                            target,
                                            length))
{
}

ab::BounceAnimation::~BounceAnimation ()
{
}
