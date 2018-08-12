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

    void ComputeGlideTransform (glm::mat4                               &transform,
                                float                                   progress,
                                float                                   initialDistance,
                                float                                   xRotationAngleDegrees,
                                float                                   yRotationAngleDegrees,
                                float                                   xAxisLocationUnit,
                                float                                   yAxisLocationUnit,
                                animation::Box <animation::Point> const &targetBox)
    {
        animation::Point rotationAxis (abc::ComputeRotationAxis (targetBox,
                                                                 xAxisLocationUnit,
                                                                 yAxisLocationUnit));

        transform = 
            glm::translate (
                glm::rotate (
                    glm::rotate (
                        glm::translate (glm::mat4 (1.0f),
                                        glm::vec3 (agd::get <0> (rotationAxis),
                                                   agd::get <1> (rotationAxis),
                                                   0.0f)),
                        DegreesToRadians (xRotationAngleDegrees) * (1.0f - progress),
                        glm::vec3 (1.0f, 0.0f, 0.0f)
                    ),
                    DegreesToRadians (yRotationAngleDegrees) * (1.0f - progress),
                    glm::vec3 (0.0f, 1.0f, 0.0f)
                ),
                glm::vec3 (-1.0f * agd::get <0> (rotationAxis),
                           -1.0f * agd::get <1> (rotationAxis),
                           -1.0f * initialDistance * (1.0f - progress))
            );
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
                     animation::Box <animation::Point> const &target,
                     unsigned int                            length);

            float                             initialDistance;
            float                             xRotationAngleDegrees;
            float                             yRotationAngleDegrees;
            float                             xAxisLocationUnit;
            float                             yAxisLocationUnit;
            animation::Box <animation::Point> target;

            glm::mat4 transform;
            float progress;

            /* We store length as a float as it is the
             * divisor to work out progress. */
            float length;
        };

        GlideAnimation::Private::Private (float                                   initialDistance,
                                          float                                   xRotationAngleDegrees,
                                          float                                   yRotationAngleDegrees,
                                          float                                   xAxisLocationUnit,
                                          float                                   yAxisLocationUnit,
                                          animation::Box <animation::Point> const &target,
                                          unsigned int                            length) :
            initialDistance (initialDistance),
            xRotationAngleDegrees (xRotationAngleDegrees),
            yRotationAngleDegrees (yRotationAngleDegrees),
            xAxisLocationUnit (xAxisLocationUnit),
            yAxisLocationUnit (yAxisLocationUnit),
            target (target),
            transform (glm::mat4 (1.0)),
            progress (0.0),
            length (static_cast <float> (length))
        {
            ComputeGlideTransform (transform,
                                   progress,
                                   initialDistance,
                                   xRotationAngleDegrees,
                                   yRotationAngleDegrees,
                                   xAxisLocationUnit,
                                   yAxisLocationUnit,
                                   target);
        }
    }
}

std::array <animation::Point, 4>
ag::GlideAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Point, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

bool
ag::GlideAnimation::Step (unsigned int ms)
{
    priv->progress += ms / priv->length;
    float clamped = am::clamp (priv->progress, 0.0f, 1.0f);

    ComputeGlideTransform (priv->transform,
                           priv->progress,
                           priv->initialDistance,
                           priv->xRotationAngleDegrees,
                           priv->yRotationAngleDegrees,
                           priv->xAxisLocationUnit,
                           priv->yAxisLocationUnit,
                           priv->target);

    return clamped != 0.0f && clamped != 1.0f;
}

float * const
ag::GlideAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

ag::GlideAnimation::GlideAnimation (float                                   initialDistance,
                                    float                                   xRotationAngleDegrees,
                                    float                                   yRotationAngleDegrees,
                                    float                                   xAxisLocationUnit,
                                    float                                   yAxisLocationUnit,
                                    animation::Box <animation::Point> const &target,
                                    unsigned int                            length) :
    priv (new ag::GlideAnimation::Private (initialDistance,
                                           xRotationAngleDegrees,
                                           yRotationAngleDegrees,
                                           xAxisLocationUnit,
                                           yAxisLocationUnit,
                                           target,
                                           length))
{
}

ag::GlideAnimation::~GlideAnimation ()
{
}
