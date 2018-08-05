/*
 * animation/zoom/zoom.cpp
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
 * Animation that causes a surface to zoom from one rectangle to another.
 */

#include <algorithm>
#include <array>

#include <animation/box_calculation.h>
#include <animation/geometry.h>
#include <animation/math.h>
#include <animation/transform_calculation.h>
#include <animation/zoom/zoom.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace agd = animation::geometry::dimension;
namespace abc = animation::box_calculation;
namespace am = animation::math;
namespace atc = animation::transform_calculation;
namespace az = animation::zoom;

namespace
{
    animation::Point ComputeScaleFactors (animation::Box <animation::Point> const &src,
                                          animation::Box <animation::Point> const &dst)
    {
        return animation::Point ((agd::get <0> (dst.bottomRight ()) -
                                  agd::get <0> (dst.topLeft ())) /
                                 (agd::get <0> (src.bottomRight ()) -
                                  agd::get <0> (src.topLeft ())),
                                 (agd::get <1> (dst.bottomRight ()) -
                                  agd::get <1> (dst.topLeft ())) /
                                 (agd::get <1> (src.bottomRight ()) -
                                  agd::get <1> (src.topLeft ())));
    }

    void ComputeZoomTransform (glm::mat4                               &transform,
                               float                                   progress,
                               animation::Box <animation::Point> const &from,
                               animation::Box <animation::Point> const &to)
    {
        animation::Point const &fromTopLeft (from.topLeft ());
        animation::Point const &fromBottomRight (from.bottomRight ());
        animation::Point const &toTopLeft (to.topLeft ());
        animation::Point const &toBottomRight (to.bottomRight ());

        animation::Point topLeftDelta (toTopLeft);
        agd::pointwise_subtract (topLeftDelta, fromTopLeft);

        animation::Point bottomRightDelta (toBottomRight);
        agd::pointwise_subtract (bottomRightDelta, fromBottomRight);

        animation::Point currentTopLeft (from.topLeft ());
        animation::Point scaledTopLeftDelta (topLeftDelta);
        agd::scale (scaledTopLeftDelta, progress);
        agd::pointwise_add (currentTopLeft, scaledTopLeftDelta);

        animation::Point currentBottomRight (from.bottomRight ());
        animation::Point scaledBottomRightDelta (bottomRightDelta);
        agd::scale (scaledBottomRightDelta, progress);
        agd::pointwise_add (currentBottomRight, scaledBottomRightDelta);

        /* Figure out the delta to center the curent box on top
         * of the center of the target box, then apply scale
         * scale factors equally */
        animation::Box <animation::Point> current (currentTopLeft, currentBottomRight);
        animation::Point currentCenter (abc::ComputeBoxCenter (current));
        animation::Point targetCenterOffset (abc::ComputeBoxCenterOffset (to));
        animation::Point targetCenter (to.topLeft ());
        agd::pointwise_add (targetCenter, targetCenterOffset);

        /* Computation of delta required to get from target to current */
        animation::Point translation (currentCenter);
        agd::pointwise_subtract (translation, targetCenter);

        animation::Point scale (ComputeScaleFactors (to, from));

        transform =
            glm::translate (
                glm::scale (
                    glm::translate (
                        glm::mat4 (1.0),
                        glm::vec3 (agd::get <0> (targetCenterOffset),
                                   agd::get <1> (targetCenterOffset),
                                   0.0)
                    ),
                    glm::vec3 (agd::get <0> (scale), agd::get <1> (scale), 1.0)
                ),
                glm::vec3 (-1 * agd::get <0> (targetCenterOffset) + agd::get <0> (translation),
                           -1 * agd::get <1> (targetCenterOffset) + agd::get <1> (translation),
                           0.0)
            );
    }
}

namespace animation
{
    namespace zoom
    {
        struct ZoomAnimation::Private
        {
            Private (animation::Box <animation::Point> const &from,
                     animation::Box <animation::Point> const &to,
                     unsigned int                            length);

            animation::Box <animation::Point> from;
            animation::Box <animation::Point> to;

            glm::mat4 transform;
            float progress;

            /* We store length as a float as it is the
             * divisor to work out progress. */
            float length;
        };

        ZoomAnimation::Private::Private (animation::Box <animation::Point> const &from,
                                         animation::Box <animation::Point> const &to,
                                         unsigned int                            length) :
            from (from),
            to (to),
            transform (glm::mat4 (1.0)),
            progress (0.0),
            length (static_cast <float> (length))
        {
            ComputeZoomTransform (transform,
                                  progress,
                                  from,
                                  to);
        }
    }
}

std::array <animation::Point, 4>
az::ZoomAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Point, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

bool
az::ZoomAnimation::Step (unsigned int ms)
{
    priv->progress += ms / priv->length;
    float clamped = am::clamp (priv->progress, 0.0f, 1.0f);

    ComputeZoomTransform (priv->transform,
                          clamped,
                          priv->from,
                          priv->to);

    return clamped != 0.0f && clamped != 1.0f;
}

float * const
az::ZoomAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

az::ZoomAnimation::ZoomAnimation (animation::Box <animation::Point> const &from,
                                  animation::Box <animation::Point> const &to,
                                  unsigned int                            length) :
    priv (new az::ZoomAnimation::Private (from, to, length))
{
}

az::ZoomAnimation::~ZoomAnimation ()
{
}
