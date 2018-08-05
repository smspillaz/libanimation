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
#include <animation/property.h>
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
    animation::Box <animation::Point> EnsureNonZeroArea (animation::Box <animation::Point> const &box)
    {
        auto x1 = agd::get <0> (box.topLeft());
        auto y1 = agd::get <1> (box.topLeft());
        auto x2 = agd::get <0> (box.bottomRight());
        auto y2 = agd::get <1> (box.bottomRight());

        return animation::Box <animation::Point> (animation::Point (x1, y1),
                                                  animation::Point (x2 == x1 ? x2 + 1 : x2,
                                                                    y2 == y1 ? y2 + 1 : y2));
    }

    /* This is a simple affine transformation that goes from
     * one box to another.
     *
     * The animation runs "from" some source point "to" some target point
     * which is the surface' "natural" position within the scene (eg, if
     * there were a scene graph, the "to" position forms its co-ordinates
     * and its geometry).
     *
     * For instance, if the window was minimizing, then the animation would
     * have to be run in reverse, "from" the minimized point "to" the unminimized
     * point, but stepping backwards. */
    void ComputeZoomTransform (glm::mat4                               &transform,
                               float                                   progress,
                               animation::Box <animation::Point> const &from,
                               animation::Box <animation::Point> const &to)
    {
        animation::Point const &fromTopLeft (from.topLeft ());
        animation::Point const &fromBottomRight (from.bottomRight ());
        animation::Point const &toTopLeft (to.topLeft ());
        animation::Point const &toBottomRight (to.bottomRight ());

        auto fromWidth = agd::get <0> (fromBottomRight) - agd::get <0> (fromTopLeft);
        auto fromHeight = agd::get <1> (fromBottomRight) - agd::get <1> (fromTopLeft);

        auto toWidth = agd::get <0> (toBottomRight) - agd::get <0> (toTopLeft);
        auto toHeight = agd::get <1> (toBottomRight) - agd::get <1> (toTopLeft);

        animation::Point fromCenter (abc::ComputeBoxCenter (from));
        animation::Point toCenter (abc::ComputeBoxCenter (to));
        animation::Point toCenterOffset (abc::ComputeBoxCenterOffset (to));

        /* Translate backwards from "to" to "from" according to the inverse of progress
         * (eg, at progress == 1.0, there will be no translation from the natural point
         * but at progress == 0.0 we translate all the way back from "to" to "from"). */
        animation::Point translation ((agd::get <0> (fromCenter) - agd::get <0> (toCenter)) * (1.0f - progress),
                                      (agd::get <1> (fromCenter) - agd::get <1> (toCenter)) * (1.0f - progress));

        /* Interpolate between the source and destination width */
        animation::Point scaleFactor (((toWidth * progress) + (fromWidth * (1.0 - progress))) /
                                      toWidth,
                                      ((toHeight * progress) + (fromHeight * (1.0 - progress))) /
                                      toHeight);

        animation::Point invScaleFactor (1.0 / agd::get <0> (scaleFactor),
                                         1.0 / agd::get <1> (scaleFactor));

        /* Remember that transforamtions are done back to front when postmultiplying.
         *
         * So we translate to the center first, then scale, then undo the translation
         * then translate to the correct point along the animation. */
        auto centerMat = glm::translate (glm::mat4 (1.0),
                                         glm::vec3 (-1.0 * agd::get <0> (toCenterOffset),
                                                    -1.0 * agd::get <1> (toCenterOffset),
                                                    0.0));
        auto scaleMat = glm::scale (glm::mat4 (1.0),
                                    glm::vec3 (agd::get <0> (scaleFactor),
                                               agd::get <1> (scaleFactor),
                                               1.0));
        auto invCenterMat = glm::translate (glm::mat4 (1.0),
                                            glm::vec3 (agd::get <0> (toCenterOffset),
                                                       agd::get <1> (toCenterOffset),
                                                       0.0));
        auto translationMat = glm::translate (glm::mat4 (1.0),
                                              glm::vec3 (agd::get <0> (translation),
                                                         agd::get <1> (translation),
                                                         0.0));

        transform = translationMat * invCenterMat * scaleMat * centerMat;
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
                     animation::stepper::Stepper       const &stepper);

            animation::Box <animation::Point> from;
            animation::Box <animation::Point> to;

            glm::mat4 transform;
            float progress;

            animation::stepper::Stepper stepper;
        };

        ZoomAnimation::Private::Private (animation::Box <animation::Point> const &from,
                                         animation::Box <animation::Point> const &to,
                                         animation::stepper::Stepper       const &stepper) :
            from (EnsureNonZeroArea (from)),
            to (EnsureNonZeroArea (to)),
            transform (glm::mat4 (1.0)),
            progress (stepper (0)),
            stepper (stepper)
        {
            ComputeZoomTransform (transform,
                                  progress,
                                  this->from,
                                  this->to);
        }
    }
}

std::array <animation::Vector4D, 4>
az::ZoomAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    return std::array <animation::Vector4D, 4> {
        atc::TransformFlattened2DPointBy3DMatrix (corners[0], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[1], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[2], priv->transform),
        atc::TransformFlattened2DPointBy3DMatrix (corners[3], priv->transform)
    };
}

float
az::ZoomAnimation::Progress () const
{
    return priv->progress;
}

bool
az::ZoomAnimation::Step (unsigned int ms)
{
    priv->progress = am::clamp (priv->stepper (ms), 0.0f, 1.0f);

    ComputeZoomTransform (priv->transform,
                          priv->progress,
                          priv->from,
                          priv->to);

    return priv->progress != 0.0f && priv->progress != 1.0f;
}

float * const
az::ZoomAnimation::Matrix () const
{
    return glm::value_ptr (priv->transform);
}

ANIMATION_DEFINE_PROPERTY (az::ZoomAnimation, From, animation::Box <animation::Point>, priv->from)
ANIMATION_DEFINE_READONLY_PROPERTY (az::ZoomAnimation, To, animation::Box <animation::Point>, priv->to)
ANIMATION_DEFINE_PROPERTY (az::ZoomAnimation, Stepper, animation::stepper::Stepper, priv->stepper)

az::ZoomAnimation::ZoomAnimation (animation::Box <animation::Point> const &from,
                                  animation::Box <animation::Point> const &to,
                                  animation::stepper::Stepper       const &stepper) :
    priv (new az::ZoomAnimation::Private (from, to, stepper))
{
}

az::ZoomAnimation::~ZoomAnimation ()
{
}
