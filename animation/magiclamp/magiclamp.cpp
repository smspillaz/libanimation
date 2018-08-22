/*
 * animation/magiclamp/magiclamp.cpp
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
 * An animation that causes a surface to magiclamp onto screen, gently
 * following an attenuating sine wave.
 */

#include <algorithm>
#include <array>
#include <vector>

#include <cmath>

#include <animation/box_calculation.h>
#include <animation/geometry.h>
#include <animation/grid/grid.h>
#include <animation/magiclamp/magiclamp.h>
#include <animation/math.h>
#include <animation/property.h>
#include <animation/stepper/linear.h>
#include <animation/stepper/stepper.h>
#include <animation/transform_calculation.h>

#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/gtc/matrix_transform.hpp>

namespace abc = animation::box_calculation;
namespace ag = animation::geometry;
namespace agd = animation::geometry::dimension;
namespace agr = animation::grid;
namespace am = animation::math;
namespace aml = animation::magiclamp;
namespace atc = animation::transform_calculation;

namespace
{
    std::vector <animation::Point> InitializeGridObjects (ag::PointModel <size_t> const           &resolution)
    {
        size_t const gridWidth = agd::get <0> (resolution);
        size_t const gridHeight = agd::get <1> (resolution);
        return std::vector <animation::Point> (gridWidth * gridHeight);
    }

    float Sigmoid (float x, float bend, float offset)
    {
        /* A slightly stronger sigmoid curve */
        return (1.0 / (1.0 + exp (-bend * ((x) - offset))));
    }

    /* Moves from source to target, with the assumption that target
     * represents the normal geometry of the surface
     *
     * This animation simulates the "genie" effect on another popular
     * operating system, appearing to "suck" the window into a destination
     * rectangle.
     *
     * The way it works is a little complicated. Basically, we subdivide
     * the surface into a very high resolution grid and from each point
     * in the source geometry to each point in the target geometry, we
     * we have a sigmoid function (S-curve, eg e^x / (1 + ke^px) that each
     * grid point travels down towards its destination. */
    void MoveGridVerticallyTowardsTargetWithXSigmoidDeformation (animation::Box <animation::Point> const &source,
                                                                 animation::Box <animation::Point> const &target,
                                                                 std::vector <animation::Point>          &grid,
                                                                 ag::PointModel <size_t>           const &gridResolution,
                                                                 float                                   bendFactor,
                                                                 float                                   offsetFactor,
                                                                 float                                   stretchFactor,
                                                                 float                                   deformSpeedFactor,
                                                                 float                                   progress)
    {
        size_t const gridWidth = agd::get <0> (gridResolution);
        size_t const gridHeight = agd::get <1> (gridResolution);
        float const sourceWidth = agd::get <0> (source.bottomRight ()) - agd::get <0> (source.topLeft ());
        float const sourceHeight = agd::get <1> (source.bottomRight ()) - agd::get <1> (source.topLeft ());
        float const targetWidth = agd::get <0> (target.bottomRight ()) - agd::get <0> (target.topLeft ());
        float const targetHeight = agd::get <1> (target.bottomRight ()) - agd::get <1> (target.topLeft ());

        animation::Point topLeftTargetPoint (target.topLeft ());
        animation::Point topLeftSourcePoint (source.topLeft ());
        float const topLeftTargetY = agd::get <1> (topLeftTargetPoint);
        float const topLeftSourceY = agd::get <1> (topLeftSourcePoint);
        float const yDistance = topLeftSourceY - topLeftTargetY;
        float const invProgress = 1.0f - progress;

        float const sigmoid0 = Sigmoid (0.0f, bendFactor, offsetFactor);
        float const sigmoid1 = Sigmoid (1.0f, bendFactor, offsetFactor);
        float const sigmoid1SubSigmoid0 = sigmoid1 - sigmoid0;

        for (size_t j = 0; j < gridHeight; ++j)
        {
            for (size_t i = 0; i < gridWidth; ++i)
            {
                animation::Point sourcePoint (agd::get <0> (source.topLeft ()) +
                                              i * sourceWidth / (gridWidth - 1),
                                              agd::get <1> (source.topLeft ()) +
                                              j * sourceHeight / (gridHeight - 1));
                animation::Point targetPoint (agd::get <0> (target.topLeft ()) +
                                              i * targetWidth / (gridWidth - 1),
                                              agd::get <1> (target.topLeft ()) +
                                              j * targetHeight / (gridHeight - 1));

                /* On the y-axis the grid object moves linearly towards
                 * its y-destination on the target */
                float yStretchFactor = 1.0f / (1.0f - (j / static_cast <float> (gridHeight - 1)) * stretchFactor);
                float yStretchProgress = std::min (invProgress * yStretchFactor, 1.0f);
                float yDelta = agd::get <1> (sourcePoint) - agd::get <1> (targetPoint);
                float stretchedYCoord = agd::get <1> (targetPoint) + yDelta * yStretchProgress;
                float xDeformationProgress = std::min (1.0f - ((topLeftSourceY - stretchedYCoord) / yDistance), 1.0f) *
                                             std::min (invProgress * deformSpeedFactor, 1.0f);
                float xDeformationFactor = (Sigmoid (xDeformationProgress, bendFactor, offsetFactor) - sigmoid0) / sigmoid1SubSigmoid0;
                float stretchedXCoord = agd::get <0> (targetPoint) +
                                        (agd::get <0> (sourcePoint) - agd::get <0> (targetPoint)) *
                                        xDeformationFactor;

                agd::set <0> (grid[j * gridWidth + i], stretchedXCoord);
                agd::set <1> (grid[j * gridWidth + i], stretchedYCoord);
            }
        }
    }

    /* When deforming a UV into the pre-computed grid we must do a linear
     * interpolation along both axes, finding the nearest subdivision source
     * point on a UV mesh, then interpolating into that mesh component
     * on the scene-relative internal mesh. */
    animation::Point InterpolateIntoGrid (animation::Point                  const &uv,
                                          ag::PointModel <size_t>           const &gridResolution,
                                          std::vector <animation::Point>    const &grid)
    {
        animation::Point clampedUV (am::clamp (agd::get <0> (uv), 0.0, 1.0),
                                    am::clamp (agd::get <1> (uv), 0.0, 1.0));
        size_t const gridWidth = agd::get <0> (gridResolution);
        size_t const gridHeight = agd::get <1> (gridResolution);
        size_t nearestLeftX = static_cast <size_t> (agd::get <0> (uv) * (gridWidth - 1));
        size_t nearestTopY = static_cast <size_t> (agd::get <1> (uv) * (gridHeight - 1));

        size_t nearestRightX = std::min (gridWidth - 1, nearestLeftX + 1);
        size_t nearestBottomY = std::min (gridHeight - 1, nearestTopY + 1);

        float unitGridCellWidth = 1.0f / (gridWidth - 1);
        float unitGridCellHeight = 1.0f / (gridHeight - 1);

        float xUnit = (agd::get <0> (uv) - unitGridCellWidth * nearestLeftX) / unitGridCellWidth;
        float yUnit = (agd::get <1> (uv) - unitGridCellHeight * nearestTopY) / unitGridCellHeight;

        float xGridInterpLeft = agd::get <0> (grid[nearestLeftX + nearestTopY * gridWidth]) * (1.0f - yUnit) +
                                agd::get <0> (grid[nearestLeftX + nearestBottomY * gridWidth]) * yUnit;
        float xGridInterpRight = agd::get <0> (grid[nearestRightX + nearestTopY * gridWidth]) * (1.0f - yUnit) +
                                 agd::get <0> (grid[nearestRightX + nearestBottomY * gridWidth]) * yUnit;
        float xGridInterp = xGridInterpLeft + (xGridInterpRight - xGridInterpLeft) * xUnit;

        float yGridInterpTop = agd::get <1> (grid[nearestLeftX + nearestTopY * gridWidth]) * (1.0f - xUnit) +
                               agd::get <1> (grid[nearestRightX + nearestTopY * gridWidth]) * xUnit;
        float yGridInterpBottom = agd::get <1> (grid[nearestLeftX + nearestBottomY * gridWidth]) * (1.0f - xUnit) +
                                  agd::get <1> (grid[nearestRightX + nearestBottomY * gridWidth]) * xUnit;
        float yGridInterp = yGridInterpTop + (yGridInterpBottom - yGridInterpTop) * yUnit;

        return animation::Point (xGridInterp, yGridInterp);
    }
}

namespace animation
{
    namespace magiclamp
    {
        struct MagicLampAnimation::Private
        {
            Private (animation::Box <animation::Point> const &source,
                     animation::Box <animation::Point> const &target,
                     ag::PointModel <size_t>           const &resolution,
                     float                                    bendFactor,
                     float                                    offsetFactor,
                     float                                    stretchFactor,
                     float                                    deformSpeedFactor,
                     animation::stepper::Stepper       const &stepper);

            animation::Box <animation::Point> source;
            animation::Box <animation::Point> target;

            std::vector <animation::Point> grid;
            ag::PointModel <size_t> gridResolution;

            float bendFactor;
            float offsetFactor;
            float stretchFactor;
            float deformSpeedFactor;

            float progress;

            animation::stepper::Stepper       stepper;
        };

        MagicLampAnimation::Private::Private (animation::Box <animation::Point> const &source,
                                              animation::Box <animation::Point> const &target,
                                              ag::PointModel <size_t>           const &resolution,
                                              float                                   bendFactor,
                                              float                                   offsetFactor,
                                              float                                   stretchFactor,
                                              float                                   deformSpeedFactor,
                                              animation::stepper::Stepper       const &stepper) :
            source (source),
            target (target),
            grid (InitializeGridObjects (resolution)),
            gridResolution (resolution),
            bendFactor (bendFactor),
            offsetFactor (offsetFactor),
            stretchFactor (stretchFactor),
            deformSpeedFactor (deformSpeedFactor),
            progress (stepper (0)),
            stepper (stepper)
        {
            MoveGridVerticallyTowardsTargetWithXSigmoidDeformation (source,
                                                                    target,
                                                                    grid,
                                                                    gridResolution,
                                                                    bendFactor,
                                                                    offsetFactor,
                                                                    stretchFactor,
                                                                    deformSpeedFactor,
                                                                    progress);
        }
    }
}

std::array <animation::Vector4D, 4>
aml::MagicLampAnimation::Extremes (std::array <animation::Point, 4> const &corners) const
{
    size_t const gridWidth = agd::get <0> (priv->gridResolution);
    size_t const gridHeight = agd::get <1> (priv->gridResolution);

    size_t topLeftIndex = 0;
    size_t topRightIndex = gridWidth - 1;
    size_t bottomLeftIndex = (gridHeight - 1) * gridWidth;
    size_t bottomRightIndex = bottomLeftIndex + gridWidth - 1;

    return std::array <animation::Vector4D, 4> {
        animation::Vector4D (agd::get <0> (priv->grid[topLeftIndex]),
                             agd::get <1> (priv->grid[topLeftIndex]), 0, 1),
        animation::Vector4D (agd::get <0> (priv->grid[topRightIndex]),
                             agd::get <1> (priv->grid[topRightIndex]), 0, 1),
        animation::Vector4D (agd::get <0> (priv->grid[bottomLeftIndex]),
                             agd::get <1> (priv->grid[bottomLeftIndex]), 0, 1),
        animation::Vector4D (agd::get <0> (priv->grid[bottomRightIndex]),
                             agd::get <1> (priv->grid[bottomRightIndex]), 0, 1)
    };
}

float
aml::MagicLampAnimation::Progress () const
{
    return priv->progress;
}

bool
aml::MagicLampAnimation::Step (unsigned int ms)
{
    priv->progress = am::clamp (priv->stepper (ms), 0.0f, 1.0f);

    MoveGridVerticallyTowardsTargetWithXSigmoidDeformation (priv->source,
                                                            priv->target,
                                                            priv->grid,
                                                            priv->gridResolution,
                                                            priv->bendFactor,
                                                            priv->offsetFactor,
                                                            priv->stretchFactor,
                                                            priv->deformSpeedFactor,
                                                            priv->progress);

    return priv->progress != 0.0f && priv->progress != 1.0f;
}

animation::Point
aml::MagicLampAnimation::DeformUVToModelSpace (animation::Point const &uv) const
{
    return InterpolateIntoGrid (uv, priv->gridResolution, priv->grid);
}

animation::geometry::PointModel <size_t>
aml::MagicLampAnimation::Resolution () const
{
    return priv->gridResolution;
}

ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, Source, animation::Box <animation::Point>, priv->source)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, Target, animation::Box <animation::Point>, priv->target)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, GridResolution, animation::geometry::PointModel <size_t>, priv->gridResolution)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, BendFactor, float, priv->bendFactor)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, OffsetFactor, float, priv->offsetFactor)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, StretchFactor, float, priv->stretchFactor)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, DeformSpeedFactor, float, priv->deformSpeedFactor)
ANIMATION_DEFINE_PROPERTY (aml::MagicLampAnimation, Stepper, animation::stepper::Stepper, priv->stepper)

aml::MagicLampAnimation::MagicLampAnimation (animation::Box <animation::Point> const &source,
                                             animation::Box <animation::Point> const &target,
                                             ag::PointModel <size_t>           const &resolution,
                                             float                                   bendFactor,
                                             float                                   offsetFactor,
                                             float                                   stretchFactor,
                                             float                                   deformSpeedFactor,
                                             animation::stepper::Stepper       const &stepper) :
    priv (new aml::MagicLampAnimation::Private (source,
                                                target,
                                                resolution,
                                                bendFactor,
                                                offsetFactor,
                                                stretchFactor,
                                                deformSpeedFactor,
                                                stepper))
{
}

aml::MagicLampAnimation::~MagicLampAnimation ()
{
}
