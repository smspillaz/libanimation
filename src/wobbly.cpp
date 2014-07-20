/*
 * src/wobbly.cpp
 *
 * A spring model for implementing "wobbly" textures.
 *
 * This works by subdividing a texture's unit-coordinate-space
 * into a series of points, called "Objects". Object in turn, are
 * linked together into a mesh by springs, with each object being
 * connected bidirectionally by a spring both south and east of it,
 * for instance:
 *
 *    o(1)---s(1)---o(2)
 *    |               |
 *    |               |
 *    s(2)          s(4)
 *    |               |
 *    |               |
 *    o(3)---s(3)---o(4)
 *
 * The force delta applied to each spring at time t is
 *
 *     dF[t] = (distance * (o[a] - o[b])) * k
 *
 * This is in turn applied to the child objects
 *
 *     F[a] += -dF[t]
 *     F[b] += dF[t]
 *
 * Velocity at for a given time (t) is calculated by:
 *
 *     A[t] = (F/M)
 *     dV[t] = A[t] * dT
 *     V[t] += dV[t]
 *
 * We don't want infinite acceleration so we apply a friction vector
 * proportional to the velocity of the point at time (t)
 *
 *     Fr[t] = V[t] * Fk
 *
 * "Anchor" objects are special. No force can ever be applied to them,
 * and the feedback effect of this is that greater force will be
 * applied to its siblings.
 *
 * See LICENCE.md for Copyright information.
 *
 * The original implementation of this for Luminocity is
 * Copyright (c) Kristian Hoegsberg
 *
 * The original implementation of this for Compiz is
 * Copyright (c) 2005 David Reveman
 */
#include <limits>

/* boost/geometry/strategies/cartesian/distance_pythagoras depends on
 * boost/numeric/conversion/cast for boost::numeric_cast but does not
 * include it. See Boost bug #10177 */
#include <boost/numeric/conversion/cast.hpp>

#include <boost/geometry/algorithms/distance.hpp>
#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/geometry/strategies/cartesian/distance_pythagoras.hpp>
#include <boost/optional.hpp>

#include <smspillaz/wobbly/wobbly.h>

#include "wobbly_internal.h"

namespace bg = boost::geometry;

namespace
{
    size_t
    SpringCountForGridSize (unsigned int width,
                            unsigned int height)
    {
        return ((width - 1) * (height - 1)) * 2 + width + height - 2;
    }
}

namespace wobbly
{
    struct Anchor::Private
    {
        Private (PointView <double>  &&position,
                 LTH                 &&lifetime,
                 Anchor::Storage     &anchors,
                 size_t              index);
        ~Private ();

        PointView <double>              position;
        LTH lifetime;
        Anchor::Storage                 &anchors;
        size_t                          index;
    };
}

wobbly::Anchor::Private::Private (PointView <double> &&position,
                                  LTH                &&lifetime,
                                  Anchor::Storage    &anchors,
                                  size_t             index) :
    position (std::move (position)),
    lifetime (std::move (lifetime)),
    anchors (anchors),
    index (index)
{
    anchors.Lock (index);
}

wobbly::Anchor::Anchor (PointView <double> &&position,
                        LTH                &&lifetime,
                        Storage            &anchors,
                        size_t            index) :
    priv (new Private (std::move (position),
                       std::move (lifetime),
                       anchors,
                       index))
{
}

wobbly::Anchor::Anchor (Anchor &&grab) :
    priv (std::move (grab.priv))
{
}

wobbly::Anchor::Private::~Private ()
{
    anchors.Unlock (index);
}

wobbly::Anchor::~Anchor ()
{
}

void
wobbly::Anchor::MoveBy (Point const &delta)
{
    bg::add_point (priv->position, delta);
}

wobbly::Spring::Spring (PointView <double> &&forceA,
                        PointView <double> &&forceB,
                        PointView <double const> &&posA,
                        PointView <double const> &&posB,
                        Vector distance) :
    forceA (std::move (forceA)),
    forceB (std::move (forceB)),
    posA (std::move (posA)),
    posB (std::move (posB)),
    desiredDistance (distance)
{
}

wobbly::Spring::Spring (Spring &&spring) noexcept :
    forceA (std::move (spring.forceA)),
    forceB (std::move (spring.forceB)),
    posA (std::move (spring.posA)),
    posB (std::move (spring.posB)),
    desiredDistance (std::move (spring.desiredDistance))
{
}

wobbly::Spring &
wobbly::Spring::operator= (wobbly::Spring &&other) noexcept (true)
{
    if (this == &other)
        return *this;

    forceA = std::move (other.forceA);
    forceB = std::move (other.forceB);
    posA = std::move (other.posA);
    posB = std::move (other.posB);
    desiredDistance = std::move (other.desiredDistance);

    return *this;
}

wobbly::Spring::~Spring ()
{
}

void
wobbly::Spring::ScaleLength (Vector scaleFactor)
{
    bg::multiply_point (desiredDistance, scaleFactor);
}

wobbly::SpringMesh::SpringMesh (MeshArray    &points,
                                Vector const &springDimensions) :
    mSpringDimensions (springDimensions)
{
    DistributeSprings (points);
}

void
wobbly::SpringMesh::DistributeSprings (MeshArray &points)
{
    double const springWidth = bg::get <0> (mSpringDimensions);
    double const springHeight = bg::get <1> (mSpringDimensions);

    size_t const nSprings = SpringCountForGridSize (config::Width,
                                                    config::Height);

    mSprings.clear ();
    mSprings.reserve (nSprings);

    for (size_t j = 0; j < config::Height; ++j)
    {
        for (size_t i = 0; i < config::Width; ++i)
        {
            typedef PointView <double> DPV;
            typedef PointView <double const> CDPV;

            size_t current = j * config::Width + i;
            size_t below = (j + 1) * config::Width + i;
            size_t right = j * config::Width + i + 1;

            /* Spring from us to object below us */
            if (j < config::Height - 1)
            {
                mSprings.emplace_back (DPV (mForces, current),
                                       DPV (mForces, below),
                                       CDPV (points, current),
                                       CDPV (points, below),
                                       Vector (0.0, springHeight));
            }

            /* Spring from us to object right of us */
            if (i < config::Width - 1)
            {
                mSprings.emplace_back (DPV (mForces, current),
                                       DPV (mForces, right),
                                       CDPV (points, current),
                                       CDPV (points, right),
                                       Vector (springWidth, 0.0f));
            }
        }
    }

    assert (mSprings.size () == nSprings);
}

void
wobbly::SpringMesh::Scale (Vector const &scaleFactor)
{
    bg::multiply_point (mSpringDimensions, scaleFactor);

    for (Spring &spring : mSprings)
        spring.ScaleLength (scaleFactor);
}

wobbly::Anchor::LTH
wobbly::SpringMesh::InstallAnchorSprings (Point     const &installationPoint,
                                          MeshArray       &positions)
{
    double firstDistanceScalar = 0;
    double secondDistanceScalar = 0;

    size_t firstIndex = 0;
    size_t secondIndex = 0;

    for (size_t i = 0; i < config::TotalIndices; ++i)
    {
        wobbly::PointView <double> pv (positions, i);
        double distance = bg::distance (pv, installationPoint);

        if (distance > firstDistanceScalar)
        {
            firstIndex = i;
            firstDistanceScalar = distance;
        }
        else if (distance > secondDistanceScalar)
        {
            secondIndex = i;
            secondDistanceScalar = distance;
        }
    }

    mAnchorSprings.emplace_back (installationPoint,
                                 positions,
                                 mForces,
                                 firstIndex,
                                 secondIndex);

    typedef Anchor::LTH LTH;
    return LTH (&(mAnchorSprings.back ()),
                [this](Anchor::Lifetime *lifetime) {
                    auto condition = [lifetime](AnchorPackage const &p) ->bool {
                        return *lifetime == p;
                    };
                    auto it = std::remove_if (std::begin (mAnchorSprings),
                                              std::end (mAnchorSprings),
                                              condition);

                    mAnchorSprings.erase (it);
                });
}

namespace wobbly
{
    class Model::Private
    {
        public:

            Private (Point    const &initialPosition,
                     double         width,
                     double         height,
                     Settings const &settings);

            std::array <wobbly::Point, 4> const
            Extremes () const;

            wobbly::Point
            TargetPosition () const;

            wobbly::Vector
            TileSize () const;

            double mWidth, mHeight;

            /* Anchor - is the point locked or unlocked */
            AnchorArray                   mAnchors;

            /* Constrainment data for each point */
            ConstrainmentStep             mConstrainment;

            /* Position of the point on the grid */
            BezierMesh                    mPositions;

            /* Force of each point on the grid */
            SpringStep <EulerIntegration> mSpring;

            /* Velocity of the point on the grid */
            EulerIntegration              mVelocityIntegrator;

            Model::Settings         const &mSettings;

            bool mCurrentlyUnequal;
    };
}

namespace
{
    inline void
    CalculatePositionArray (wobbly::Point  const &initialPosition,
                            wobbly::MeshArray    &array,
                            wobbly::Vector const &tileSize)
    {
        assert (array.size () == wobbly::config::ArraySize);

        for (size_t i = 0; i < wobbly::config::TotalIndices; ++i)
        {
            size_t const row = i / wobbly::config::Width;
            size_t const column = i % wobbly::config::Width;

            wobbly::PointView <double> position (array, i);
            bg::assign (position, initialPosition);
            bg::add_point (position,
                           wobbly::Point (column * bg::get <0> (tileSize),
                                          row * bg::get <1> (tileSize)));
        }
    }
}

wobbly::Model::Private::Private (Point    const &initialPosition,
                                 double         width,
                                 double         height,
                                 Settings const &settings) :
    mWidth (width),
    mHeight (height),
    mConstrainment (settings.maximumRange, mWidth, mHeight),
    mSpring (mVelocityIntegrator,
             mPositions.PointArray (),
             settings.springConstant,
             settings.friction,
             TileSize ()),
    mSettings (settings)
{
    /* First construct the position array */
    CalculatePositionArray (initialPosition,
                            mPositions.PointArray (),
                            TileSize ());
}

wobbly::Model::Settings wobbly::Model::DefaultSettings =
{
    wobbly::Model::DefaultSpringConstant,
    wobbly::Model::Friction,
    wobbly::Model::DefaultObjectRange
};

wobbly::Model::Model (Point const &initialPosition,
                      double      width,
                      double      height,
                      Settings    const &settings) :
    priv (new Private (initialPosition, width, height, settings))
{
}

wobbly::Model::Model (Point const &initialPosition,
                      double      width,
                      double      height) :
    priv (new Private (initialPosition, width, height, DefaultSettings))
{
}


wobbly::Model::~Model ()
{
}

namespace
{
    template <typename Integrator>
    bool PerformIntegration (wobbly::MeshArray         &positions,
                             wobbly::AnchorArray const &anchors,
                             Integrator                &&integrator)
    {
        return integrator (positions, anchors);
    }

    template <typename Integrator, typename... Remaining>
    bool PerformIntegration (wobbly::MeshArray         &positions,
                             wobbly::AnchorArray const &anchors,
                             Integrator                &&integrator,
                             Remaining&&...            remaining)
    {
        bool more = integrator (positions, anchors);
        more |= PerformIntegration (positions,
                                    anchors,
                                    std::forward <Remaining> (remaining)...);
        return more;
    }

    template <typename... Args>
    bool Integrate (wobbly::MeshArray         &positions,
                    wobbly::AnchorArray const &anchors,
                    unsigned int              steps,
                    Args&&                    ...integrators)
    {
        bool more = false;

        /* Force is actually going to be something that changes over time
         * depending on how far about the objects are away from each other.
         *
         * Unfortunately that's not a simple thing to model. It requires us to
         * provide an integration of the force function which is in turn
         * dependent on the position of the objects which are in turn dependent
         * on the force applied. That requires implicit integration, which is
         * not dynamic.
         *
         * Its far easier to simply just approximate this by sampling the
         * integration function. The problem that you end up facing then is that
         * for large enough stepsizes (where stepsize > 2 * friction/ k) then
         * you end up with model instability.
         *
         * Having one step every frame is a good approximation although that
         * might need to change in the future */
        while (steps--)
            more |= PerformIntegration (positions,
                                        anchors,
                                        std::forward <Args> (integrators)...);

        return more;
    }

    template <typename AnchorPoint>
    wobbly::Point TopLeftPositionInSettledMesh (AnchorPoint    const &anchor,
                                                size_t         const index,
                                                wobbly::Vector const &tileSize)
    {
        wobbly::Point start;
        bg::assign_point (start, anchor);

        size_t const row = index % wobbly::config::Width;
        size_t const column = index / wobbly::config::Width;

        wobbly::Point deltaToTopLeft (bg::get <0> (tileSize) * row,
                                      bg::get <1> (tileSize) * column);
        bg::subtract_point (start, deltaToTopLeft);

        return start;
    }
}

wobbly::Point
wobbly::Model::Private::TargetPosition () const
{
    wobbly::Vector const tileSize (TileSize ());

    auto points (mPositions.PointArray ());
    auto anchors (mAnchors);

    /* If we have at least one anchor, we can take a short-cut and determine
     * the target position by reference to it */
    boost::optional <wobbly::Point> early;
    mAnchors.WithFirstGrabbed ([&early, &points, &tileSize] (size_t index) {
                                    auto const anchor =
                                        wobbly::PointView <double> (points,
                                                                    index);

                                    early =
                                        TopLeftPositionInSettledMesh (anchor,
                                                                      index,
                                                                      tileSize);
                               });

    if (early.is_initialized ())
        return early.get ();

    /* Make our own copies of the integrators and run the integration on them */
    EulerIntegration              integrator (mVelocityIntegrator);
    SpringStep <EulerIntegration> spring (integrator,
                                          points,
                                          mSettings.springConstant,
                                          mSettings.friction,
                                          tileSize);
    ConstrainmentStep             constrainment (mConstrainment);

    /* Keep on integrating this copy until we know the final position */
    while (Integrate (points, anchors, 1, spring, constrainment));

    /* Model will be settled, return the top left point */
    wobbly::Point result;
    bg::assign_point (result, wobbly::PointView <double> (points, 0));

    return result;
}

wobbly::Vector
wobbly::Model::Private::TileSize () const
{
    return wobbly::Vector (mWidth / (config::Width - 1),
                           mHeight / (config::Height - 1));
}

namespace
{
    size_t
    ClosestIndexToAbsolutePosition (wobbly::MeshArray   &points,
                                    wobbly::Point const &pos)
    {
        boost::optional <size_t> nearestIndex;
        double distance = std::numeric_limits <double>::max ();

        assert (points.size () == wobbly::config::ArraySize);

        for (size_t i = 0; i < wobbly::config::TotalIndices; ++i)
        {
            wobbly::PointView <double> view (points, i);
            double objectDistance = std::fabs (bg::distance (pos, view));
            if (objectDistance < distance)
            {
                nearestIndex = i;
                distance = objectDistance;
            }
        }

        assert (nearestIndex.is_initialized ());
        return nearestIndex.get ();
    }
}

wobbly::Anchor
wobbly::Model::GrabAnchor (Point const &position)
{
    auto &points = priv->mPositions.PointArray ();
    size_t index = ClosestIndexToAbsolutePosition (points,
                                                   position);

    /* Bets are off once we've grabbed an anchor, the model is now unequal */
    priv->mCurrentlyUnequal = true;

    auto &positions (priv->mPositions.PointArray ());
    auto lifetime (priv->mSpring.InstallAnchorSprings (position,
                                                       positions));
    return Anchor (wobbly::PointView <double> (points, index),
                   std::move (lifetime),
                   priv->mAnchors,
                   index);
}

void
wobbly::Model::MoveModelBy (Point const &delta)
{
    auto &points (priv->mPositions.PointArray ());

    for (size_t i = 0; i < config::TotalIndices; ++i)
    {
        PointView <double> pv (points, i);
        bg::add_point (pv, delta);
    }
}

void
wobbly::Model::MoveModelTo (Point const &point)
{
    /* We need to calculate the target position for the
     * top left corner. If we do that, then moving the model
     * relative to that will ensure that it settles in the
     * place that we expect. */

    auto const &target (priv->TargetPosition ());

    Vector delta (point);
    bg::subtract_point (delta, target);

    MoveModelBy (delta);
}

void
wobbly::Model::ResizeModel (double width, double height)
{
    /* First, zero or negative widths are invalid */
    assert (width > 0.0f);
    assert (height > 0.0f);

    /* Second, work out the scale factors */
    wobbly::Vector const scaleFactor (width / priv->mWidth,
                                      height / priv->mHeight);

    if (bg::equals (scaleFactor, wobbly::Vector (1.0, 1.0)))
        return;

    wobbly::Point const modelTargetOrigin (priv->TargetPosition ());

    /* Then on each point, implement a transformation
     * for non-anchors that scales the distance between
     * points in model space */
    auto &points (priv->mPositions.PointArray ());

    auto rescaleAction =
        [&points, &modelTargetOrigin, &scaleFactor](size_t index) {
            wobbly::PointView <double> p (points, index);
            bg::subtract_point (p, modelTargetOrigin);
            bg::multiply_point (p, scaleFactor);
            bg::add_point (p, modelTargetOrigin);
        };

    priv->mAnchors.PerformActions ([](size_t index) {
                                   },
                                   rescaleAction);

    /* On each spring, apply the scale factor */
    priv->mSpring.Scale (scaleFactor);

    /* Apply width and height changes */
    priv->mWidth = width;
    priv->mHeight = height;
}

wobbly::ConstrainmentStep::ConstrainmentStep (double const &threshold,
                                              double const &width,
                                              double const &height) :
    threshold (threshold),
    mWidth (width),
    mHeight (height)
{
    targetBuffer.fill (0.0);
}

bool
wobbly::ConstrainmentStep::operator () (MeshArray         &points,
                                        AnchorArray const &anchors)
{
    bool ret = false;
    /* If an anchor is grabbed, then the model will be considered constrained.
     * The first anchor taking priority - we work out the allowable range for
     * each spring and then apply correction as appropriate before even
     * starting to integrate the model
     */
    auto const action =
        [this, &points, &ret](size_t index) {
            wobbly::Vector tileSize (mWidth / (config::Width - 1),
                                     mHeight / (config::Height - 1));
            auto const anchor = wobbly::PointView <double> (points, index);

            wobbly::Point start (TopLeftPositionInSettledMesh (anchor,
                                                               index,
                                                               tileSize));

            CalculatePositionArray (start,
                                    targetBuffer,
                                    tileSize);

            /* In each position in the main position array we'll work out the
             * pythagorean delta between the ideal positon and current one.
             * If it is outside the maximum range, then we'll shrink the delta
             * and reapply it */
            double const maximumRange = threshold;

            for (size_t i = 0; i < config::TotalIndices; ++i)
            {
                wobbly::PointView <double> point (points, i);
                wobbly::PointView <double> target (targetBuffer, i);

                double range = bg::distance (target, point);

                if (range < maximumRange)
                    continue;

                ret |= true;

                auto sin = (bg::get <1> (target) - bg::get <1> (point)) / range;
                auto cos = (bg::get <0> (target) - bg::get <0> (point)) / range;

                /* Now we want to vectorize range and
                 * find our new x and y offsets */
                double const newRange = std::min (maximumRange, range);
                wobbly::Point newDelta (newRange * cos,
                                        newRange * sin);

                /* Offset from the "target" position */
                bg::assign_point (point, target);
                bg::subtract_point (point, newDelta);
            }
        };

    anchors.WithFirstGrabbed (action);

    return ret;
}

wobbly::EulerIntegration::EulerIntegration ()
{
    velocities.fill (0.0);
}

bool
wobbly::Model::Step (unsigned int time)
{
    bool moreStepsRequired = priv->mCurrentlyUnequal;

    double const FPStepResolution = 16.0f;
    unsigned int steps =
        static_cast <unsigned int> (std::ceil (time / FPStepResolution));

    /* We might not need more steps - set to false initially and then
     * integrate the model to see if we do */
    if (time)
        moreStepsRequired = false;

    moreStepsRequired |= Integrate (priv->mPositions.PointArray (),
                                    priv->mAnchors,
                                    steps,
                                    priv->mConstrainment,
                                    priv->mSpring);

    priv->mCurrentlyUnequal = moreStepsRequired;

    /* If we've settled and have grabbed anchors, snap to the mesh resting
     * point where the cursor is, this ensures exact positioning */
    if (!priv->mCurrentlyUnequal)
    {
        auto       &positions (priv->mPositions.PointArray ());
        auto const &anchors (priv->mAnchors);

        auto const action =
            [this, &positions](size_t index) {
                wobbly::Vector tileSize (priv->TileSize ());

                auto const anchor = wobbly::PointView <double> (positions,
                                                                index);

                auto const tl (TopLeftPositionInSettledMesh (anchor,
                                                             index,
                                                             tileSize));

                CalculatePositionArray (tl,
                                        positions,
                                        tileSize);
            };
        anchors.WithFirstGrabbed (action);
    }

    return priv->mCurrentlyUnequal;
}

wobbly::Point
wobbly::Model::DeformTexcoords (Point const &normalized) const
{
    return priv->mPositions.DeformUnitCoordsToMeshSpace (normalized);
}

std::array <wobbly::Point, 4> const
wobbly::Model::Extremes () const
{
    return priv->mPositions.Extremes ();
}

wobbly::BezierMesh::BezierMesh ()
{
    mPoints.fill (0.0);
}

wobbly::BezierMesh::~BezierMesh ()
{
}

namespace
{
    class PointRoundOperation
    {
        public:

            template <typename P, int I>
            void apply (P &p) const
            {
                bg::set <I> (p, std::round (bg::get <I> (p)));
            }
    };

    template <typename Point>
    void PointRound (Point &point)
    {
        bg::for_each_coordinate (point, PointRoundOperation ());
    }

    template <typename Numeric>
    struct NumericTraits
    {
        typedef Numeric (*Chooser) (Numeric, Numeric);
    };

    template <typename Numeric>
    void SetToExtreme (wobbly::Point                             &p,
                       Numeric                                   x,
                       typename NumericTraits <Numeric>::Chooser xFinder,
                       Numeric                                   y,
                       typename NumericTraits <Numeric>::Chooser yFinder)
    {
        bg::set <0> (p, xFinder (bg::get <0> (p), x));
        bg::set <1> (p, yFinder (bg::get <1> (p), y));

        /* Round to next integer */
        PointRound (p);
    }

    unsigned int CoordIndex (size_t x,
                             size_t y,
                             unsigned int width)
    {
        return y * width + x;
    }
}

std::array <wobbly::Point, 4> const
wobbly::BezierMesh::Extremes () const
{
    double const maximum = std::numeric_limits <double>::max ();
    double const minimum = std::numeric_limits <double>::lowest ();

    std::array <wobbly::Point, 4> extremes =
    {
        {
            wobbly::Point (maximum, maximum),
            wobbly::Point (minimum, maximum),
            wobbly::Point (maximum, minimum),
            wobbly::Point (minimum, minimum)
        }
    };

    wobbly::Point &topLeft (extremes[0]);
    wobbly::Point &topRight (extremes[1]);
    wobbly::Point &bottomLeft (extremes[2]);
    wobbly::Point &bottomRight (extremes[3]);

    auto min = [](double lhs, double rhs) -> double {
        double result =  std::min (lhs, rhs);
        return result;
    };

    auto max = [](double lhs, double rhs) -> double {
        double result = std::max (lhs, rhs);
        return result;
    };

    for (size_t i = 0; i < config::Width * config::Height * 2; i += 2)
    {
        double const x = mPoints[i];
        double const y = mPoints[i + 1];

        SetToExtreme (topLeft, x, min, y, min);
        SetToExtreme (topRight, x, max, y, min);
        SetToExtreme (bottomLeft, x, min, y, max);
        SetToExtreme (bottomRight, x, max, y, max);
    }

    return extremes;
}

wobbly::PointView <double>
wobbly::BezierMesh::PointForIndex (size_t x, size_t y)
{
    return wobbly::PointView <double> (mPoints,
                                       CoordIndex (x, y,
                                                   config::Width));
}
