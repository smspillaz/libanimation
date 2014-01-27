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
 *    |              |
 *    |              |
 *    o(3)---s(3)---o(4)
 *
 * The force delta applied to each spring at time t is
 *
 *     dF[t] = (distance * (o[a] - o[b])) * 0.4
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
 *     Fr[t] = V[t] * 0.083
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
 * Copright (c) 2005 David Reveman
 */
#include <cmath>
#include <limits>

#include <boost/geometry/arithmetic/arithmetic.hpp>

#include <assert.h>

#include <wobbly.h>

namespace bg = boost::geometry;

namespace
{
    template <typename P1, typename P2>
    wobbly::Vector
    DetermineDeltaBetweenPointsFromDesired (P1             const &a,
                                            P2             const &b,
                                            wobbly::Vector const &desired)
    {
        wobbly::Vector delta (0.5 * (bg::get<0> (b) - bg::get<0> (a) +
                                     desired.get<0> ()),
                              0.5 * (bg::get<1> (b) - bg::get<1> (a) +
                                     desired.get<1> ()));
        return delta;
    }

    size_t
    ObjectCountForGridSize (unsigned int width,
                            unsigned int height)
    {
        return width * height;
    }

    size_t
    SpringCountForGridSize (unsigned int width,
                            unsigned int height)
    {
        return ((width - 1) * (height - 1)) * 2 + width + height - 2;
    }

    template <typename P1, typename P2>
    float
    PointDistanceScalar (P1 const &a, P2 const &b)
    {
        float xDistance = bg::get<0> (b) - bg::get<0> (a);
        float yDistance = bg::get<1> (b) - bg::get<1> (a);

        return std::sqrt (std::pow (xDistance, 2) + std::pow(yDistance, 2));
    }
}

float wobbly::Object::Mass = 15.0f;
float wobbly::Object::Friction = 0.1f;

wobbly::Object::Object () noexcept :
    mIsAnchor (false),
    mInternalPoint (2),
    force (Vector (0.0, 0.0)),
    velocity (Vector (0.0, 0.0)),
    position (mInternalPoint, 0)
{
}

wobbly::Object::Object (PointView<double> &&position) noexcept :
    mIsAnchor (false),
    force (Vector (0.0, 0.0)),
    velocity (Vector (0.0, 0.0)),
    position (std::move (position))
{
}

wobbly::Object::Object (Point const &p) noexcept :
    mIsAnchor (false),
    mInternalPoint (2),
    force (Vector (0.0, 0.0)),
    velocity (Vector (0.0, 0.0)),
    position (mInternalPoint, 0)
{
    bg::assign (position, p);
}

wobbly::Object::Object (Object &&object) noexcept :
    mIsAnchor (object.mIsAnchor),
    force (std::move (object.force)),
    velocity (std::move (object.velocity)),
    position (std::move (object.position))
{
}

void
wobbly::Object::Step (float time)
{
    if (mIsAnchor)
    {
        velocity = Vector (0.0, 0.0);
        return;
    }

    Vector acceleration (force);
    bg::divide_value (acceleration, Mass);

    /* v[t] = v[t - 1] + at */
    Vector additionalVelocity (acceleration);
    Vector originalVelocity (velocity);
    bg::multiply_value (additionalVelocity, time);
    bg::add_point (velocity, additionalVelocity);

    /* Distance travelled will be
     *
     *   d[t] = ((v[t - 1] + v[t]) / 2) * t
     */

    /* Apply velocity to position */
    Vector positionDelta (originalVelocity);
    bg::add_point (positionDelta, velocity);
    bg::divide_value (positionDelta, 2);
    bg::multiply_value (positionDelta, time);
    bg::add_point (position, positionDelta);

    /* Apply friction, which is exponentially
     * proportional to both velocity and time */
    bg::multiply_value (velocity, pow ((1 - Friction), time));

    /* Reset force */
    force = Vector (0.0, 0.0);
}

void
wobbly::Object::Lock ()
{
    mIsAnchor = true;
}

void
wobbly::Object::Unlock ()
{
    mIsAnchor = false;
}

bool
wobbly::Object::IsAnchor ()
{
    return mIsAnchor;
}

wobbly::Spring::Spring (Object &a,
                        Object &b,
                        Vector distance):
    a (a),
    b (b),
    desiredDistance (distance)
{
}

wobbly::Spring::Spring () :
    a (static_a),
    b (static_b)
{
}

namespace
{
    class ClipOperation
    {
        public:

            ClipOperation(double threshold) :
                threshold (threshold)
            {
            }

            template <typename P, int I>
            void apply(P &point) const
            {
                auto value = point.template get<I>();
                if (std::fabs(value) < threshold)
                    point.template set<I>(0);
            }

        private:

            double threshold;
    };

    template <typename C, std::size_t D, typename S>
    void PointClip (bg::model::point<C, D, S> &p,
                    double                    threshold)
    {
        bg::for_each_coordinate(p, ClipOperation(threshold));
    }
}

void
wobbly::Spring::applyForces (float springConstant)
{
    Vector desiredNegative (desiredDistance);
    bg::multiply_value (desiredNegative, -1);

    Vector deltaA (DetermineDeltaBetweenPointsFromDesired(a.position,
                                                          b.position,
                                                          desiredNegative));
    Vector deltaB (DetermineDeltaBetweenPointsFromDesired(b.position,
                                                          a.position,
                                                          desiredDistance));

    PointClip(deltaA, ClipThreshold);
    PointClip(deltaB, ClipThreshold);

    Vector forceA (deltaA);
    Vector forceB (deltaB);

    bg::multiply_value (forceA, springConstant);
    bg::multiply_value (forceB, springConstant);

    bg::add_point (a.force,
                   a.IsAnchor() ? Vector(0.0, 0.0) : forceA);
    bg::add_point (b.force,
                   b.IsAnchor() ? Vector(0.0, 0.0) : forceB);
}

wobbly::Model::Model (Point const &initialPosition,
                      float width,
                      float height,
                      unsigned int gridWidth,
                      unsigned int gridHeight) :
    mLastPosition (initialPosition),
    mWidth (width),
    mHeight (height),
    mGridWidth (gridWidth),
    mGridHeight (gridHeight),
    mAnchorObject (nullptr)
{
    size_t objectsSize = ObjectCountForGridSize (gridWidth, gridHeight);

    float tileWidth = mWidth / (gridWidth - 1);
    float tileHeight = mHeight / (gridHeight - 1);

    mObjects.reserve (objectsSize);

    for (size_t i = 0; i < objectsSize; ++i)
    {
        size_t row = i / gridWidth;
        size_t column = i - (row * gridWidth);

        wobbly::PointView<double> pv (mMesh.PointForIndex (row, column));

        mObjects.emplace_back(std::move(pv));
        bg::assign(mObjects[i].position, initialPosition);
        bg::add_point (mObjects[i].position,
                       Point (column * tileWidth,
                              row * tileHeight));
    }

    size_t const nSprings = SpringCountForGridSize (gridWidth, gridHeight);

    mSprings.reserve (nSprings);

    for (size_t j = 0; j < gridHeight; ++j)
    {
        for (size_t i = 0; i < gridWidth; ++i)
        {
            Object &current (mObjects[j * gridWidth + i]);
            /* Spring from us to object below us */
            if (j < gridHeight - 1)
            {
                mSprings.push_back (Spring (current,
                                            mObjects[(j + 1) * gridWidth + i],
                                            Vector (0.0, tileHeight)));
            }

            /* Spring from us to object right of us */
            if (i < gridWidth - 1)
            {
                mSprings.push_back (Spring (current,
                                            mObjects[j * gridWidth + i + 1],
                                            Vector (tileWidth, 0.0f)));
            }

        }
    }

    assert (mSprings.size() == nSprings);
}

void
wobbly::Model::GrabAnchor (const Point &grab)
{
    if (mAnchorObject)
        throw std::logic_error ("Anchor object already grabbed");

    Object *nearest = nullptr;
    float distance = std::numeric_limits<float>::max ();

    for (auto &object : mObjects)
    {
        float objectDistance =
            std::fabs (PointDistanceScalar (object.position, grab));
        if (objectDistance < distance)
        {
            nearest = &object;
            distance = objectDistance;
        }
    }

    mAnchorObject = nearest;
    mAnchorObject->Lock ();
}

void
wobbly::Model::UngrabAnchor ()
{
    if (!mAnchorObject)
        throw std::logic_error ("Anchor object not yet grabbed");

    mAnchorObject->Unlock ();
    mAnchorObject = nullptr;
}

void
wobbly::Model::MoveModelTo (const Point &point)
{
    Vector delta (point);
    bg::subtract_point (delta, mLastPosition);

    mLastPosition = point;

    if (mAnchorObject)
        bg::add_point (mAnchorObject->position, delta);
}

void
wobbly::Model::StepModel (float time)
{
    for (auto &spring : mSprings)
        spring.applyForces(0.4);

    for (auto &object : mObjects)
        object.Step(time);
}

/* Assumed 4 * 4 */
wobbly::BezierMesh::BezierMesh() :
    mPoints(32)
{
}

wobbly::Point
wobbly::BezierMesh::DeformUnitCoordsToMeshSpace (const Point &normalized) const
{
    double const u = normalized.get<0>();
    double const v = normalized.get<1>();

    /* Create a vector of coefficients like
     * | (1 - u)^3      |
     * | 3u * (1 - u)^2 |
     * | 3u^2 * (1 - u) |
     * | u^3            |
     *
     * We store some commonly used variables here so that we don't need
     * to recalculate them over and over again
     */
    const double one_u = 1 - u;
    const double one_v = 1 - v;

    const double three_u = 3 * u;
    const double three_v = 3 * v;

    const double u_pow2 = pow (u, 2);
    const double v_pow2 = pow (v, 2);
    const double one_u_pow2 = pow (one_u, 2);
    const double one_v_pow2 = pow (one_v, 2);

    double const uCoefficients[] =
    {
        std::pow(one_u, 3),
        three_u * one_u_pow2,
        3 * u_pow2 * one_u,
        u_pow2 * u
    };

    double const vCoefficients[] =
    {
        std::pow(one_v, 3),
        three_v * one_v_pow2,
        3 * v_pow2 * one_v,
        u_pow2 * v
    };

    double x = 0.0;
    double y = 0.0;

    /* This will access the point matrix in a linear fashion for
     * cache-efficiency */
    for (size_t j = 0; j < 4; ++j)
    {
        for (size_t i = 0; i < 4; ++i)
        {
            const size_t xLookup = j * 2 * 4 + i * 2;
            const size_t yLookup = j * 2 * 4 + i * 2 + 1;

            x += uCoefficients[i] * vCoefficients[j] * mPoints[xLookup];
            y += uCoefficients[i] * vCoefficients[j] * mPoints[yLookup];
        }
    }

    return Point (x, y);
}

wobbly::PointView<double>
wobbly::BezierMesh::PointForIndex (unsigned int x, unsigned int y)
{
    unsigned int meshIndex = y * 4 + x;
    return wobbly::PointView<double> (mPoints, meshIndex);
}

wobbly::Point
wobbly::Model::DeformTexcoords (const Point &normalized)
{
    return mMesh.DeformUnitCoordsToMeshSpace (normalized);
}
