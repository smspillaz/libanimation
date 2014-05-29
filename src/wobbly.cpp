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
 * Copright (c) 2005 David Reveman
 */
#include <cmath>
#include <limits>
#include <iomanip>

#include <boost/geometry/arithmetic/arithmetic.hpp>
#include <boost/optional.hpp>

#include <smspillaz/wobbly/wobbly.h>

namespace bg = boost::geometry;

namespace
{
    template <typename P1, typename P2>
    wobbly::Vector
    DetermineDeltaBetweenPointsFromDesired (P1             const &a,
                                            P2             const &b,
                                            wobbly::Vector const &desired)
    {
        wobbly::Vector delta (0.5 * (bg::get <0> (b) - bg::get <0> (a) +
                                     bg::get <0> (desired)),
                              0.5 * (bg::get <1> (b) - bg::get <1> (a) +
                                     bg::get <1> (desired)));
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
        float xDistance = bg::get <0> (b) - bg::get <0> (a);
        float yDistance = bg::get <1> (b) - bg::get <1> (a);

        return std::sqrt (std::pow (xDistance, 2) + std::pow (yDistance, 2));
    }
}

float wobbly::Object::Mass = 15.0f;
float wobbly::Object::Friction = 3.0f;

namespace wobbly
{
    class Object::Private :
        public ImmediatelyMovablePosition
    {
        public:

            Private (PointView <double>       &&position,
                     PointView <double>       &&velocity,
                     PointView <double const> &&force,
                     PointView <double const> &&immediateMovement) noexcept :
                mIsAnchorCount (0),
                position (std::move (position)),
                velocity (std::move (velocity)),
                force (std::move (force)),
                immediateMovement (std::move (immediateMovement))
            {
            }

            void MoveByDelta (Vector const &);
            Vector DeltaTo (Vector const &);

            /* Possibly replace this with a strategy */
            unsigned int         mIsAnchorCount;

            PointView <double> position;
            PointView <double> velocity;
            PointView <double const> force;
            PointView <double const> immediateMovement;
    };
}

wobbly::Object::Object (PointView <double>       &&position,
                        PointView <double>       &&velocity,
                        PointView <double const> &&force,
                        PointView <double const> &&immediateMovement) noexcept :
    priv (new Private (std::move (position),
                       std::move (velocity),
                       std::move (force),
                       std::move (immediateMovement)))
{
}

wobbly::Object::Object (Object &&object) noexcept :
    priv (std::move (object.priv))
{
}

wobbly::Object::~Object ()
{
}

namespace boost
{
    namespace geometry
    {
        /* There is a bug in boost where a concept check was used on
         * the second point only, making it impossible to add from
         * a const type
         */
        namespace fixups
        {
            template <typename Point1, typename Point2>
            inline void assign_point (Point1 &p1, Point2 const &p2)
            {
                BOOST_CONCEPT_ASSERT ((concept::Point <Point1>));
                BOOST_CONCEPT_ASSERT ((concept::ConstPoint <Point2>));

                for_each_coordinate (p1,
                                     detail::point_assignment <Point2> (p2));
            }

            template <typename Point1, typename Point2>
            inline void subtract_point (Point1 &p1, Point2 const &p2)
            {
                BOOST_CONCEPT_ASSERT ((concept::Point <Point1>));
                BOOST_CONCEPT_ASSERT ((concept::ConstPoint <Point2>));

                for_each_coordinate (p1,
                                     detail::point_operation <Point2,
                                                              std::minus> (p2));
            }

            template <typename Point1, typename Point2>
            inline void add_point (Point1 &p1, Point2 const &p2)
            {
                BOOST_CONCEPT_ASSERT ((concept::Point <Point1>));
                BOOST_CONCEPT_ASSERT ((concept::ConstPoint <Point2>));

                for_each_coordinate (p1,
                                     detail::point_operation <Point2,
                                                              std::plus> (p2));
            }
        }
    }
}

namespace
{
    class ClipOperation
    {
        public:

            ClipOperation (double threshold) :
                threshold (threshold)
            {
            }

            template <typename P, int I>
            void apply (P &point) const
            {
                auto value = bg::get <I> (point);
                if (std::fabs (value) < threshold)
                    bg::set <I> (point, 0);
            }

        private:

            double threshold;
    };

    template <typename Point>
    void PointClip (Point  &p, double threshold)
    {
        bg::for_each_coordinate (p, ClipOperation (threshold));
    }

    class AbsoluteOperation
    {
        public:

            template <typename P, int I>
            void apply (P &point) const
            {
                auto value = bg::get <I> (point);
                bg::set <I> (point, fabs (value));
            }
    };

    template <typename Point>
    void MakeAbsolute (Point &p)
    {
        bg::for_each_coordinate (p, AbsoluteOperation ());
    }

    template <typename Point>
    Point Absolute (Point &p)
    {
        Point ret;
        bg::assign (ret, p);
        MakeAbsolute (ret);
        return ret;
    }

    template <typename Velocity, typename Force>
    void ApplyAccelerativeForce (Velocity &velocity,
                                 Force    &force,
                                 float    mass,
                                 float    time)
    {
        wobbly::Vector acceleration;
        bg::fixups::assign_point (acceleration, force);
        bg::divide_value (acceleration, mass);

        /* v[t] = v[t - 1] + at */
        wobbly::Vector additionalVelocity (acceleration);
        bg::multiply_value (additionalVelocity, time);
        bg::add_point (velocity, additionalVelocity);
    }

    inline bool EulerIntegrate (float time, float friction, float mass,
                                wobbly::PointView <double>       &position,
                                wobbly::PointView <double>       &velocity,
                                wobbly::PointView <double const> &force,
                                wobbly::PointView <double const> &immediate)
    {
        assert (mass > 0.0f);

        /* Apply friction, which is exponentially
         * proportional to both velocity and time */
        wobbly::Vector totalForce;
        bg::fixups::assign_point (totalForce, force);
        wobbly::Vector frictionForce;
        bg::assign_point (frictionForce, velocity);
        bg::multiply_value (frictionForce, friction);
        bg::fixups::subtract_point (totalForce, frictionForce);

        /* First apply velocity change for force
         * exerted over time */
        ApplyAccelerativeForce (velocity, totalForce, mass, time);

        /* Clip velocity */
        PointClip (velocity, 0.05);

        /* Distance travelled will be
         *
         *   d[t] = ((v[t - 1] + v[t]) / 2) * t
         */
        wobbly::Vector positionDelta;
        bg::assign_point (positionDelta, velocity);
        bg::multiply_value (positionDelta, time / 2);

        bg::add_point (position, positionDelta);
        //bg::fixups::add_point (position, immediate);

        /* Return true if we still have velocity remaining */
        return std::fabs (bg::get <0> (velocity)) > 0.00 ||
               std::fabs (bg::get <1> (velocity)) > 0.00;
    }
}

bool
wobbly::Object::Step (float time, float friction, float mass)
{
    if (priv->mIsAnchorCount > 0)
    {
        bg::assign_point (priv->velocity,  Vector (0.0, 0.0));
        return false;
    }

    assert (mass > 0.0f);

    return EulerIntegrate (time,
                           friction,
                           mass,
                           priv->position,
                           priv->velocity,
                           priv->force,
                           priv->immediateMovement);
}

bool
wobbly::Object::Step (float time)
{
    return wobbly::Object::Step (time, Friction, Mass);
}

void
wobbly::Object::ModifyPosition (PositionFunc const &modifier)
{
    if (priv->mIsAnchorCount)
        return;

    modifier (priv->position);
}

wobbly::Object::AnchorGrab
wobbly::Object::Grab (AnchorGrab::GrabNotify const &grab,
                      AnchorGrab::ReleaseNotify const &release)
{
    return AnchorGrab (*this->priv,
                       this->priv->mIsAnchorCount,
                       grab,
                       release);
}

wobbly::Object::AnchorGrab
wobbly::Object::Grab ()
{
    auto const nullNotify =
        [](ImmediatelyMovablePosition &) {
        };

    return AnchorGrab (*this->priv,
                       this->priv->mIsAnchorCount,
                       nullNotify, nullNotify);
}

wobbly::PointView <double> const &
wobbly::Object::Position () const
{
    return priv->position;
}

void
wobbly::Object::Private::MoveByDelta (Vector const &delta)
{
    bg::add_point (position, delta);
}

wobbly::Vector
wobbly::Object::Private::DeltaTo (Vector const &delta)
{
    return wobbly::Vector (0, 0);
}

wobbly::Object::AnchorGrab::AnchorGrab (ImmediatelyMovablePosition &position,
                                        unsigned int               &lockCount,
                                        GrabNotify const           &grab,
                                        ReleaseNotify const        &release) :
    position (position),
    lockCount (lockCount),
    release (release)
{
    grab (position);
    ++lockCount;
}

wobbly::Object::AnchorGrab::~AnchorGrab ()
{
    if (lockCount)
        --lockCount;
}

wobbly::Object::AnchorGrab::AnchorGrab (AnchorGrab &&grab) :
    position (grab.position),
    /* This looks counter-intuitive, but its actually the result of a small
     * detail in move semantics. The source object is still going to be
     * destroyed which means the _shared_ lock count between the move-to
     * object and this one is going to be decremented when the source
     * goes out of scope. That means that we need to increment the lock
     * count here when a move is performed.
     */
    lockCount (++grab.lockCount)
{
}

void
wobbly::Object::AnchorGrab::MoveBy (Point const &delta)
{
    position.MoveByDelta (delta);
}

namespace wobbly
{
    class Spring::Private
    {
        public:

            Private (PointView <double> &&forceA,
                     PointView <double> &&forceB,
                     PointView <double const> &&posA,
                     PointView <double const> &&posB,
                     PointView <double> &&immediateA,
                     PointView <double> &&immediateB,
                     Vector distance) :
                forceA (std::move (forceA)),
                forceB (std::move (forceB)),
                posA (std::move (posA)),
                posB (std::move (posB)),
                immediateA (std::move (immediateA)),
                immediateB (std::move (immediateB)),
                desiredDistance (distance)
            {
            }

            PointView <double> forceA;
            PointView <double> forceB;
            PointView <double const> posA;
            PointView <double const> posB;
            PointView <double> immediateA;
            PointView <double> immediateB;
            Vector desiredDistance;
    };
}

wobbly::Spring::Spring (PointView <double> &&forceA,
                        PointView <double> &&forceB,
                        PointView <double const> &&posA,
                        PointView <double const> &&posB,
                        PointView <double> &&immediateA,
                        PointView <double> &&immediateB,
                        Vector distance):
    priv (new Private (std::move (forceA),
                       std::move (forceB),
                       std::move (posA),
                       std::move (posB),
                       std::move (immediateA),
                       std::move (immediateB),
                       distance))
{
}

wobbly::Spring::Spring (Spring &&spring) noexcept :
    priv (std::move (spring.priv))
{
}

wobbly::Spring::~Spring ()
{
}

/* On a linear function we'll have a degree of immediate movement,
 * which ramps up to be at a lower threshold depending on how far
 * away it is from any anchors */
bool
wobbly::Spring::applyForces (float springConstant, float forceRatio)
{
    Vector &desiredDistance (priv->desiredDistance);
    Vector desiredNegative (desiredDistance);
    bg::multiply_value (desiredNegative, -1);

    Vector deltaA (DetermineDeltaBetweenPointsFromDesired (priv->posA,
                                                           priv->posB,
                                                           desiredNegative));
    Vector deltaB (DetermineDeltaBetweenPointsFromDesired (priv->posB,
                                                           priv->posA,
                                                           desiredDistance));

    PointClip (deltaA, ClipThreshold);
    PointClip (deltaB, ClipThreshold);

    Vector springForceA (deltaA);
    Vector springForceB (deltaB);

    bg::multiply_value (springForceA, springConstant * forceRatio);
    bg::multiply_value (springForceB, springConstant * forceRatio);

    bg::add_point (priv->forceA, springForceA);
    bg::add_point (priv->forceB, springForceB);

    /* Calculate immediate movement as the inverse of any
     * delta remaining */
    bg::assign (priv->immediateA, deltaA);
    bg::assign (priv->immediateB, deltaB);

    bg::multiply_value (priv->immediateA, 1.0 - forceRatio);
    bg::multiply_value (priv->immediateB, 1.0 - forceRatio);

    /* Return true if a delta was applied at any point */
    Vector delta (Absolute (deltaA));
    bg::add_point (delta, Absolute (deltaB));

    return fabs (bg::get <0> (delta)) > 0.00 ||
           fabs (bg::get <1> (delta)) > 0.00;
}

bool
wobbly::Spring::applyForces (float springConstant)
{
    return applyForces (springConstant, 1.0);
}

void
wobbly::Spring::scaleLength (Vector scaleFactor)
{
    bg::multiply_point (priv->desiredDistance, scaleFactor);
}

namespace wobbly
{
    class Model::Private
    {
        public:

            typedef std::reference_wrapper <ImmediatelyMovablePosition> IMPRef;

            Private (Point const &initialPosition,
                     float       width,
                     float       height,
                     Settings    &settings);

            std::vector <Object::AnchorGrab>
            Adopt (Private const &other);

            std::array <wobbly::Point, 4> const
            Extremes () const;

            wobbly::Point
            TargetPosition () const;

            BezierMesh           mMesh;
            std::vector <double> mVelocities;
            std::vector <double> mForces;
            std::vector <double> mImmediateMoves;
            std::vector <IMPRef> mAnchoredObjects;

            Settings &mSettings;

            std::vector <Object> mObjects;
            std::vector <Spring> mSprings;

            float mWidth, mHeight;
            bool mCurrentlyUnequal;
    };
}

wobbly::Model::Private::Private (Point const &initialPosition,
                                 float       width,
                                 float       height,
                                 Settings    &settings) :
    mSettings (settings),
    mWidth (width),
    mHeight (height),
    mCurrentlyUnequal (false)
{
    unsigned int const gridWidth = wobbly::BezierMesh::Width;
    unsigned int const gridHeight = wobbly::BezierMesh::Height;

    size_t const objectsSize = ObjectCountForGridSize (gridWidth, gridHeight);

    float tileWidth = mWidth / (gridWidth - 1);
    float tileHeight = mHeight / (gridHeight - 1);

    mObjects.reserve (objectsSize);

    /* Allocate enough storage to keep all our velocities and forces */
    mVelocities.resize (objectsSize * 2);
    mForces.resize (objectsSize * 2);
    mImmediateMoves.resize (objectsSize * 2);

    /* Keep track of positions */
    auto &mutablePositions = mMesh.PointArray ();

    for (size_t i = 0; i < objectsSize; ++i)
    {
        size_t row = i / gridWidth;
        size_t column = i - (row * gridWidth);

        wobbly::PointView <double> position (mutablePositions, i);
        bg::assign (position, initialPosition);
        bg::add_point (position,
                       Point (column * tileWidth,
                              row * tileHeight));

        wobbly::PointView <double> velocity (mVelocities, i);
        wobbly::PointView <double const> force (mForces, i);
        wobbly::PointView <double const> immediateMovement (mImmediateMoves, i);

        mObjects.emplace_back (std::move (position),
                               std::move (velocity),
                               std::move (force),
                               std::move (immediateMovement));
    }

    size_t const nSprings = SpringCountForGridSize (gridWidth, gridHeight);

    mSprings.reserve (nSprings);

    for (size_t j = 0; j < gridHeight; ++j)
    {
        for (size_t i = 0; i < gridWidth; ++i)
        {
            typedef PointView <double> DPV;
            typedef PointView <double const> CDPV;

            size_t current = j * gridWidth + i;
            size_t below = (j + 1) * gridWidth + i;
            size_t right = j * gridWidth + i + 1;

            /* Spring from us to object below us */
            if (j < gridHeight - 1)
            {
                mSprings.emplace_back (DPV (mForces, current),
                                       DPV (mForces, below),
                                       CDPV (mutablePositions, current),
                                       CDPV (mutablePositions, below),
                                       DPV (mImmediateMoves, current),
                                       DPV (mImmediateMoves, below),
                                       Vector (0.0, tileHeight));
            }

            /* Spring from us to object right of us */
            if (i < gridWidth - 1)
            {
                mSprings.emplace_back (DPV (mForces, current),
                                       DPV (mForces, right),
                                       CDPV (mutablePositions, current),
                                       CDPV (mutablePositions, right),
                                       DPV (mImmediateMoves, current),
                                       DPV (mImmediateMoves, right),
                                       Vector (tileWidth, 0.0f));
            }
        }
    }

    assert (mSprings.size () == nSprings);
}

namespace
{
    wobbly::Model::Settings DefaultSettings =
    {
        wobbly::Model::DefaultSpringConstant,
        wobbly::Object::Friction
    };
}

wobbly::Model::Model (Point const &initialPosition,
                      float width,
                      float height,
                      Settings &settings) :
    priv (new Private (initialPosition, width, height, settings))
{
}

wobbly::Model::Model (Point const &initialPosition,
                      float width,
                      float height) :
    Model (initialPosition, width, height, DefaultSettings)
{
}

wobbly::Model::~Model ()
{
}

namespace
{
    typedef std::vector <wobbly::Object> ObjectVector;

    float const StepResolution = 15.0f;

    size_t
    ClosestObjectIndexToAbsolutePosition (ObjectVector        &objects,
                                          wobbly::Point const &position)
    {
        boost::optional <size_t> nearestIndex;
        float distance = std::numeric_limits <float>::max ();

        for (size_t i = 0; i < objects.size (); ++i)
        {
            wobbly::Object const &object (objects[i]);
            float objectDistance =
                std::fabs (PointDistanceScalar (object.Position (), position));
            if (objectDistance < distance)
            {
                nearestIndex = i;
                distance = objectDistance;
            }
        }

        assert (nearestIndex.is_initialized ());
        return nearestIndex.get ();
    }

    wobbly::Object &
    ClosestObjectToAbsolutePosition (ObjectVector        &objects,
                                     wobbly::Point const &position)
    {
        size_t index (ClosestObjectIndexToAbsolutePosition (objects, position));
        return objects[index];
    }
}

void
wobbly::Model::TransformClosestObjectToPosition (TransformFunc const &func,
                                                 Point const         &point)
{
    size_t index (ClosestObjectIndexToAbsolutePosition (priv->mObjects, point));

    wobbly::PointView <double> position (priv->mMesh.PointArray (), index);
    wobbly::PointView <double> velocity (priv->mVelocities, index);
    wobbly::PointView <double> force (priv->mForces, index);

    func (position,
          velocity,
          force);
}

wobbly::Object::AnchorGrab
wobbly::Model::GrabAnchor (Point const &position)
{
    typedef ImmediatelyMovablePosition IMP;
    typedef std::reference_wrapper <IMP> IMPRef;

    wobbly::Object &closest (ClosestObjectToAbsolutePosition (priv->mObjects,
                                                              position));

    /* Bets are off once we've grabbed an anchor, the model is now unequal */
    priv->mCurrentlyUnequal = true;

    /* Set up grab notification */
    auto const grabNotify =
        [this](ImmediatelyMovablePosition &pos) {
            priv->mAnchoredObjects.push_back (std::ref (pos));
        };

    auto const releaseNotify =
        [this](IMP &pos) {
            auto &anchored (priv->mAnchoredObjects);
            anchored.erase (std::remove_if (anchored.begin (),
                                            anchored.end (),
                                            [&pos](IMPRef const &imp) -> bool {
                                                return &(imp.get ()) == &pos;
                                            }),
                            anchored.end ());
        };

    return closest.Grab (grabNotify, releaseNotify);
}

void
wobbly::Model::MoveModelBy (Point const &delta)
{
    auto &points (priv->mMesh.PointArray ());
    size_t const nPoints = points.size () / 2;
    for (size_t i = 0; i < nPoints; ++i)
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

    Vector delta (point);
    bg::subtract_point (delta, priv->TargetPosition ());

    MoveModelBy (delta);
}

void
wobbly::Model::ResizeModel (float width, float height)
{
    /* First, zero or negative widths are invalid */
    assert (width > 0.0f);
    assert (height > 0.0f);

    /* Second, work out the scale factors */
    float const scaleFactorX = width / priv->mWidth;
    float const scaleFactorY = height / priv->mHeight;

    wobbly::Vector const scaleFactor (scaleFactorX, scaleFactorY);

    if (bg::equals (scaleFactor, wobbly::Vector (1.0, 1.0)))
        return;

    wobbly::Point const modelTargetOrigin (priv->TargetPosition ());

    /* Then on each point, implement a transformation
     * for non-anchors that scales the distance between
     * points in model space */
    auto const positionModifier =
        [&](wobbly::PointView <double> &p) {
            bg::subtract_point (p, modelTargetOrigin);
            bg::multiply_point (p, scaleFactor);
            bg::add_point (p, modelTargetOrigin);
        };

    for (auto &obj : priv->mObjects)
        obj.ModifyPosition (positionModifier);

    /* On each spring, apply the scale factor */
    for (auto &spring : priv->mSprings)
        spring.scaleLength (scaleFactor);

    /* Apply width and height changes */
    priv->mWidth = width;
    priv->mHeight = height;
}

std::vector <wobbly::Object::AnchorGrab>
wobbly::Model::Private::Adopt (Private const &other)
{
    BezierMesh const           &mesh (other.mMesh);
    std::vector <double> const &velocities (other.mVelocities);
    std::vector <double> const &forces (other.mForces);
    std::vector <double> const &immediateMovements (other.mImmediateMoves);

    std::vector <wobbly::Object::AnchorGrab> grabs;

    unsigned int const gridWidth = wobbly::BezierMesh::Width;
    unsigned int const gridHeight = wobbly::BezierMesh::Height;

    size_t const objectsSize = ObjectCountForGridSize (gridWidth, gridHeight);

    for (size_t i = 0; i < objectsSize; ++i)
    {
        size_t row = i / gridWidth;
        size_t column = i - (row * gridWidth);

        auto ourPoint (mMesh.PointForIndex (row, column));
        auto theirPoint (mesh.PointForIndex (row, column));

        /* We can't use assign since the types are equal and operator=
         * has been explicitly disabled */
        bg::set <0> (ourPoint, bg::get <0> (theirPoint));
        bg::set <1> (ourPoint, bg::get <1> (theirPoint));
    }

    mVelocities = velocities;
    mForces = forces;
    mImmediateMoves = immediateMovements;

    /* Just assume that the model is currently unequal, this will only
     * lead to a single harmless step in the event that we're not unequal
     * which will then just reset our equality state */
    mCurrentlyUnequal = true;

    /* We also need to copy the grab states. This is tricky.
     * We don't really want to expose a "Grabbed" API to the
     * user but there's a way we can figure out if an object
     * is grabbed - pass a function to modify its position
     * and check to see if that function got called. If it
     * didn't, then it means that the object is grabbed.
     * This is safe to do as it is part of ModifyPosition's
     * contract */
    assert (mObjects.size () == other.mObjects.size ());
    for (size_t i = 0; i < mObjects.size (); ++i)
    {
        bool functionWasCalled = false;
        auto modifier =
            [&functionWasCalled](PointView <double> &p) {
                functionWasCalled = true;
            };

        auto const &otherObject (other.mObjects[i]);
        const_cast <wobbly::Object &> (otherObject).ModifyPosition (modifier);

        if (!functionWasCalled)
            grabs.emplace_back (mObjects[i].Grab ());
    }

    return grabs;
}

std::array <wobbly::Point, 4> const
wobbly::Model::Private::Extremes () const
{
    return mMesh.Extremes ();
}

wobbly::Point
wobbly::Model::Private::TargetPosition () const
{
    /* We can calculate the target position by "forking"
     * the model and performing euler integrations on it
     * until it settles.
     *
     * Of course we only need to know the target position
     * if the model is not at equilibrium.
     */
    wobbly::Point target (0, 0);

    /* Model is not at equilibrium */
    if (mCurrentlyUnequal)
    {
        /* The fact that the model starts at 0, 0 is fine,
         * we will be copying our own data into this one */
        wobbly::Model model (wobbly::Point (0, 0),
                             mWidth,
                             mHeight);

        /* We MUST keep a reference on the grabs. Failure to do so
         * will result in them being automatically released and the
         * target position being incorrect.
         */
        auto const grabs = model.priv->Adopt (*this);

        /* Now that we have a clone of this model, keep
         * stepping it until it is at equilibrium */
        while (model.StepModel (StepResolution));

        /* Now that the model is at equilibrium, Extremes ()[0]
         * will be the top-left corner, we can use this as our
         * reference point */
        return model.Extremes ()[0];
    }

    /* Model is at equilibrium, we can just use our own
     * Extremes ()[0] */
    return Extremes ()[0];
}

bool
wobbly::Model::StepModel (unsigned int time)
{
    bool moreStepsRequired = priv->mCurrentlyUnequal;

    unsigned int steps =
        static_cast <int> (std::ceil (time / StepResolution));

    /* We might not need more steps - set to false initially and then
     * integrate the model to see if we do */
    if (time)
        moreStepsRequired = false;

    /* Force is actually going to be something that changes over time
     * depending on how far about the objects are away from each other.
     *
     * Unfortunately that's not a simple thing to model. It requires us to
     * provide an integration of the force function which is in turn dependent
     * on the position of the objects which are in turn dependent on the force
     * applied. That requires implicit integration, which is quite difficult
     * to implement.
     *
     * Its far easier to simply just approximate this by sampling the
     * integration function. The problem that you end up facing then is that
     * for large enough stepsizes (where stepsize > 2 * friction/ k) then you
     * end up with model instability.
     *
     * Having one step every frame is a good approximation although that
     * might need to change in the future */
    while (steps--)
    {
        /* Reset force map */
        priv->mForces.assign (priv->mForces.size (), 0.0);
        priv->mImmediateMoves.assign (priv->mImmediateMoves.size (), 0.0);

        /* Calculate forces */
        for (auto &spring : priv->mSprings)
            moreStepsRequired |=
                spring.applyForces (priv->mSettings.springConstant);

        /* Integrate forces with positions */
        for (auto &object : priv->mObjects)
            moreStepsRequired |= object.Step (1.0f,
                                              priv->mSettings.friction,
                                              wobbly::Object::Mass);
    }

    priv->mCurrentlyUnequal = moreStepsRequired;
    return priv->mCurrentlyUnequal;
}

wobbly::Point
wobbly::Model::DeformTexcoords (Point const &normalized) const
{
    return priv->mMesh.DeformUnitCoordsToMeshSpace (normalized);
}

std::array <wobbly::Point, 4> const
wobbly::Model::Extremes () const
{
    return priv->Extremes ();
}

namespace wobbly
{
    class BezierMesh::Private
    {
        public:

            std::array <double, TotalIndices> mPoints;
    };
}

wobbly::BezierMesh::BezierMesh () :
    priv (new Private ())
{
}

wobbly::BezierMesh::~BezierMesh ()
{
}

wobbly::Point
wobbly::BezierMesh::DeformUnitCoordsToMeshSpace (Point const &normalized) const
{
    double const u = bg::get <0> (normalized);
    double const v = bg::get <1> (normalized);

    /* Create a vector of coefficients like
     * | (1 - u)^3      |
     * | 3u * (1 - u)^2 |
     * | 3u^2 * (1 - u) |
     * | u^3            |
     *
     * We store some commonly used variables here so that we don't need
     * to recalculate them over and over again
     */
    double const one_u = 1 - u;
    double const one_v = 1 - v;

    double const three_u = 3 * u;
    double const three_v = 3 * v;

    double const u_pow2 = pow (u, 2);
    double const v_pow2 = pow (v, 2);
    double const one_u_pow2 = pow (one_u, 2);
    double const one_v_pow2 = pow (one_v, 2);

    double const uCoefficients[] =
    {
        std::pow (one_u, 3),
        three_u * one_u_pow2,
        3 * u_pow2 * one_u,
        u_pow2 * u
    };

    double const vCoefficients[] =
    {
        std::pow (one_v, 3),
        three_v * one_v_pow2,
        3 * v_pow2 * one_v,
        v_pow2 * v
    };

    double x = 0.0;
    double y = 0.0;

    std::array <double, TotalIndices> const &points (priv->mPoints);

    /* This will access the point matrix in a linear fashion for
     * cache-efficiency */
    for (size_t j = 0; j < Height; ++j)
    {
        for (size_t i = 0; i < Width; ++i)
        {
            size_t const xLookup = j * 2 * Width + i * 2;
            size_t const yLookup = j * 2 * Width + i * 2 + 1;

            x += uCoefficients[i] * vCoefficients[j] * points[xLookup];
            y += uCoefficients[i] * vCoefficients[j] * points[yLookup];
        }
    }

    Point absolutePosition (x, y);
    return absolutePosition;
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

    typedef std::function <double (double, double)> ExtremeFinder;

    template <typename Numeric, typename Minimum, typename Maximum>
    void SetToExtreme (wobbly::Point  &p,
                       Numeric        x,
                       Minimum        xFinder,
                       Numeric        y,
                       Maximum        yFinder)
    {
        bg::set <0> (p, xFinder (bg::get <0> (p), x));
        bg::set <1> (p, yFinder (bg::get <1> (p), y));

        /* Round to next integer */
        PointRound (p);
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

    for (size_t i = 0; i < Width * Height * 2; i += 2)
    {
        double const x = priv->mPoints[i];
        double const y = priv->mPoints[i + 1];

        SetToExtreme (topLeft, x, min, y, min);
        SetToExtreme (topRight, x, max, y, min);
        SetToExtreme (bottomLeft, x, min, y, max);
        SetToExtreme (bottomRight, x, max, y, max);
    }

    return extremes;
}

namespace
{
    unsigned int CoordIndex (unsigned int x,
                             unsigned int y,
                             unsigned int width)
    {
        return y * width + x;
    }
}

wobbly::PointView <double>
wobbly::BezierMesh::PointForIndex (unsigned int x, unsigned int y)
{
    return wobbly::PointView <double> (priv->mPoints,
                                       CoordIndex (x, y,
                                                   BezierMesh::Width));
}

wobbly::PointView <double const>
wobbly::BezierMesh::PointForIndex (unsigned int x, unsigned int y) const
{
    return wobbly::PointView <double const> (priv->mPoints,
                                             CoordIndex (x, y,
                                                         BezierMesh::Width));
}

std::array <double, wobbly::BezierMesh::TotalIndices> &
wobbly::BezierMesh::PointArray ()
{
    return priv->mPoints;
}

std::array <double, wobbly::BezierMesh::TotalIndices> const &
wobbly::BezierMesh::PointArray () const
{
    return priv->mPoints;
}
