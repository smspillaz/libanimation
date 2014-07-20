/*
 * include/wobbly_internal.h
 *
 * Internal class definitions and inline functions for wobbly mesh. These
 * functions need to be declared in a header which is included wherever they
 * are used so that they can be inlined correctly.
 *
 * Implicitly depends on:
 *  - std::function
 *  - std::array
 *  - boost::geometry
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_INTERNAL_H
#define WOBBLY_INTERNAL_H

#include <functional>

#include <boost/geometry/geometry.hpp>
#include <boost/optional.hpp>

#include <smspillaz/wobbly/wobbly.h>

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
        }
    }
}

namespace wobbly
{
    class Spring
    {
        public:

            Spring (PointView <double> &&forceA,
                    PointView <double> &&forceB,
                    PointView <double const> &&posA,
                    PointView <double const> &&posB,
                    Vector distance);
            Spring (Spring &&spring) noexcept;
            ~Spring ();

            Spring & operator= (Spring &&spring) noexcept (true);

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            bool ApplyForces (double springConstant) const;
            void ScaleLength (Vector scaleFactor);

            static constexpr double ClipThreshold = 0.25;

        private:

            PointView <double> mutable forceA;
            PointView <double> mutable forceB;
            PointView <double const>   posA;
            PointView <double const>   posB;
            Vector desiredDistance;
    };

    /* Precision of the model itself */
    namespace config
    {
        static constexpr size_t Width = 4;
        static constexpr size_t Height = 4;
        static constexpr size_t TotalIndices = Width * Height;
        static constexpr size_t ArraySize  = TotalIndices * 2;
    }

    template <int N>
    class TrackedAnchors :
        public Anchor::Storage
    {
        public:

            typedef std::array <Anchor, N> InternalArray;

            TrackedAnchors ()
            {
                anchors.fill (0);
            }

            void Lock (size_t index) override
            {
                /* Keep track of the first-anchor value */
                unsigned int const previousValue = anchors[index]++;
                if (previousValue == 0 && !firstAnchor)
                    firstAnchor = index;
            }

            void Unlock (size_t index) override
            {
                unsigned int const currentValue = --anchors[index];
                bool const firstAnchorIsThisIndex =
                    firstAnchor.is_initialized () &&
                    firstAnchor.get () == index;

                /* Don't have first anchor anymore. Go to the
                 * heaviest anchor next */
                if (currentValue == 0 && firstAnchorIsThisIndex)
                {
                    auto largest = std::max_element (anchors.begin (),
                                                     anchors.end ());
                    if (*largest > 0)
                        firstAnchor = std::distance (anchors.begin (), largest);
                    else
                        firstAnchor.reset ();
                }
            }

            template <typename AnchorAction, typename NonAnchorAction>
            void PerformActions (AnchorAction const &anchorAction,
                                 NonAnchorAction const &nonAnchorAction) const
            {
                for (size_t i = 0; i < N; ++i)
                {
                    if (anchors[i] > 0)
                        anchorAction (i);
                    else
                        nonAnchorAction (i);
                }
            }

            template <typename Action>
            void WithFirstGrabbed (Action const &action) const
            {
                if (firstAnchor.is_initialized ())
                    action (firstAnchor.get ());
            }

        private:

            std::array <unsigned int, N> anchors;
            boost::optional <size_t> firstAnchor;
    };

    typedef std::array <double, config::ArraySize> MeshArray;
    typedef TrackedAnchors <config::TotalIndices> AnchorArray;

    class BezierMesh
    {
        public:

            BezierMesh ();
            ~BezierMesh ();

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            /* Direct access to the points in this mesh is permitted.
             *
             * PointForIndex is just a convenience function to get a PointView
             * by an x, y index.
             *
             * PointArray gets the entire array at once and should be used
             * where the array is being accessed sequentially */
            PointView <double> PointForIndex (size_t x, size_t y);

            MeshArray & PointArray ()
            {
                return mPoints;
            }

            MeshArray const & PointArray () const
            {
                return mPoints;
            }

        private:

            MeshArray mPoints;
    };

    class ConstrainmentStep
    {
        public:

            ConstrainmentStep (double const &threshold,
                               double const &width,
                               double const &height);

            bool operator () (MeshArray         &points,
                              AnchorArray const &anchors);

        private:

            double const &threshold;
            double const &mWidth;
            double const &mHeight;

            MeshArray targetBuffer;
    };

    /* AnchoredIntegration wraps an IntegrationStrategy and performs it on
     * a point only if there is no corresponding anchor set for that point */
    template <typename IntegrationStrategy>
    class AnchoredIntegration
    {
        public:

            AnchoredIntegration (IntegrationStrategy &strategy) :
                strategy (strategy)
            {
            }

            bool operator () (MeshArray         &positions,
                              MeshArray   const &forces,
                              AnchorArray const &anchors,
                              double            friction)
            {
                bool more = false;
                auto const resetAction =
                    [this](size_t index) {
                        strategy.Reset (index);
                    };
                auto const stepAction =
                    [this, friction, &positions, &forces, &more](size_t index) {
                        more |= strategy.Step (index,
                                               1.0,
                                               friction,
                                               Model::Mass,
                                               positions,
                                               forces);
                    };

                anchors.PerformActions (resetAction, stepAction);

               return more;
            }

        private:

            typedef IntegrationStrategy IS;

            AnchoredIntegration (AnchoredIntegration <IS> const &) = delete;
            AnchoredIntegration <IS> &
            operator= (AnchoredIntegration <IS> const &) = delete;

            IntegrationStrategy &strategy;
    };

    bool
    EulerIntegrate (double                           time,
                    double                           friction,
                    double                           mass,
                    wobbly::PointView <double>       &&inposition,
                    wobbly::PointView <double>       &&invelocity,
                    wobbly::PointView <double const> &&inforce);

    class EulerIntegration
    {
        public:

            EulerIntegration ();

            void Reset (size_t i);
            bool Step (size_t          index,
                       double          time,
                       double          friction,
                       double          mass,
                       MeshArray       &positions,
                       MeshArray const &forces);

        private:

            MeshArray velocities;
    };

    class Anchor::Lifetime
    {
    public:

        virtual ~Lifetime () {};

        bool operator== (Lifetime const &other) const
        {
            return SameAs (other);
        }

        bool operator!= (Lifetime const &other) const
        {
            return !(*this == other);
        }

    private:
        virtual bool SameAs (Lifetime const &other) const = 0;
    };

    class AnchorPackage :
        public Anchor::Lifetime
    {
        public:

            AnchorPackage (Point              const &grabPoint,
                           MeshArray                &positions,
                           MeshArray                &forces,
                           size_t                   first,
                           size_t                   second) :
                firstSpring (wobbly::PointView <double> (forces, first),
                             wobbly::PointView <double> (mForces, 0),
                             wobbly::PointView <double> (positions, second),
                             wobbly::PointView <double> (mPositions, 0),
                             Delta (grabPoint, positions, first)),
                secondSpring (wobbly::PointView <double> (forces, second),
                              wobbly::PointView <double> (mForces, 1),
                              wobbly::PointView <double> (positions, second),
                              wobbly::PointView <double> (mPositions, 1),
                              Delta (grabPoint, positions, second))
            {
                mPositions.fill (0);
                mForces.fill (0);
            }

            AnchorPackage (AnchorPackage &&package) noexcept (true):
                mPositions (std::move (package.mPositions)),
                mForces (std::move (package.mForces)),
                firstSpring (std::move (package.firstSpring)),
                secondSpring (std::move (package.secondSpring))
            {
            }

            AnchorPackage &
            operator= (AnchorPackage &&other) noexcept (true)
            {
                if (this == &other)
                    return *this;

                mPositions = std::move (other.mPositions);
                mForces = std::move (other.mForces);
                firstSpring = std::move (other.firstSpring);
                secondSpring = std::move (other.secondSpring);

                return *this;
            }

            ~AnchorPackage () noexcept (true)
            {
            }

        private:

            bool SameAs (Lifetime const &lifetime) const
            {
                return static_cast <Lifetime const *> (this) == &lifetime;
            }

            Vector Delta (Point     const &installationPoint,
                          MeshArray const &positions,
                          size_t          index)
            {
                wobbly::PointView <double const> gridPoint (positions, index);

                Vector distance;
                bg::assign (distance, gridPoint);
                bg::subtract_point (distance, installationPoint);
                return distance;
            }

            std::array <double, 4> mPositions;
            std::array <double, 4> mForces;

            wobbly::Spring firstSpring;
            wobbly::Spring secondSpring;
    };

    class SpringMesh
    {
        public:

            SpringMesh (MeshArray    &array,
                        Vector const &tileSize);

            struct CalculationResult
            {
                bool            forcesExist;
                MeshArray const &forces;
            };

            CalculationResult CalculateForces (double springConstant) const;
            void Scale (Vector const &scaleFactor);

            Anchor::LTH
            InstallAnchorSprings (Point     const &installationPoint,
                                  MeshArray       &positions);

        private:

            SpringMesh (SpringMesh const &mesh) = delete;
            SpringMesh & operator= (SpringMesh other) = delete;

            MeshArray mutable    mForces;
            std::vector <Spring> mSprings;

            Vector               mSpringDimensions;

            std::vector <AnchorPackage> mAnchorSprings;

            void DistributeSprings (MeshArray &array);
    };

    template <typename IntegrationStrategy>
    class SpringStep
    {
        public:

            SpringStep (IntegrationStrategy  &strategy,
                        MeshArray            &array,
                        double         const &constant,
                        double         const &friction,
                        wobbly::Vector const &tileSize) :
                constant (constant),
                friction (friction),
                integrator (strategy),
                mesh (array, tileSize)
            {
            }

            void Scale (Vector const &scaleFactor)
            {
                mesh.Scale (scaleFactor);
            }

            Anchor::LTH
            InstallAnchorSprings (Point     const &installationPoint,
                                  MeshArray       &positions)
            {
                return mesh.InstallAnchorSprings (installationPoint,
                                                  positions);
            }

            bool operator () (MeshArray         &positions,
                              AnchorArray const &anchors)
            {
                auto result = mesh.CalculateForces (constant);

                bool more = result.forcesExist;
                more |= integrator (positions,
                                    result.forces,
                                    anchors,
                                    friction);

                return more;
            }

        private:

            SpringStep (IntegrationStrategy const &) = delete;
            SpringStep <IntegrationStrategy> &
            operator= (SpringStep <IntegrationStrategy> const &) = delete;

            double const &constant;
            double const &friction;

            AnchoredIntegration <IntegrationStrategy> integrator;
            SpringMesh                                    mesh;
    };
}

namespace wobbly
{
    namespace geometry
    {
        namespace bg = boost::geometry;
        namespace bgt = bg::traits;

        namespace detail
        {
            template <typename Function>
            class CoordinateOperationWrapper
            {
                public:

                    CoordinateOperationWrapper (Function const &function) :
                        mFunction (function)
                    {
                    }

                    template <typename P, int I>
                    void apply (P &point) const
                    {
                        bg::set <I> (point, mFunction (bg::get <I> (point)));
                    }

                private:

                    Function const &mFunction;
            };

            template <typename P, typename F>
            inline void CoordinateOperation (P &p, F const &f)
            {
                CoordinateOperationWrapper <F> const wrapper (f);
                bg::for_each_coordinate (p, wrapper);
            }
        }

        template <typename Point>
        inline void ResetIfCloseToZero (Point &p, double t)
        {
            typedef typename bgt::coordinate_type <Point>::type Component;
            detail::CoordinateOperation (p,
                                         [t](Component c) -> Component {
                                             return std::fabs (c) < t ? 0.0 : c;
                                         });
        }

        template <typename Point>
        inline void MakeAbsolute (Point &p)
        {
            typedef typename bgt::coordinate_type <Point>::type Component;
            detail::CoordinateOperation (p,
                                         [](Component c) -> Component {
                                             return std::fabs (c);
                                         });
        }

        template <typename Point>
        inline Point Absolute (Point &p)
        {
            Point ret;
            bg::assign (ret, p);
            MakeAbsolute (ret);
            return ret;
        }
    }
}


namespace wobbly
{
    namespace euler
    {
        template <typename Velocity, typename Force>
        void ApplyAccelerativeForce (Velocity &velocity,
                                     Force    &force,
                                     double   mass,
                                     double   time)
        {
            namespace bg = boost::geometry;

            wobbly::Vector acceleration;
            bg::fixups::assign_point (acceleration, force);
            bg::divide_value (acceleration, mass);

            /* v[t] = v[t - 1] + at */
            wobbly::Vector additionalVelocity (acceleration);
            bg::multiply_value (additionalVelocity, time);
            bg::add_point (velocity, additionalVelocity);
        }
    }
}

inline bool
wobbly::EulerIntegrate (double                           time,
                        double                           friction,
                        double                           mass,
                        wobbly::PointView <double>       &&inposition,
                        wobbly::PointView <double>       &&invelocity,
                        wobbly::PointView <double const> &&inforce)
{
    assert (mass > 0.0f);

    wobbly::PointView <double> position (std::move (inposition));
    wobbly::PointView <double const> force (std::move (inforce));
    wobbly::PointView <double> velocity (std::move (invelocity));

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
    euler::ApplyAccelerativeForce (velocity, totalForce, mass, time);

    /* Clip velocity */
    geometry::ResetIfCloseToZero (velocity, 0.05);

    /* Distance travelled will be
     *
     *   d[t] = ((v[t - 1] + v[t]) / 2) * t
     */
    wobbly::Vector positionDelta;
    bg::assign_point (positionDelta, velocity);
    bg::multiply_value (positionDelta, time / 2);

    bg::add_point (position, positionDelta);

    /* Return true if we still have velocity remaining */
    bool result = std::fabs (bg::get <0> (velocity)) > 0.00 ||
                  std::fabs (bg::get <1> (velocity)) > 0.00;

    return result;
}


inline bool
wobbly::EulerIntegration::Step (size_t          index,
                                double          time,
                                double          friction,
                                double          mass,
                                MeshArray       &positions,
                                MeshArray const &forces)
{
    return EulerIntegrate (time,
                           friction,
                           mass,
                           PointView <double> (positions, index),
                           PointView <double> (velocities, index),
                           PointView <double const> (forces, index));
}

inline void
wobbly::EulerIntegration::Reset (size_t index)
{
    wobbly::PointView <double> velocity (velocities, index);
    bg::assign_value (velocity, 0.0);
}

namespace wobbly
{
    namespace springs
    {
        template <typename P1, typename P2>
        inline wobbly::Vector
        DeltaFromDesired (P1             const &a,
                          P2             const &b,
                          wobbly::Vector const &desired)
        {
            namespace bg = boost::geometry;

            wobbly::Vector delta (0.5 * (bg::get <0> (b) - bg::get <0> (a) +
                                         bg::get <0> (desired)),
                                  0.5 * (bg::get <1> (b) - bg::get <1> (a) +
                                         bg::get <1> (desired)));
            return delta;
        }
    }
}

inline bool
wobbly::Spring::ApplyForces (double springConstant) const
{
    Vector desiredNegative (desiredDistance);
    bg::multiply_value (desiredNegative, -1);

    Vector deltaA (springs::DeltaFromDesired (posA,
                                              posB,
                                              desiredNegative));
    Vector deltaB (springs::DeltaFromDesired (posB,
                                              posA,
                                              desiredDistance));

    geometry::ResetIfCloseToZero (deltaA, ClipThreshold);
    geometry::ResetIfCloseToZero (deltaB, ClipThreshold);

    Vector springForceA (deltaA);
    Vector springForceB (deltaB);

    bg::multiply_value (springForceA, springConstant);
    bg::multiply_value (springForceB, springConstant);

    bg::add_point (forceA, springForceA);
    bg::add_point (forceB, springForceB);

    /* Return true if a delta was applied at any point */
    Vector delta (geometry::Absolute (deltaA));
    bg::add_point (delta, geometry::Absolute (deltaB));

    bool result = bg::get <0> (delta) > 0.00 ||
                  bg::get <1> (delta) > 0.00;

    return result;
}

inline wobbly::SpringMesh::CalculationResult
wobbly::SpringMesh::CalculateForces (double springConstant) const
{
    bool more = false;
    /* Reset all forces back to zero */
    mForces.fill (0.0);

    /* Accumulate force on each end of each spring. Some points are endpoints
     * of multiple springs so these functions may cause a force to be updated
     * multiple (different) times */
    for (Spring const &spring : mSprings)
        more |= spring.ApplyForces (springConstant);

    return { more, mForces };
}

inline wobbly::Point
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
     * We store some commonly used variables here so that we don't
     * need to recalculate them over and over again
     */
    double const one_u = 1 - u;
    double const one_v = 1 - v;

    double const three_u = 3 * u;
    double const three_v = 3 * v;

    double const u_pow2 = u * u;
    double const v_pow2 = v * v;
    double const one_u_pow2 = one_u * one_u;
    double const one_v_pow2 = one_v * one_v;

    long double const uCoefficients[] =
    {
        one_u * one_u * one_u,
        three_u * one_u_pow2,
        3 * u_pow2 * one_u,
        u_pow2 * u
    };

    long double const vCoefficients[] =
    {
        one_v * one_v * one_v,
        three_v * one_v_pow2,
        3 * v_pow2 * one_v,
        v_pow2 * v
    };

    double x = 0.0;
    double y = 0.0;

    /* This will access the point matrix in a linear fashion for
     * cache-efficiency */
    for (size_t j = 0; j < config::Height; ++j)
    {
        for (size_t i = 0; i < config::Width; ++i)
        {
            size_t const xIdx = j * 2 * config::Width + i * 2;
            size_t const yIdx = j * 2 * config::Width + i * 2 + 1;

            x += uCoefficients[j] * vCoefficients[i] * mPoints[xIdx];
            y += uCoefficients[j] * vCoefficients[i] * mPoints[yIdx];
        }
    }

    Point absolutePosition (x, y);
    return absolutePosition;
}

#endif
