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
#include <list>

#include <boost/geometry/geometry.hpp>

/* boost::optional supports references in optional <T> while xstd::optional
 * does not. xstd::optional supports move semantics. Use the latter unless
 * there is a usecase for the former */
#include <third_party/allow_move_optional/optional.hpp>
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
    namespace collections
    {
        namespace detail
        {
            /* Count down until we have all distances */
            template <typename Function,
                      typename Collection,
                      typename IteratorTuple,
                      size_t   N>
            struct PreserveIterator
            {
                IteratorTuple
                operator () (Function      const &function,
                             Collection          &collection,
                             IteratorTuple const &tuple)
                {
                    /* We need to call std::begin twice as the value of it
                     * may change after we've called our iterator-modifying
                     * function */
                    auto distance = std::distance (std::begin (collection),
                                                   std::get <N - 1> (tuple));
                    auto result (PreserveIterator <Function,
                                                   Collection,
                                                   IteratorTuple,
                                                   N - 1> () (function,
                                                              collection,
                                                              tuple));
                    std::get <N - 1> (result) = std::begin (collection) +
                                                distance;
                    return result;
                }
            };

            /* Start counting back up again */
            template <typename Function,
                      typename Collection,
                      typename IteratorTuple>
            struct PreserveIterator <Function, Collection, IteratorTuple, 0>
            {
                IteratorTuple
                operator () (Function      const &function,
                             Collection          &collection,
                             IteratorTuple const &tuple)
                {
                    function ();
                    return IteratorTuple ();
                }
            };
        }

        template <typename Function,
                  typename Collection,
                  typename IteratorTuple>
        IteratorTuple
        KeepIteratorsAlive (Function      const &function,
                            Collection          &collection,
                            IteratorTuple const &tuple)
        {
            typedef Function F;
            typedef Collection C;
            typedef IteratorTuple T;
            constexpr size_t const S = std::tuple_size <IteratorTuple>::value;

            return detail::PreserveIterator <F, C, T, S> () (function,
                                                             collection,
                                                             tuple);
        }
    }

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

    /* Precision of the model itself */
    namespace config
    {
        static constexpr size_t Width = 4;
        static constexpr size_t Height = 4;
        static constexpr size_t TotalIndices = Width * Height;
        static constexpr size_t ArraySize  = TotalIndices * 2;
    }

    typedef std::array <double, config::ArraySize> MeshArray;

    namespace mesh
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

        size_t
        ClosestIndexToPosition (wobbly::MeshArray   &points,
                                wobbly::Point const &pos)
        {
            xstd::optional <size_t> nearestIndex;
            double distance = std::numeric_limits <double>::max ();

            assert (points.size () == wobbly::config::ArraySize);

            for (size_t i = 0; i < wobbly::config::TotalIndices; ++i)
            {
                wobbly::PointView <double> view (points, i);
                double objectDistance = bg::distance (pos, view);
                if (objectDistance < distance)
                {
                    nearestIndex = i;
                    distance = objectDistance;
                }
            }

            assert (nearestIndex);
            return *nearestIndex;
        }

        size_t
        ClosestNeighbour (size_t              initial,
                          wobbly::MeshArray   &points,
                          wobbly::Point const &pos)
        {
            size_t const width = config::Width;

            xstd::optional <size_t> nearestIndex;
            double distance = std::numeric_limits <double>::max ();

            assert (points.size () == wobbly::config::ArraySize);

            std::array <size_t, 4> neighbourIndices =
            {
                {
                    /* left (or right) */
                    initial % width == 0 ? initial + 1 : initial - 1,
                    /* top (or bottom) */
                    initial / width == 0 ? initial + width : initial - width,
                    /* right (or left) */
                    initial % width == (width - 1) ? initial - 1 : initial + 1,
                    /* bottom (or top) */
                    initial / width ==
                        (width - 1) ? initial - width : initial + width
                }
            };

            for (size_t neighbour : neighbourIndices)
            {
                wobbly::PointView <double> view (points, neighbour);
                double objectDistance = bg::distance (pos, view);
                if (objectDistance < distance)
                {
                    nearestIndex = neighbour;
                    distance = objectDistance;
                }
            }

            assert (nearestIndex);
            return *nearestIndex;
        }
    }

    class Spring
    {
        public:

            Spring (PointView <double>       &&forceA,
                    PointView <double>       &&forceB,
                    PointView <double const> &&posA,
                    PointView <double const> &&posB,
                    Vector             const &distance);
            Spring (Spring &&spring) noexcept;
            ~Spring ();

            Spring & operator= (Spring &&spring) noexcept (true);

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            bool ApplyForces (double springConstant) const;
            void ScaleLength (Vector scaleFactor);

            PointView <double const> const & FirstPosition () const
            {
                return posA;
            }

            PointView <double const> const & SecondPosition () const
            {
                return posB;
            }

            PointView <double> const & FirstForce () const
            {
                return forceA;
            }

            PointView <double> const & SecondForce () const
            {
                return forceB;
            }

            class ID
            {
                public:
                    /* Disable copy, enable move */
                    ID (size_t internal) :
                        id (internal)
                    {
                    }

                    ID (ID &&other) noexcept (true) :
                        id (other.id)
                    {
                        Nullify (other);
                    }

                    ID & operator= (ID &&other) noexcept (true)
                    {
                        id = std::move (other.id);
                        Nullify (other);
                        return *this;
                    }

                    bool operator== (ID const &other) const
                    {
                        return this->id == other.id;
                    }

                    bool operator!= (ID const &other) const
                    {
                        return !(*this == other);
                    }

                private:

                    ID (ID const &other) = delete;
                    ID & operator= (ID const &other) = delete;

                    static void Nullify (ID &id)
                    {
                        id.id = 0;
                    }

                    size_t id;
            };

            bool HasID (ID const &candidateId) const
            {
                return id == candidateId;
            }

            struct ConstructionPackage;

            /* A special static factory which returns both a spring and its
             * ID, if it needs to be tracked for further use. The returned
             * ID will be move-only and as such only one object can keep
             * a reference to it - this prevents proliferation of ID's
             * throughout the system */
            static ConstructionPackage
            CreateWithTrackingID (PointView <double>       &&forceA,
                                  PointView <double>       &&forceB,
                                  PointView <double const> &&posA,
                                  PointView <double const> &&posB,
                                  Vector             const &distance);

            static constexpr double ClipThreshold = 0.25;

        private:

            typedef std::function <size_t ()> IDFetchStrategy;
            Spring (PointView<double>       &&forceA,
                    PointView<double>       &&forceB,
                    PointView<const double> &&posA,
                    PointView<const double> &&posB,
                    Vector                  distance,
                    IDFetchStrategy const   &fetchID);

            PointView <double> mutable forceA;
            PointView <double> mutable forceB;
            PointView <double const>   posA;
            PointView <double const>   posB;
            Vector                     desiredDistance;
            ID                         id;
    };

    struct Spring::ConstructionPackage
    {
        Spring spring;
        ID     id;
    };

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
                    firstAnchor && *firstAnchor == index;

                /* Don't have first anchor anymore. Go to the
                 * heaviest anchor next */
                if (currentValue == 0 && firstAnchorIsThisIndex)
                {
                    auto largest = std::max_element (anchors.begin (),
                                                     anchors.end ());
                    if (*largest > 0)
                        firstAnchor = std::distance (anchors.begin (), largest);
                    else
                        firstAnchor.extract ();
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
                if (firstAnchor)
                    action (*firstAnchor);
            }

        private:

            std::array <unsigned int, N> anchors;
            xstd::optional <size_t> firstAnchor;
    };

    typedef TrackedAnchors <config::TotalIndices> AnchorArray;

    template <typename T>
    struct EnableIfMoveOnly :
        public std::enable_if <std::is_nothrow_move_assignable <T>::value &&
                               std::is_nothrow_move_constructible <T>::value &&
                               !std::is_copy_assignable <T>::value &&
                               !std::is_copy_constructible <T>::value>
    {
    };

    template <typename Resource,
              typename = typename EnableIfMoveOnly <Resource>::type>
    class TemporaryOwner
    {
        public:

            typedef std::function <void (Resource &&)> Release;

            TemporaryOwner (Resource      &&resource,
                            Release const &release) :
                resource (std::move (resource)),
                release (release)
            {
            }

            TemporaryOwner (TemporaryOwner &&owner) noexcept (true) :
                resource (std::move (owner.resource)),
                release (std::move (owner.release))
            {
                Nullify (owner);
            }

            TemporaryOwner & operator= (TemporaryOwner &&owner) noexcept (true)
            {
                resource = std::move (owner.resource);
                release = std::move (owner.release);

                Nullify (owner);
                return *this;
            }

            ~TemporaryOwner ()
            {
                if (release)
                    release (std::move (resource));
            }

            operator Resource const & () const
            {
                return resource;
            }

        private:

            static void Nullify (TemporaryOwner &owner)
            {
                /* Cause release to be a newly-created object - which is
                 * technically more expensive than a move, but will
                 * cause the function to be empty meaning we can't
                 * call it on our destructor */
                owner.release = Release ();
            }

            Resource resource;
            Release  release;
    };

    class TargetMesh
    {
        public:

            typedef std::function <void (MeshArray &)> OriginRecalcStrategy;

            class Handle
            {
                public:

                    Handle (size_t id) :
                        id (id)
                    {
                    }

                    Handle (Handle &&handle) noexcept (true) :
                        id (std::move (handle.id))
                    {
                        handle.id = 0;
                    }

                    Handle & operator= (Handle &&handle) noexcept (true)
                    {
                        if (this == &handle)
                            return *this;

                        id = std::move (handle.id);
                        handle.id = 0;

                        return *this;
                    }

                    operator size_t () const
                    {
                        return id;
                    }

                private:

                    size_t id;
            };

            TargetMesh (OriginRecalcStrategy const &recalc);

            wobbly::TemporaryOwner <Handle> Activate () noexcept (true);

            wobbly::MeshArray const & PointArray () const noexcept (true)
            {
                return mPoints;
            }

            wobbly::MeshArray & PointArray () noexcept (true)
            {
                return mPoints;
            }

        private:

            MeshArray            mPoints;
            size_t               activationCount;
            OriginRecalcStrategy origin;
    };

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

            ConstrainmentStep (double    const &threshold,
                               MeshArray const &estimated);

            bool operator () (MeshArray         &points,
                              AnchorArray const &anchors);

        private:

            double    const &threshold;
            MeshArray const &estimated;
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
                    [this](size_t i) {
                        strategy.Reset (i);
                    };
                auto const stepAction =
                    [this, friction, &positions, &forces, &more](size_t i) {
                        more |= strategy.Step (i,
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
            bool Step (size_t          i,
                       double          time,
                       double          friction,
                       double          mass,
                       MeshArray       &positions,
                       MeshArray const &forces);

        private:

            MeshArray velocities;
    };

    class Anchor::GrabStrategy
    {
        public:

            virtual ~GrabStrategy () {};

            virtual void MoveBy (wobbly::Point const &delta)
            {
            }
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

            class SpringVector
            {
                public:

                    SpringVector (std::vector <Spring> &&baseSprings) :
                        mSprings (std::move (baseSprings))
                    {
                    }

                    typedef std::function <void (Spring &)> Function;
                    void Each (Function const &function)
                    {
                        for (auto &spring : mSprings)
                            function (spring);
                    }

                    typedef std::function <void (Spring const &)> ConstFunction;
                    void Each (ConstFunction const &function) const
                    {
                        for (auto const &spring : mSprings)
                            function (spring);
                    }

                    typedef std::function <bool (Spring const &)> Predicate;
                    TemporaryOwner <Spring>
                    TakeMatching (Predicate const &comparator)
                    {
                        auto it = std::remove_if (std::begin (mSprings),
                                                  std::end (mSprings),
                                                  comparator);

                        if (it == mSprings.end ())
                            throw std::logic_error ("Couldn't find matching "
                                                    "spring");

                        Spring steal (std::move (*it));
                        mSprings.erase (it);

                        auto const replacer =
                            [this](Spring &&spring) {
                                auto const idExists =
                                    [this, &spring](Spring::ID const &id) {
                                        return spring.HasID (id);
                                    };

                                auto exists =
                                    std::remove_if (std::begin (mPending),
                                                    std::end (mPending),
                                                    idExists);

                                if (exists != mPending.end ())
                                    mPending.erase (exists);
                                else
                                    mSprings.emplace_back (std::move (spring));
                            };

                       TemporaryOwner <Spring> tmp (std::move (steal),
                                                    replacer);
                       return std::move (tmp);
                    }

                    TemporaryOwner <Spring::ID>
                    EmplaceAndTrack (PointView <double>       &&forceA,
                                     PointView <double>       &&forceB,
                                     PointView <double const> &&posA,
                                     PointView <double const> &&posB,
                                     Vector             const &distance)
                    {
                        auto package =
                            Spring::CreateWithTrackingID (std::move (forceA),
                                                          std::move (forceB),
                                                          std::move (posA),
                                                          std::move (posB),
                                                          distance);

                        mSprings.emplace_back (std::move (package.spring));

                        auto const remover =
                           [this](Spring::ID &&id) {
                                auto const predicate =
                                    [this, &id](Spring const &spring) {
                                        return spring.HasID (id);
                                    };

                                auto exists =
                                    std::remove_if (std::begin (mSprings),
                                                    std::end (mSprings),
                                                    predicate);

                                if (exists == mSprings.end ())
                                    mPending.emplace_back (std::move (id));
                                else
                                    mSprings.erase (exists);
                            };

                        TemporaryOwner <Spring::ID> tmp (std::move (package.id),
                                                         remover);
                        return std::move (tmp);
                    }

                private:

                    SpringVector (SpringVector const &) = delete;
                    SpringVector & operator= (SpringVector const &) = delete;

                    std::vector <Spring>     mSprings;
                    std::vector <Spring::ID> mPending;
            };

            typedef PointView <double const> DCPV;
            typedef std::function <DCPV (Spring const &)> PosPreference;

            Anchor::LTH
            InstallAnchorSprings (Point              const &installationPoint,
                                  PosPreference const &firstPref,
                                  PosPreference const &secondPref);

        private:

            SpringMesh (SpringMesh const &mesh) = delete;
            SpringMesh & operator= (SpringMesh other) = delete;

            MeshArray mutable    mForces;
            SpringVector         mSprings;
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
            InstallAnchorSprings (Point                          const &install,
                                  SpringMesh::PosPreference const &first,
                                  SpringMesh::PosPreference const &second)
            {
                return mesh.InstallAnchorSprings (install, first, second);
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
            SpringMesh                                mesh;
    };

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
    mSprings.Each ([&more, &springConstant](Spring const &spring) {
                       more |= spring.ApplyForces (springConstant);
                   });

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
