/*
 * include/wobbly.h
 *
 * C++ Interface for "wobbly" textures
 *
 * Implicitly depends on:
 *  - std::vector
 *  - boost::geometry
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_H
#define WOBBLY_H

#include <array>
#include <functional>
#include <memory>
#include <vector>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <boost/utility/enable_if.hpp>
#include <cstdio>

namespace wobbly
{
    namespace bg = boost::geometry;

    template <typename NumericType>
    struct PointView
    {
        typedef NumericType NT;
        typedef typename std::remove_const <NumericType>::type NTM;
        typedef typename std::add_const <NumericType>::type NTC;

        typedef typename std::is_const <NT> IC;

        template <std::size_t N>
        PointView (std::array <NTM, N> &points,
                   std::size_t         index) :
            x (points.at (index * 2)),
            y (points.at (index * 2 + 1))
        {
        }

        template <std::size_t N,
                  typename = typename std::enable_if <IC::value && N>::type>
        PointView (std::array <NTM, N> const &points,
                   std::size_t               index) :
            x (points.at (index * 2)),
            y (points.at (index * 2 + 1))
        {
        }

        PointView (std::vector <NTM> &points,
                   std::size_t       index) :
            x (points.at (index * 2)),
            y (points.at (index * 2 + 1))
        {
        }

        PointView (NTM         *points,
                   std::size_t index) :
            x (points[index * 2]),
            y (points[index * 2 + 1])
        {
        }

        PointView (PointView <NumericType> &&view) :
            x (view.x),
            y (view.y)
        {
        }

        PointView (PointView <NumericType> const &p) = delete;
        PointView & operator= (PointView <NumericType> const &p) = delete;

        /* cppcheck thinks this is a constructor, so we need to suppress
         * a constructor warning here.
         */
        // cppcheck-suppress uninitMemberVar
        operator PointView <NTC> ()
        {
            return PointView <NTC> (&this->x, 0);
        }

        NumericType &x;
        NumericType &y;
    };

    template <typename NumericType>
    class IPointView
    {
        public:

            typedef typename std::remove_const <NumericType>::type NTM;
            typedef typename std::add_const <NumericType>::type NTC;

            template <typename Container>
            IPointView (Container const &container,
                        size_t          index) :
                index (index),
                data (container.data ())
            {
                assert (index < container.size () - 1);
            }

            IPointView (NumericType *array,
                        size_t      index) :
                index (index),
                data (array)
            {
            }

            IPointView (IPointView <NumericType> const &p) = delete;
            IPointView & operator= (IPointView <NumericType> const &p) = delete;

            IPointView (IPointView <NumericType> &&view) :
                index (view.index),
                data (view.data)
            {
                view.index = 0;
            }

            /* cppcheck thinks this is a constructor, so we need to suppress
             * a constructor warning here.
             */
            // cppcheck-suppress uninitMemberVar
            operator IPointView <NTC> ()
            {
                return IPointView <NTC> (&this->x, 0);
            }

        private:

            size_t              index;
            NumericType * const data;
    };

    struct Point
    {
        Point (double x, double y) :
            x (x),
            y (y)
        {
        }

        Point () :
            x (0),
            y (0)
        {
        }

        Point (Point const &p) :
            x (p.x),
            y (p.y)
        {
        }

        void swap (Point &a, Point &b)
        {
            using std::swap;

            swap (a.x, b.x);
            swap (a.y, b.y);
        }

        Point & operator= (Point other)
        {
            swap (*this, other);

            return *this;
        }

        double x;
        double y;
    };

    typedef Point Vector;

    class ImmediatelyMovablePosition
    {
        public:

            typedef ImmediatelyMovablePosition IMP;

            ImmediatelyMovablePosition () {};
            ImmediatelyMovablePosition (IMP const &) = delete;
            IMP & operator= (IMP const &) = delete;

            virtual ~ImmediatelyMovablePosition () {};

            virtual void MoveByDelta (Vector const &) = 0;
            virtual Vector DeltaTo (Vector const &) const = 0;
            virtual wobbly::PointView <double> const & Position () const = 0;
            virtual Point const & Index () const = 0;
    };

    typedef std::reference_wrapper <ImmediatelyMovablePosition> IMPRef;

    class AnchorGrab
    {
        public:

             AnchorGrab (AnchorGrab &&);
             void MoveBy (Point const &delta);

             AnchorGrab (wobbly::PointView <double> &&point,
                         unsigned int               &lockCount);
             ~AnchorGrab ();

        private:

            AnchorGrab (const AnchorGrab &) = delete;
            AnchorGrab & operator= (const AnchorGrab &) = delete;

            PointView <double> position;
            unsigned int       &lockCount;
    };

    class Object
    {
        public:

            Object (PointView <double>       &&position,
                    PointView <double>       &&velocity,
                    PointView <double const> &&force,
                    Point                    &&index) noexcept;
            Object (Object &&object) noexcept;
            ~Object ();

            Object (Object const &object) = delete;
            Object & operator= (Object const &) = delete;

            bool Step (float time);
            bool Step (float time, float friction, float mass);

            /* These two functions will only ever modify the force or
             * position (never the velocity) of an object if it is not
             * anchored. Anchored objects are by definition movable only
             * by the object which has a reference in its object graph
             * to a grab */
            typedef std::function <void (PointView <double> &)> PositionFunc;

            void ModifyPosition (PositionFunc const &modifier);

            /* Default values */
            static float Mass;
            static float Friction;

            class AnchorGrab
            {
                public:

                    AnchorGrab (AnchorGrab &&);
                    void MoveBy (Point const &delta);

                    typedef ImmediatelyMovablePosition IMP;
                    typedef std::function <void (IMP &)> Notify;
                    typedef Notify GrabNotify;
                    typedef Notify ReleaseNotify;

                    AnchorGrab (ImmediatelyMovablePosition &position,
                                unsigned int               &lockCount,
                                GrabNotify const           &grab,
                                ReleaseNotify const        &release);
                    ~AnchorGrab ();

                private:

                    AnchorGrab (const AnchorGrab &) = delete;
                    AnchorGrab & operator= (const AnchorGrab &) = delete;

                    ImmediatelyMovablePosition &position;
                    unsigned int               &lockCount;
                    ReleaseNotify              release;
            };

            AnchorGrab Grab ();
            AnchorGrab Grab (AnchorGrab::GrabNotify const &grab,
                             AnchorGrab::ReleaseNotify const &release);
            PointView <double> const & Position () const;

        private:

            bool IsAnchored () const;

            class Private;
            std::unique_ptr <Private> priv;
    };

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

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            bool applyForces (double springConstant);
            void scaleLength (Vector scaleFactor);

            static constexpr double ClipThreshold = 0.25;

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };

    class BezierMesh
    {
        public:

            BezierMesh ();
            ~BezierMesh ();

            static constexpr unsigned int Width = 4;
            static constexpr unsigned int Height = 4;
            static constexpr unsigned int TotalIndices = Width * Height;
            static constexpr unsigned int TotalIndices2D  = TotalIndices * 2;

            typedef std::array <double, TotalIndices2D> MeshArray;
            typedef std::array <unsigned int, TotalIndices> AnchorArray;

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            /* Direct access to the points in this mesh is permitted.
             *
             * PointForIndex is just a convenience function to get a PointView
             * by an x, y index.
             *
             * MeshArray gets the entire array at once and should be used
             * where the array is being accessed sequentially */
            PointView <double> PointForIndex (size_t x, size_t y);

            MeshArray & PointArray ();

            PointView <double const> PointForIndex (size_t x, size_t y) const;

            MeshArray const & PointArray () const;

        private:

            BezierMesh::MeshArray mPoints;
    };

    typedef std::function <wobbly::Point ()> TargetPositionQuery;

    class ConstrainmentStep
    {
        public:

            ConstrainmentStep (double const &threshold,
                               double const &width,
                               double const &height);

            bool operator () (BezierMesh::MeshArray         &points,
                              BezierMesh::AnchorArray const &anchors);

        private:

            double const &threshold;
            double const &mWidth;
            double const &mHeight;

            BezierMesh::MeshArray targetBuffer;
    };

    class EulerIntegration
    {
        public:

            EulerIntegration ();
        
            bool operator () (BezierMesh::MeshArray         &positions,
                              BezierMesh::MeshArray const   &forces,
                              BezierMesh::AnchorArray const &anchors,
                              double                        friction);

        private:

            BezierMesh::MeshArray velocities;
    };

    class SpringMesh
    {
        public:

            SpringMesh (BezierMesh::MeshArray &array,
                        double       springWidth,
                        double       springHeight);

            /* Need to get rid of this later */
            SpringMesh (SpringMesh const &step);

            SpringMesh & operator= (SpringMesh other);

            friend void swap (SpringMesh &first, SpringMesh &second)
            {
                using std::swap;

                swap (first.mForces, second.mForces);
                swap (first.mSprings, second.mSprings);
                swap (first.springWidth, second.springWidth);
                swap (first.springHeight, second.springHeight);
            }

            void scale (double x, double y);

            BezierMesh::MeshArray &mPositions;
            BezierMesh::MeshArray mForces;
            std::vector <Spring> mSprings;

            double springWidth;
            double springHeight;

        private:

            void DistributeSprings (double width, double height);
    };

    template <class IntegrationStrategy>
    class SpringStep
    {
        public:

            SpringStep (BezierMesh::MeshArray &array,
                        double const &constant,
                        double const &friction,
                        double       springWidth,
                        double       springHeight) :
                constant (constant),
                friction (friction),
                mesh (array, springWidth, springHeight)
            {
            }

            /* Need to get rid of this later */
            SpringStep (SpringStep const &step) :
                constant (step.constant),
                friction (step.friction),
                strategy (step.strategy),
                mesh (step.mesh)
            {
            }

            friend void swap (SpringStep &first, SpringStep &second)
            {
                using std::swap;

                swap (first.strategy, second.strategy);
                swap (first.mesh, second.mesh);
            }

            SpringStep & operator= (SpringStep other)
            {
                swap (*this, other);

                return *this;
            }

            void scale (double x, double y)
            {
                mesh.scale (x, y);
            }

            bool operator () (BezierMesh::MeshArray         &positions,
                              BezierMesh::AnchorArray const &anchors)
            {
                bool more = false;
                mesh.mForces.fill (0.0);
                for (Spring &spring : mesh.mSprings)
                    more |= spring.applyForces (constant);

                more |= strategy (positions, mesh.mForces, anchors, friction);
                return more;
            }

        private:

            double const &constant;
            double const &friction;

            IntegrationStrategy strategy;
            SpringMesh          mesh;
    };

    class NewModel
    {
        public:

            struct Settings
            {
                double springConstant;
                double friction;
                double maximumRange;
            };


            NewModel (Point const &initialPosition,
                      double width,
                      double height,
                      Settings const &settings);
            NewModel (Point const &initialPosition,
                      double width,
                      double height);
            NewModel (NewModel const &other);
            ~NewModel ();

            wobbly::AnchorGrab GrabAnchor (Point const &grab);

            void MoveModelTo (Point const &point);
            void MoveModelBy (Point const &delta);
            void ResizeModel (double width, double height);
            bool StepModel (unsigned int millisecondsDelta);
            Point DeformTexcoords (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            static Settings const DefaultSettings;

            bool Step (unsigned int ms);

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };

    class Model
    {
        public:

            struct Settings
            {
                float springConstant;
                float friction;
                float maximumRange;
            };

            Model (Point const &initialPosition,
                   double width,
                   double height,
                   Settings &settings);
            Model (Point const &initialPosition,
                   double width,
                   double height);
            Model (Model const &other);
            ~Model ();

            typedef PointView <double> DoublePointView;
            typedef std::function <void (DoublePointView &position,
                                         DoublePointView &velocity,
                                         DoublePointView &force)> TransformFunc;
            void TransformClosestObjectToPosition (TransformFunc const &,
                                                   Point const         &);

            wobbly::Object::AnchorGrab GrabAnchor (Point const &grab);

            void MoveModelTo (Point const &point);
            void MoveModelBy (Point const &delta);
            void ResizeModel (double width, double height);
            bool StepModel (unsigned int millisecondsDelta);

            Point DeformTexcoords (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            static constexpr float DefaultSpringConstant = 8.0;
            static constexpr float DefaultObjectRange = 500.0f;

        private:

            class Private;
            std::unique_ptr <Private> priv;

            friend class Private;
    };
}

BOOST_GEOMETRY_REGISTER_POINT_2D (wobbly::PointView <double>,
                                  double,
                                  wobbly::bg::cs::cartesian, x, y)

BOOST_GEOMETRY_REGISTER_POINT_2D_CONST (wobbly::PointView <double const>,
                                        double,
                                        wobbly::bg::cs::cartesian, x, y)

BOOST_GEOMETRY_REGISTER_POINT_2D (wobbly::Point,
                                  double,
                                  wobbly::bg::cs::cartesian, x, y)

#endif
