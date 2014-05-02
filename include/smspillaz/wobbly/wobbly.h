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
#include <memory>
#include <vector>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace wobbly
{
    namespace bg = boost::geometry;

    template <typename NumericType>
    struct PointView
    {
        typedef typename std::remove_const <NumericType>::type NumericTypeMut;

        template <std::size_t N>
        PointView (std::array <NumericTypeMut, N> &points,
                   unsigned int                   index) :
            x (points.at (index * 2)),
            y (points.at (index * 2 + 1))
        {
        }

        PointView (std::vector <NumericTypeMut> &points,
                   unsigned int                 index) :
            x (points.at (index * 2)),
            y (points.at (index * 2 + 1))
        {
        }

        PointView (PointView &&view) :
            x (view.x),
            y (view.y)
        {
        }

        PointView (PointView <NumericType> const &p) = delete;
        PointView & operator= (PointView <NumericType> const &p) = delete;

        NumericType &x;
        NumericType &y;
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
            virtual Vector DeltaTo (Vector const &) = 0;
    };

    class Object
    {
        public:

            Object (PointView <double>       &&position,
                    PointView <double>       &&velocity,
                    PointView <double const> &&force,
                    PointView <double const> &&immediateMovement) noexcept;
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

            bool ApplyForce (Vector const &force);
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
                    PointView <double> &&immediateA,
                    PointView <double> &&immediateB,
                    Vector distance);
            Spring (Spring &&spring) noexcept;
            ~Spring ();

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            bool applyForces (float springConstant);
            bool applyForces (float springConstant, float forceRatio);
            void scaleLength (Vector scaleFactor);

            static constexpr double ClipThreshold = 1.0;

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
            static constexpr unsigned int TotalIndices = Width * Height * 2;

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            /* Direct access to the points in this mesh is permitted.
             *
             * PointForIndex is just a convenience function to get a PointView
             * by an x, y index.
             *
             * PointArray gets the entire array at once and should be used
             * where the array is being accessed sequentially */
            PointView <double> PointForIndex (unsigned int width,
                                              unsigned int height);

            std::array <double, TotalIndices> & PointArray ();

            PointView <double const> PointForIndex (unsigned int width,
                                                    unsigned int height) const;

            std::array <double, TotalIndices> const & PointArray () const;

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
            };

            Model (Point const &initialPosition,
                   float width,
                   float height,
                   Settings &settings);
            Model (Point const &initialPosition,
                   float width,
                   float height);
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
            void ResizeModel (float width, float height);
            bool StepModel (unsigned int millisecondsDelta);

            Point DeformTexcoords (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;

            static constexpr float DefaultSpringConstant = 8.0;

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
