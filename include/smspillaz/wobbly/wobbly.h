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
        PointView (std::vector <NumericType> &points,
                   unsigned int              index) :
            x (*((&points[0]) + index * 2)),
            y (*((&points[0]) + index * 2 + 1))
        {
            assert (index * 2 < points.size ());
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
    };

    class Object
    {
        public:

            explicit Object (PointView <double> &&position,
                             PointView <double> &&velocity,
                             PointView <double> &&force) noexcept;
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

                    AnchorGrab (ImmediatelyMovablePosition &position,
                                unsigned int               &lockCount);
                    ~AnchorGrab ();

                private:

                    AnchorGrab (const AnchorGrab &) = delete;
                    AnchorGrab & operator= (const AnchorGrab &) = delete;

                    ImmediatelyMovablePosition &position;
                    unsigned int               &lockCount;
            };

            AnchorGrab Grab ();
            PointView <double> const & Position () const;

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };

    class Spring
    {
        public:

            Spring (Object &a,
                    Object &b,
                    Vector distance);
            Spring (Spring &&spring) noexcept;
            ~Spring ();

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            bool applyForces (float springConstant);
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

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            std::array <Point, 4> const Extremes () const;
            PointView <double> PointForIndex (unsigned int width,
                                              unsigned int height) const;

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

BOOST_GEOMETRY_REGISTER_POINT_2D (wobbly::Point,
                                  double,
                                  wobbly::bg::cs::cartesian, x, y)

#endif
