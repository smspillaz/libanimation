/*
 * include/wobbly.h
 *
 * C++ Interface for "wobbly" textures
 *
 * Implicitly depends on:
 *  - std::array
 *  - boost::geometry
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_H
#define WOBBLY_H

#include <array>
#include <memory>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

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

namespace wobbly
{
    class Anchor
    {
        public:

             class Storage
             {
                 public:

                     virtual ~Storage () {};
                     virtual void Lock (size_t index) = 0;
                     virtual void Unlock (size_t index) = 0;
             };

             Anchor (wobbly::PointView <double> &&point,
                     Storage                    &storage,
                     size_t                     index);
             ~Anchor ();
             Anchor (Anchor &&);

             void MoveBy (Point const &delta);

        private:

            Anchor (const Anchor &) = delete;
            Anchor & operator= (const Anchor &) = delete;

            struct Private;
            std::unique_ptr <Private> priv;
    };

    class Model
    {
        public:

            struct Settings
            {
                double springConstant;
                double friction;
                double maximumRange;
            };

            Model (Point const &initialPosition,
                   double width,
                   double height,
                   Settings const &settings);
            Model (Point const &initialPosition,
                   double width,
                   double height);
            Model (Model const &other);
            ~Model ();

            /* This function will cause a point on the sprint mesh closest
             * to grab in absolute terms to become immobile in the mesh.
             *
             * The resulting point can be moved freely by the returned
             * object. Integrating the model after the point has been moved
             * will effectively cause force to be exerted on all the other
             * points. */
            wobbly::Anchor GrabAnchor (Point const &grab);

            /* Performs a single integration per 16 ms in millisecondsDelta */
            bool Step (unsigned int millisecondsDelta);

            /* Takes a normalized texture co-ordinate from 0 to 1 and returns
             * an absolute-position on-screen for that texture co-ordinate
             * as deformed by the model */
            Point DeformTexcoords (Point const &normalized) const;

            /* Bounding box for the model */
            std::array <Point, 4> const Extremes () const;

            /* These functions will attempt to move and resize
             * the model relative to its target position, however,
             * caution should be exercised when using them.
             *
             * A full integration until the model has reached equillibrium
             * may need to be performed in order to determine the target
             * position and given the nature of the calculations, there may
             * be some error in determining that target position. That may
             * affect the result of these operations to a slight degree.
             *
             * If a precise position is required, then the recommended course
             * of action is to destroy and re-create the model. */
            void MoveModelTo (Point const &point);
            void MoveModelBy (Point const &delta);
            void ResizeModel (double width, double height);

            static constexpr double DefaultSpringConstant = 8.0;
            static constexpr double DefaultObjectRange = 500.0f;
            static constexpr double Mass = 15.0f;
            static constexpr double Friction = 3.0f;

            static Settings const DefaultSettings;

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };
}
#endif
