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

    typedef bg::model::point <double, 2, bg::cs::cartesian> Vector;
    typedef bg::model::point <double, 2, bg::cs::cartesian> Point;

    class Object
    {
        public:

            Object (PointView <double> &&position) noexcept;
            Object (Point const &position) noexcept;
            Object (Object &&object) noexcept;
            Object ();

            Object (Object const &object) = delete;
            Object & operator= (Object const &) = delete;

            void Step (float time);

            static float Mass;
            static float Friction;

            void Lock ();
            void Unlock ();

            bool IsAnchor ();

        private:

            bool                 mIsAnchor;
            std::vector <double> mInternalPoint;

        public:

            Vector force;
            Vector velocity;
            PointView <double> position;
    };

    class Spring
    {
        public:

            Spring (Object &a,
                    Object &b,
                    Vector distance);
            Spring (Spring &&spring) noexcept;

            Spring (Spring const &spring) = delete;
            Spring & operator= (Spring const &spring) = delete;

            void applyForces (float springConstant);

            static constexpr double ClipThreshold = 0.0005;

        private:

            Object &a;
            Object &b;
            Vector desiredDistance;
    };

    class BezierMesh
    {
        public:

            BezierMesh ();

            static constexpr unsigned int Width = 4;
            static constexpr unsigned int Height = 4;

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            PointView <double> PointForIndex (unsigned int width,
                                              unsigned int height);

        private:

            std::vector <double> mPoints;
    };

    class Model
    {
        public:

            Model (Point const &initialPosition,
                   float width,
                   float height);

            void GrabAnchor (Point const &grab);
            void UngrabAnchor ();

            void MoveModelTo (Point const &point);
            void StepModel (float time);

            Point DeformTexcoords (Point const &normalized);

            static constexpr float SpringConstant = 0.4;

        private:

            BezierMesh mMesh;

            Point mLastPosition;

            float mWidth, mHeight;

            std::vector <Object> mObjects;
            std::vector <Spring> mSprings;
            Object *mAnchorObject;
    };
}

BOOST_GEOMETRY_REGISTER_POINT_2D (wobbly::PointView <double>,
                                  double,
                                  wobbly::bg::cs::cartesian, x, y)

#endif
