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
            x(*((&points[0]) + index * 2)),
            y(*((&points[0]) + index * 2 + 1))
        {
            assert (index * 2 < points.size ());
        }

        PointView (PointView &&view) :
            x(view.x),
            y(view.y)
        {
        }

        PointView (const PointView<NumericType> &p) = delete;
        PointView & operator= (const PointView<NumericType> &p) = delete;

        NumericType &x;
        NumericType &y;
    };

    typedef bg::model::point<double, 2, bg::cs::cartesian> Vector;
    typedef bg::model::point<double, 2, bg::cs::cartesian> Point;

    class Object
    {
        public:

            Object(PointView<double> &&position) noexcept;
            Object(Point const &position) noexcept;
            Object(Object &&object) noexcept;
            Object() noexcept;

            Object (const Object &object) = delete;
            Object & operator= (const Object &) = delete;

            void Step(float time);

            static float Mass;
            static float Friction;

            void Lock();
            void Unlock();

            bool IsAnchor();

        private:

            bool                 mIsAnchor;
            std::vector <double> mInternalPoint;

        public:

            Vector force;
            Vector velocity;
            PointView<double> position;
    };

    class Spring
    {
        public:

            Spring ();
            Spring (Object &a,
                    Object &b,
                    Vector distance);

            void applyForces (float springConstant);

            static constexpr double ClipThreshold = 0.0005;

    private:

            /* This is a stupid workaround for the fact that Spring is
             * really an object type, but I'd much rather not use
             * heap allocation for it */
            static Object static_a, static_b;

            Object &a;
            Object &b;
            Vector desiredDistance;
    };

    Object Spring::static_a;
    Object Spring::static_b;

    class BezierMesh
    {
        public:

            BezierMesh();

            Point DeformUnitCoordsToMeshSpace (Point const &normalized) const;
            PointView<double> PointForIndex (unsigned int width,
                                             unsigned int height);

        private:

            std::vector <double> mPoints;
    };

    class Model
    {
        public:

            Model (Point const &initialPosition,
                   float width,
                   float height,
                   unsigned int gridWidth,
                   unsigned int gridHeight);

            void GrabAnchor (Point const &grab);
            void UngrabAnchor ();

            void MoveModelTo (Point const &point);
            void StepModel (float time);

            Point DeformTexcoords (Point const &normalized);

        private:

            BezierMesh mMesh;

            Point mLastPosition;

            float mWidth, mHeight;
            unsigned int mGridWidth, mGridHeight;

            std::vector<Object> mObjects;
            std::vector<Spring> mSprings;
            Object *mAnchorObject;
    };
}

BOOST_GEOMETRY_REGISTER_POINT_2D(wobbly::PointView<double>, double, wobbly::bg::cs::cartesian, x, y)

#endif
