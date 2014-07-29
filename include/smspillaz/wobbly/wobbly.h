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
#include <type_traits>

#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/register/point.hpp>

namespace wobbly
{
    namespace bg = boost::geometry;

    namespace detail
    {
        struct CopyablePV
        {
            public:

                CopyablePV (CopyablePV const &copy) = default;
                CopyablePV & operator= (CopyablePV const &copy) = default;
        };

        struct NoncopyablePV
        {
            public:

                NoncopyablePV (NoncopyablePV const &copy) = delete;
                NoncopyablePV & operator= (NoncopyablePV const &copy) = delete;
        };

        template <typename NumericType>
        class PointViewCopyabilityBase
        {
            public:
                typedef NumericType NT;
                typedef std::is_const <NumericType> IC;
                typedef std::conditional <IC::value,
                                          CopyablePV,
                                          NoncopyablePV> type;
        };
    }

    template <typename NumericType>
    class PointView :
        public detail::PointViewCopyabilityBase <NumericType>::type
    {
        public:

            typedef NumericType NT;
            typedef typename std::remove_const <NumericType>::type NTM;
            typedef typename std::add_const <NumericType>::type NTC;

            typedef typename std::is_const <NT> IC;

            template <std::size_t N>
            PointView (std::array <NTM, N> &points,
                       std::size_t         index) :
                array (points.data ()),
                offset (index * 2)
            {
            }

            template <std::size_t N,
                      typename = typename std::enable_if <IC::value && N>::type>
            PointView (std::array <NTM, N> const &points,
                       std::size_t               index) :
                array (points.data ()),
                offset (index * 2)
            {
            }

            PointView (NTM         *points,
                       std::size_t index) :
                array (points),
                offset (index * 2)
            {
            }

            PointView (PointView <NumericType> &&view) :
                array (view.array),
                offset (view.offset)
            {
            }

            PointView &
            operator= (PointView <NumericType> &&view) noexcept
            {
                if (this == &view)
                    return *this;

                array = view.array;
                offset = view.offset;

                return *this;
            }

            /* Copy constructor and assignment operator enabled only if the view
             * is truly just observing and has no write access to the point,
             * see detail::PointViewCopyabilityBase */
            PointView (PointView <NumericType> const &p) :
                array (p.array),
                offset (p.offset)
            {
            }

            friend void swap (PointView <NumericType> &first,
                              PointView <NumericType> &second)
            {
                using std::swap;
                swap (first.array, second.array);
                swap (first.offset, second.offset);
            }

            PointView &
            operator= (PointView <NumericType> const &p)
            {
                PointView <NumericType> copy (p);
                swap (*this, copy);
                return *this;
            }

            /* cppcheck thinks this is a constructor, so we need to suppress
             * a constructor warning here.
             */
            // cppcheck-suppress uninitMemberVar
            operator PointView <NTC> ()
            {
                return PointView <NTC> (array, offset);
            }

            template <size_t Offset>
            NumericType const & get () const
            {
                static_assert (Offset < 2, "Offset must be < 2");
                return array[offset + Offset];
            }

            template <size_t Offset>
            typename std::enable_if <!(IC::value) && (Offset + 1), void>::type
            set (NumericType value)
            {
                static_assert (Offset < 2, "Offset must be < 2");
                array[offset + Offset] = value;
            }

        private:

            NumericType *array;
            size_t      offset;
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

BOOST_GEOMETRY_REGISTER_POINT_2D (wobbly::Point,
                                  double,
                                  wobbly::bg::cs::cartesian, x, y)

BOOST_GEOMETRY_REGISTER_POINT_2D_GET_SET (wobbly::PointView <double>,
                                          double,
                                          wobbly::bg::cs::cartesian,
                                          wobbly::PointView <double>::get <0>,
                                          wobbly::PointView <double>::get <1>,
                                          wobbly::PointView <double>::set <0>,
                                          wobbly::PointView <double>::set <1>);

/* Register the const-version of PointView. We are not using the
 * provided macro as non exists for a const-version */
namespace boost
{
    namespace geometry
    {
        namespace traits
        {
            namespace wobbly
            {
                typedef ::wobbly::PointView <double const> DPV;
            }

            BOOST_GEOMETRY_DETAIL_SPECIALIZE_POINT_TRAITS (wobbly::DPV,
                                                           2,
                                                           double,
                                                           cs::cartesian);

            template <>
            struct access <wobbly::DPV, 0>
            {
                static inline double get (wobbly::DPV const &p)
                {
                    return p.get <0> ();
                }
            };

            template <>
            struct access <wobbly::DPV, 1>
            {
                static inline double get (wobbly::DPV const &p)
                {
                    return p.get <1> ();
                }
            };
        }
    }
}

namespace wobbly
{
    class MovableAnchor
    {
        public:

            virtual ~MovableAnchor () = default;
            virtual void MoveBy (Vector const &delta) noexcept (true) = 0;
    };

    typedef std::unique_ptr <MovableAnchor> Anchor;

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

            /* This function will cause a point on the spring mesh closest
             * to grab in absolute terms to become immobile in the mesh.
             *
             * The resulting point can be moved freely by the returned
             * object. Integrating the model after the point has been moved
             * will effectively cause force to be exerted on all the other
             * points. */
            wobbly::Anchor
            GrabAnchor (Point const &grab) throw (std::runtime_error);

            /* This function will insert a new point in the spring
             * mesh which is immobile, with springs from it to its
             * two nearest neighbours.
             *
             * The resulting point can be moved freely by the returned object.
             * As above, integrating the model after moving the point will
             * effectively cause force to be exerted on all other non-immobile
             * points in the mesh */
            wobbly::Anchor
            InsertAnchor (Point const &grab) throw (std::runtime_error);

            /* Performs a single integration per 16 ms in millisecondsDelta */
            bool Step (unsigned int millisecondsDelta);

            /* Takes a normalized texture co-ordinate from 0 to 1 and returns
             * an absolute-position on-screen for that texture co-ordinate
             * as deformed by the model */
            Point DeformTexcoords (Point const &normalized) const;

            /* Bounding box for the model */
            std::array <Point, 4> const Extremes () const;

            /* These functions will attempt to move and resize
             * the model relative to its t position, however,
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

            static Settings DefaultSettings;

        private:

            class Private;
            std::unique_ptr <Private> priv;
    };
}
#endif
