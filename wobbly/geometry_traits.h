/*
 * wobbly/geometry_traits.h
 *
 * Provides geometric trait mutation functions
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_GEOMETRY_TRAITS_H
#define WOBBLY_GEOMETRY_TRAITS_H

#include <cmath>

namespace wobbly
{
    namespace geometry
    {
        namespace dimension
        {
            template <typename T>
            struct Dimension
            {
                typedef int data_type;
                static const size_t dimensions = 0;
            };

            template <typename T, size_t D>
            struct DimensionAccess
            {
                static inline void set (T &p, typename Dimension<T>::data_type value)
                {
                }

                static inline typename Dimension<T>::data_type get (T const &)
                {
                    return typename Dimension <T>::data_type ();
                }
            };

            template <size_t D, typename T>
            inline void set (T &point, typename Dimension <T>::data_type const &value)
            {
                DimensionAccess <T, D>::set (point, value);
            }

            template <size_t D, typename T>
            inline typename Dimension <T>::data_type get (T const &point)
            {
                return DimensionAccess <T, D>::get (point);
            }

            namespace detail
            {
                template <typename T, typename Visitor, size_t D>
                struct MutateRecursive
                {
                    static void apply (T &point, Visitor &&v)
                    {
                        set <D> (point, v (get <D> (point)));
                        MutateRecursive <T, Visitor, D - 1>::apply (point,
                                                                    std::forward <Visitor> (v));
                    }
                };

                template <typename T, typename Visitor>
                struct MutateRecursive <T, Visitor, 0>
                {
                    static void apply (T &point, Visitor &&v)
                    {
                        set <0> (point, v (get <0> (point)));
                    }
                };

                template <typename T, typename U, typename Visitor, size_t D>
                struct VectorApplyOntoRecursive
                {
                    static void apply (T &dst, U const &src, Visitor &&v)
                    {
                        set <D> (dst, v (get <D> (dst), get <D> (src)));
                        VectorApplyOntoRecursive <T, U, Visitor, D - 1>::apply (dst,
                                                                                src,
                                                                                std::forward <Visitor> (v));
                    }
                };

                template <typename T, typename U, typename Visitor>
                struct VectorApplyOntoRecursive <T, U, Visitor, 0>
                {
                    static void apply (T &dst, U const &src, Visitor &&v)
                    {
                        set <0> (dst, v (get<0> (dst), get <0> (src)));
                    }
                };

                template <typename T, typename U, typename Visitor, size_t D>
                struct ScalarApplyOntoRecursive
                {
                    static void apply (T &dst, U const &src, Visitor &&v)
                    {
                        set <D> (dst, v (get <D> (dst), src));
                        ScalarApplyOntoRecursive <T, U, Visitor, D - 1>::apply (dst,
                                                                                src,
                                                                                std::forward <Visitor> (v));
                    }
                };
                template <typename T, typename U, typename Visitor>
                struct ScalarApplyOntoRecursive <T, U, Visitor, 0>
                {
                    static void apply (T &dst, U const &src, Visitor &&v)
                    {
                        set <0> (dst, v (get <0> (dst), src));
                    }
                };

                template <typename T, typename U, typename A, typename Visitor, size_t D>
                struct ReduceCoordinatePairsRecursive
                {
                    static A apply (T const &l,
                                    U const &r,
                                    A       &&acc,
                                    Visitor &&v)
                    {
                        typedef ReduceCoordinatePairsRecursive <T, U, A, Visitor, D - 1> C;
                        return C::apply (l,
                                         r,
                                         v (get <D> (l), get <D> (r),
                                         std::forward <A> (acc)),
                                         std::forward <Visitor> (v));
                    }
                };

                template <typename T, typename U, typename A, typename Visitor>
                struct ReduceCoordinatePairsRecursive <T, U, A, Visitor, 0>
                {
                    static A apply (T const &l,
                                    U const &r,
                                    A       &&acc,
                                    Visitor &&v)
                    {
                        return v (get <0> (l),
                                  get <0> (r),
                                  std::forward <A> (acc));
                    }
                };
            }

            /**
             * for_each_coordinate:
             *
             * Apply Visitor to each dimension of T, passing a mutable
             * reference. Applicable to types implementing the Dimension trait.
             */
            template <typename T, typename Visitor>
            inline void for_each_coordinate (T &point, Visitor &&v)
            {
                const size_t Dim = Dimension <T>::dimensions - 1;
                detail::MutateRecursive <T, Visitor, Dim>::apply (point,
                                                                  std::forward <Visitor> (v));
            }

            /**
             * vector_map_onto:
             *
             * Apply Visitor to each dimension of (T, U) tuples, copying the
             * result onto T. Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U, typename Visitor>
            inline void vector_map_onto (T &dst, U const &src, Visitor &&v)
            {
                const size_t Dim = Dimension <T>::dimensions - 1;
                detail::VectorApplyOntoRecursive <T, U, Visitor, Dim>::apply (dst,
                                                                              src,
                                                                              std::forward <Visitor> (v));
            }

            /**
             * scalar_map_onto:
             *
             * Apply Visitor to each dimension of T, passing tuples of (T, U)
             * where U repeats with the same value in each application, copying the
             * result onto T. Applicable to types implementing the Dimension trait.
             */
            template <typename T, typename U, typename Visitor>
            inline void scalar_map_onto (T &dst, U const &src, Visitor &&v)
            {
                const size_t Dim = Dimension <T>::dimensions - 1;
                detail::ScalarApplyOntoRecursive <T, U, Visitor, Dim>::apply (dst,
                                                                              src,
                                                                              std::forward <Visitor> (v));
            }

            /**
             * vector_reduce:
             *
             * Apply Visitor to each dimension of T and U, passing tuples of (T, U, A)
             * where A is an "accumulator" value being the result of the previous
             * application. Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U, typename A, typename Visitor>
            inline A vector_reduce (T const &l, U const &r, A &&initial, Visitor &&v)
            {
                const size_t Dim = Dimension <T>::dimensions - 1;
                return detail::ReduceCoordinatePairsRecursive<T, U, A, Visitor, Dim>::apply (l,
                                                                                             r,
                                                                                             std::forward <A> (initial),
                                                                                             std::forward <Visitor> (v));
            }

            /**
             * assign:
             *
             * Assign all the dimensions of U to T. Applicable to types
             * implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void assign (T &dst, U const &src)
            {
                vector_map_onto (dst,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return r;
                                 });
            }

            /**
             * assign_value:
             *
             * Assign all dimensions of T to the value of U.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void assign_value (T &dst, U const &src)
            {
                scalar_map_onto (dst,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype (auto) {
                                     return r;
                                 });
            }

            /**
             * scale:
             *
             * Scale all dimensions of U by the value of T.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename F>
            inline void scale (T &dst, F factor)
            {
                scalar_map_onto (dst,
                                 factor,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return l * r;
                                 });
            }

            /**
             * pointwise_scale:
             *
             * Scale all dimensions of T by each dimension in U.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void pointwise_scale (T &dest, U const &src)
            {
                vector_map_onto (dest,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return l * r;
                                 });
            }

            /**
             * pointwise_divide:
             *
             * Divide all dimensions of T by each dimension in U.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void pointwise_div (T &dest, U const &src)
            {
                vector_map_onto (dest,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return l / r;
                                 });
            }

            /**
             * pointwise_add:
             *
             * Add each dimension of U to T and store in T.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void pointwise_add (T &dst, U const &src)
            {
                vector_map_onto (dst,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return l + r;
                                 });
            }

            /**
             * pointwise_add:
             *
             * Subtract each dimension of U from T and store in T.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline void pointwise_subtract (T &dst, U const &src)
            {
                vector_map_onto (dst,
                                 src,
                                 [](auto const &l, auto const &r) -> decltype(auto) {
                                     return l - r;
                                 });
            }

            /**
             * distance:
             *
             * Compute cartesian distance between T and U.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline double distance (T const &l, U const &r)
            {
                return ::sqrt (vector_reduce (l,
                                              r,
                                              0.0, [](auto const &l, auto const &r, auto const &acc) -> decltype(auto) {
                                                  auto const delta = l - r;
                                                  return acc + (delta * delta);
                                              }));
            }

            /**
             * equals:
             *
             * True if each dimension of T is equal to the
             * corresponding dimension of U.
             * Applicable to type implementing the Dimension trait.
             */
            template <typename T, typename U>
            inline bool equals (T const &l, U const &r)
            {
                return ::sqrt (vector_reduce (l,
                                              r,
                                              true,
                                              [](auto const &l, auto const &r, auto const &acc) -> decltype(auto) {
                                                  return acc && (l == r);
                                              }));
            }

            /**
             * for_each_point:
             *
             * Apply Visitor to each point in T.
             */
            template <typename T, typename Visitor>
            inline void for_each_point (T const &container, Visitor &&visitor)
            {
                container.apply_visitor (std::forward <Visitor> (visitor));
            }
        }

    }
}

#endif
