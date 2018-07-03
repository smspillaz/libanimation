/*
 * matchers/mathematical_model_matcher.h
 *
 * Provides utilities to match functions producing single floating
 * point output values for single integer input values to arbitrary
 * mathematical models, for instance, asserting that a function
 * produces values in a linear sequence, or an exponential sequence
 * with a certain (low) error tolerance.
 *
 * Implicitly depends on:
 *  - testing
 *  - std::ostream
 *  - std::function
 *  - std::vector
 *
 * See LICENCE.md for Copyright information
 */
#ifndef WOBBLY_MATHEMATICAL_MODEL_MATCHER_H
#define WOBBLY_MATHEMATICAL_MODEL_MATCHER_H

#include <cassert>                     // for assert
#include <cmath>                        // for log, pow, M_E
#include <cstddef>                     // for size_t

#include <functional>                   // for function, _1, _2, bind
#include <iomanip>                      // for operator<<, setprecision
#include <memory>                       // for unique_ptr
#include <ostream>                      // for ostream, operator<<, etc
#include <type_traits>                  // for enable_if, is_same
#include <vector>                       // for vector

#include <wobbly/wobbly.h>

#include <gmock/gmock.h>       // IWYU pragma: keep

namespace wobbly
{
    namespace models
    {
        namespace impl
        {
            template <typename T>
            std::vector <T>
            CollectForPowerRange (std::function <T (int)> const &f,
                                  int                           min,
                                  int                           max)
            {
                std::vector <T> collection;
                collection.reserve (max - min);

                for (int i = min; i < max; ++i)
                    collection.push_back (f (std::pow (2, i)));

                return collection;
            }
        }

        template <typename NT>
        class DataModel
        {
            public:

                typedef NT NumericType;

                typedef std::unique_ptr <DataModel> Unique;

                typedef wobbly::PointModel <NT> Point;
                typedef std::function <NumericType (int)> Generator;
                typedef std::function <Unique (Generator const &,
                                               unsigned int)> Factory;

                virtual ~DataModel () = default;

                virtual NumericType prediction (NumericType t) const = 0;
                virtual ::std::ostream & report (::std::ostream &os) const = 0;
        };

        namespace exponential
        {
            namespace wgd = ::wobbly::geometry::dimension;

            /* Given a function which generates values from x = 0 -> y = c
             * two data points on that line, and the value c,
             * solve the following equation for the values n and w:
             *
             * y = ne^-wx + c
             *
             * Proof to solve for n:
             *   y(0) = ne^-w(0) + c
             *   y(0) = ne^0 + c
             *   y(0) = n + c
             *   y(0) - c = n
             *
             * Proof to solve for w:
             *   y(1) = ne^-w(1) + c
             *   y(1) = ne^-w + c
             *   y - c = ne^-w
             *   log y - c (ne) = 1 / -w
             *   -w (log y - c (ne)) = 1
             *   -w = 1 / (log y - c (ne))
             *   w = 1 / - (log y - c (ne))
             *
             * Once we have the values for n and w, we can simply plug them
             * into this equation and see if the data points fit the model */
            template <typename N> N
            SolveForECoeff (typename DataModel <N>::Point const &xZeroPoint,
                            N                                   constant)
            {
                N x = wgd::get <0> (xZeroPoint);
                N y = wgd::get <1> (xZeroPoint);

                assert (x == 0.0);

                return y - constant;
            }

            template <typename N> N
            SolveForXCoeff (typename DataModel <N>::Point const &xOnePoint,
                            N                                   eCoefficient,
                            N                                   constant)
            {
                N x = xOnePoint.template get <0> ();
                N y = xOnePoint.template get <1> ();

                assert (x == 1.0f);

                /* Thanks to Phillip Chimento for being awesome at physics
                 * and reminding me that this works */
                N logE (std::log (eCoefficient * M_E));
                N logYC (std::log (y - constant));

                return 1 / (-1 * (logE / logYC));
            }

            template <typename NumericType>
            class DecayModel :
                public DataModel <NumericType>
            {
                public:

                    typedef DataModel <NumericType> Base;

                    DecayModel (NumericType eCoefficient,
                                NumericType xCoefficient,
                                NumericType constant) :
                        mECoefficient (eCoefficient),
                        mXCoefficient (xCoefficient),
                        mConstant (constant)
                    {
                    }

                    NumericType prediction (NumericType t) const
                    {
                        /* ve ^ -wx */
                        return (mECoefficient *
                                (std::pow (M_E,
                                           (-1 * mXCoefficient * t)))) +
                                   mConstant;
                    }

                    ::std::ostream & report (std::ostream &os) const
                    {
                        os << "(exponential decay) f(x) = "
                           << mECoefficient
                           << "e ^ -"
                           << mXCoefficient
                           << "x";

                        return os;
                    }

                    static typename DataModel <NumericType>::Unique
                    Create (typename Base::Generator const &generator,
                            unsigned int                   upperBound,
                            NumericType                    constant)
                    {
                        typedef typename Base::Point Point;
                        typedef typename Base::Unique DataModelUnique;

                        NumericType zero = generator (0);
                        NumericType one = generator (1);

                        NumericType eCoefficient =
                            SolveForECoeff (Point (0.0, zero),
                                                   constant);
                        NumericType xCoefficient =
                            SolveForXCoeff (Point (1.0, one),
                                            eCoefficient,
                                            constant);

                        return DataModelUnique (new DecayModel (eCoefficient,
                                                                xCoefficient,
                                                                constant));
                    }

                private:

                    NumericType mECoefficient;
                    NumericType mXCoefficient;
                    NumericType mConstant;
            };
        }

        namespace linear
        {
            template <typename NumericType>
            class Model :
                public DataModel <NumericType>
            {
                public:

                    typedef DataModel <NumericType> Base;

                    Model (NumericType gradient,
                           NumericType constant) :
                        mGradient (gradient),
                        mConstant (constant)
                    {
                    }

                    NumericType prediction (NumericType t) const
                    {
                        return t * mGradient + mConstant;
                    }

                    ::std::ostream & report (std::ostream &os) const
                    {
                        os << "(linear) f(x) = "
                           << mGradient
                           << "x + "
                           << mConstant;

                        return os;
                    }

                    static typename DataModel <NumericType>::Unique
                    Create (typename Base::Generator const &generator,
                            unsigned int                   upperBound)
                    {
                        typedef typename Base::Unique DataModelUnique;

                        NumericType constant (generator (0));
                        NumericType gradient (generator (1) - constant);

                        return DataModelUnique (new Model (gradient, constant));
                    }

                private:

                    NumericType mGradient;
                    NumericType mConstant;
            };
        }

        namespace parabolic
        {
            template <typename NumericType>
            class Model :
                public DataModel <NumericType>
            {
                public:

                    typedef DataModel <NumericType> Base;

                    Model (NumericType xCoefficient,
                           NumericType xExponent,
                           NumericType constant) :
                        mXCoefficient (xCoefficient),
                        mXExponent (xExponent),
                        mConstant (constant)
                    {
                    }

                    NumericType prediction (NumericType t) const
                    {
                        /* y = mx^z + c */
                        return mXCoefficient *
                                   (std::pow (t, mXExponent))
                                   + mConstant;
                    }

                    ::std::ostream & report (std::ostream &os) const
                    {
                        os << "(parabola) f(x) = "
                           << mXCoefficient
                           << "x ^ "
                           << mXExponent
                           << " + "
                           << mConstant;

                        return os;
                    }

                    static typename DataModel <NumericType>::Unique
                    Create (typename Base::Generator const &generator,
                            unsigned int                   upperBound)
                    {
                        NumericType zero = generator (0);
                        NumericType one = generator (1);
                        NumericType upper = generator (upperBound);

                        /* Given a parabolic model that looks like:
                         *
                         *   f(x) = n * (x ^ m) + c
                         *
                         * Solve for n, m and c;
                         */

                        /* This can be calculated by using the x = 0 data point
                         *
                         * Proof:
                         *
                         *   f(x) = n * (x ^ m) + c
                         *   f(0) = n * (0) + c
                         *   f(0) = c
                         */
                        NumericType constant (zero);

                        /* This can be calculated by using the x = 1 data point
                         *
                         * Proof:
                         *
                         *  f(x) = n * (x ^ m) + c
                         *  f(1) = n * (1 ^ m) + c
                         *  f(1) - c = n
                         */
                        NumericType xCoefficient = one - constant;

                        /* This can be calcuated by using x = upperDataPoint
                         *
                         * Proof:
                         *
                         *  f(x) = n * (x ^ m) + c
                         *  f(x) = n * (x ^ m) + c
                         *  f(x) - c = n * (x ^ m)
                         *  (f(x) - c) / n = (x ^ m)
                         *  ln ((f(x) - c) / n) = ln (x ^ m)
                         *  ln ((f(x) - c) / n) = m * ln (x)
                         *  ln ((f(x) - c) / n) / ln (x) = m
                         *
                         *  (substituting for x)
                         *
                         *  ln ((f(u) - c) / n) / ln (upperDataPoint) = m
                         */
                        NumericType logUpper = std::log (upperBound);
                        NumericType logFUpperMinusCOverN =
                            std::log ((upper - constant) / xCoefficient);

                        NumericType xExponent = logFUpperMinusCOverN / logUpper;

                        typedef typename Base::Unique DataModelUnique;

                        return DataModelUnique (new Model (xCoefficient,
                                                           xExponent,
                                                           constant));
                    }

                private:

                    NumericType mXCoefficient;
                    NumericType mXExponent;
                    NumericType mConstant;
            };
        }

        template <typename N>
        class ExponentialDecayTowards
        {
            public:

                typedef DataModel <N> Model;

                ExponentialDecayTowards (N towardsConstant) :
                    mTowardsConstant (towardsConstant)
                {
                }

                typename Model::Factory Create (N towardsConstant) const
                {
                    typedef typename exponential::DecayModel <N> Model;
                    return std::bind (Model::Create,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      towardsConstant);
                }

            private:

                N mTowardsConstant;
        };

        template <typename NumericType>
        struct Linear
        {
            typedef DataModel <NumericType> Model;

            typename Model::Factory Create () const
            {
                return linear::Model <NumericType>::Create;
            }
        };

        template <typename NumericType>
        struct Parabolic
        {
            typedef DataModel <NumericType> Model;

            typename Model::Factory Create () const
            {
                return parabolic::Model <NumericType>::Create;
            }
        };
    }

    namespace testing
    {
        template <typename T, typename U, typename V>
        bool close_at_tolerance (T const &lhs, U const &rhs, V const &epsilon)
        {
            return std::fabs (rhs - lhs) < epsilon;
        }

        template <typename T, typename U>
        bool is_small (T const &value, U const &epsilon)
        {
            return close_at_tolerance (value, std::numeric_limits <T>::min(), epsilon);
        }
    }

    namespace matchers
    {
        namespace t = ::testing;
        namespace wgd = ::wobbly::geometry::dimension;

        template <typename GeometryType, typename Compare>
        class GeometricallyEqualMatcher :
            public t::MatcherInterface <GeometryType const &>
        {
            public:

                GeometricallyEqualMatcher (Compare const &geometry) :
                    geometry (geometry)
                {
                }

                bool MatchAndExplain (GeometryType     const &x,
                                      t::MatchResultListener *listener) const
                {
                    /* The right hand side is significant since we have
                     * slightly different behaviour where rhs == 0 */
                    return compare (x, geometry);
                }

                void DescribeTo (std::ostream *os) const
                {
                    *os << "geometrically equal to " << geometry;
                }

            private:

                template <typename LHS, typename RHS, size_t D, size_t Cnt>
                struct Equal
                {
                    static inline bool apply (LHS const &lhs,
                                              RHS const &rhs)
                    {
                        const float epsilon = 10e-7;
                        typedef typename wgd::Dimension <LHS>::data_type LT;

                        bool result = false;

                        /* If we are comparing to zero, then use
                         * check_is_small as opposed to check_is_close */
                        if (wgd::get <D> (rhs) == 0.0)
                            result = wobbly::testing::is_small (wgd::get <D> (lhs),
                                                                epsilon);
                        else
                        {
                            result = wobbly::testing::close_at_tolerance <LT> (wgd::get <D> (lhs),
                                                                               wgd::get <D> (rhs),
                                                                               epsilon);
                        }

                        return result &&
                               Equal <LHS, RHS, D + 1, Cnt>::apply (lhs, rhs);
                    }
                };

                template <typename LHS, typename RHS, size_t Cnt>
                struct Equal <LHS, RHS, Cnt, Cnt>
                {
                    static inline bool apply (LHS const &lhs, RHS const &rhs)
                    {
                        return true;
                    }
                };

                template <typename LHS, typename RHS>
                static inline bool compare (LHS const &lhs, RHS const &rhs)
                {
                    typedef wgd::Dimension <LHS> DimLHS;
                    typedef wgd::Dimension <RHS> DimRHS;

                    typedef Equal <LHS, RHS, 0, DimLHS::dimensions> Comparator;

                    return Comparator::apply (lhs, rhs);
                }

                Compare geometry;
        };

        /* We want to be able to pass any compatible wobbly::dimension to this
         * matcher so it needs to be a template class with a template
         * conversion operator to Matcher <T> (eg, to signify that it is
         * T that is being matched, but Geometry that is being compared) */
        template <typename Geom>
        class GeometricallyEqualHelper
        {
            public:

                GeometricallyEqualHelper (Geom const &geometry) :
                    geometry (geometry)
                {
                }

                template <typename T>
                operator t::Matcher <T> () const
                {
                    typedef GeometricallyEqualMatcher <T, Geom> EqualMatcher;
                    return t::MakeMatcher (new EqualMatcher (geometry));
                }

            private:

                Geom const &geometry;
        };

        template <typename T>
        class NormalEqHelper
        {
            public:

                NormalEqHelper (T const &matchee) :
                    matchee (matchee)
                {
                }

                operator t::Matcher <T const &> () const
                {
                    return t::Eq (matchee);
                }

            private:

                T const &matchee;
        };

        template <typename Geom>
        GeometricallyEqualHelper <Geom> GeometricallyEqual (Geom const &geom)
        {
            return GeometricallyEqualHelper <Geom> (geom);
        }

        namespace detail
        {
            template <typename T>
            struct Geo
            {
                static constexpr bool value =
                    !(std::is_same <typename wgd::Dimension <T>::data_type, void>::value);
            };

            template <typename T>
            struct IsGeometry :
                public std::enable_if <Geo <T>::value>
            {
            };

            template <typename T>
            struct IsNotGeometry :
                public std::enable_if <!Geo <T>::value>
            {
            };

            template <typename T, typename = void>
            struct EqDispatch
            {
            };

            template <typename T>
            struct EqDispatch <T,
                               typename IsNotGeometry <T>::type>
            {
                typedef NormalEqHelper <T> Factory;
            };

            template <typename T>
            struct EqDispatch <T,
                               typename IsGeometry <T>::type>
            {
                typedef GeometricallyEqualHelper <T> Factory;
            };

            template <typename Matchee, typename Factory>
            class EqDispatchHelper
            {
                public:

                    EqDispatchHelper (Matchee const &matchee) :
                        mFactory (matchee)
                    {
                    }

                    template <typename T>
                    operator t::Matcher <T const &> () const
                    {
                        return mFactory;
                    }

                    Factory mFactory;
            };
            template <typename Matchee, typename Factory>
            EqDispatchHelper <Matchee, Factory>
            EqDispatcher (Matchee const &matchee)
            {
                return EqDispatchHelper <Matchee, Factory> (matchee);
            }
        }

        template <typename T>
        detail::EqDispatchHelper <T, typename detail::EqDispatch <T>::Factory>
        Eq (T const &matchee)
        {
            typedef typename detail::EqDispatch <T>::Factory Factory;
            return detail::EqDispatcher <T, Factory> (matchee);
        }

        template <typename NumericType>
        class MathematicalModelMatcher :
            public t::MatcherInterface <std::function <NumericType (int)>>
        {
            public:

                typedef models::DataModel <NumericType> Model;

                typedef typename Model::Unique ModelUnique;
                typedef typename Model::Generator Generator;
                typedef typename Model::Factory ModelFactory;

                MathematicalModelMatcher (ModelFactory const &factory,
                                          unsigned int       nSamples,
                                          double             tolerance) :
                    mModelFactory (factory),
                    mNSamples (nSamples),
                    mTolerance (tolerance)
                {
                }

                bool MatchAndExplain (Generator              function,
                                      t::MatchResultListener *listener) const
                {
                    namespace mi = models::impl;

                    /* Collect data points on the provided user function based
                     * on powers-of-two. This very quickly ensures that we get
                     * very large values to match against */
                    auto collection (mi::CollectForPowerRange (function,
                                                               0,
                                                               mNSamples));
                    ModelUnique model (mModelFactory (function,
                                                      std::pow (2, mNSamples)));
                    ::std::ostream *os = listener->stream ();
                    bool returnValue = true;

                    model->report (mModelReport);

                    /* Now we can just plug every x value into our collection
                     * and check if the difference between the y value falls
                     * within the tolerable error range */
                    unsigned int const precision =
                        static_cast <int> (log (1.0 / (mTolerance)) / log (10));

                    for (size_t i = 0; i < collection.size (); ++i)
                    {
                        NumericType modelPrediction =
                            model->prediction (std::pow (2, i));
                        NumericType actual = collection[i];

                        if (os)
                            *os << std::endl
                                << std::setprecision (precision)
                                << "x value: "
                                << i
                                << ", prediction: "
                                << modelPrediction
                                << ", actual: "
                                << actual
                                << std::endl;

                        /* Close to specified tolerance */
                        if (!wobbly::testing::close_at_tolerance (modelPrediction,
                                                                  actual,
                                                                  mTolerance))
                        {
                            if (os)
                                *os << " (exceeding tolerance: "
                                    << mTolerance
                                    << ")";

                            returnValue = false;
                        }
                    }

                    return returnValue;
                }

                void DescribeTo (std::ostream *os) const
                {
                    if (os)
                    {
                        *os << "satisfies the model: "
                            << mModelReport.str ();
                    }
                }

                void DescribeNegationTo (std::ostream *os) const
                {
                    if (os)
                    {
                        *os << "doesn't satisfy the model: "
                            << mModelReport.str ();
                    }
                }

            private:

                ModelFactory mModelFactory;
                unsigned int mNSamples;
                double       mTolerance;

                mutable std::stringstream mModelReport;
        };

        /* By default we take thirty samples on an exponential curve.
         *
         * This ensures that we get absurdly high x-values (eg,
         * 2^30 is close to 10^9). However, ths is only provided
         * as a default. When modelling functions which operate on
         * much smaller units there's a chance that the original function
         * will fall flat after exceeding the epsilon and that our model
         * might keep holding up. As such, we should allow the user
         * to customize the number of samples */
        static constexpr unsigned int NSamples = 30;

        /* Tolerance is to nine decimal places by default. This might not
         * fit so well when numbers become excessively high because
         * floating point rounding errors can tend to compound themselves.
         * As such we also allow the user to customize this value */
        static constexpr double Tolerance = 10e-9;

        class WithSamples
        {
            public:

                WithSamples (unsigned int n) :
                    nSamples (n)
                {
                }

                unsigned int nSamples;
        };

        class WithTolerance
        {
            public:

                WithTolerance (double tolernace) :
                    tolernace (tolernace)
                {
                }

                double tolernace;
        };

        namespace m = models;

        template <typename FactoryFunc>
        inline t::Matcher <typename FactoryFunc::Model::Generator>
        SatisfiesModel (FactoryFunc const &factory,
                        WithSamples       sm = NSamples,
                        WithTolerance to = Tolerance)
        {
            auto modelFactory (factory.Create ());
            typedef typename FactoryFunc::Model::NumericType NumericType;
            typedef MathematicalModelMatcher <NumericType> ModelMatcher;
            return t::MakeMatcher (new ModelMatcher (modelFactory,
                                                     sm.nSamples,
                                                     to.tolernace));
        }
    }
}


#endif
