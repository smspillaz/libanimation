/*
 * tests/wobbly_test.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <functional>
#include <iostream>
#include <gmock/gmock.h>
#include <boost/geometry/algorithms/assign.hpp>
#include <boost/geometry/util/for_each_coordinate.hpp>

#include <mathematical_model_matcher.h>

#include <smspillaz/wobbly/wobbly.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::ElementsAreArray;
using ::testing::Eq;
using ::testing::Invoke;
using ::testing::MakeMatcher;
using ::testing::Matcher;
using ::testing::MatcherCast;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;
using ::testing::Not;
using ::testing::Test;
using ::testing::ValuesIn;
using ::testing::WithParamInterface;

using ::wobbly::matchers::GeometricallyEqual;
using ::wobbly::matchers::SatisfiesModel;
using ::wobbly::matchers::WithSamples;
using ::wobbly::matchers::WithToleranace;

using ::wobbly::models::ExponentialDecayTowards;
using ::wobbly::models::Linear;
using ::wobbly::models::Parabolic;

namespace bg = boost::geometry;

namespace boost
{
    namespace geometry
    {
        namespace model
        {
            template <typename C, std::size_t D, typename S>
            std::ostream &
            operator<< (std::ostream &lhs, point <C, D, S> const &p)
            {
                return lhs << std::setprecision (10)
                           << "x: "
                           << bg::get <0> (p)
                           << " y: "
                           << bg::get <1> (p);
            }
        }
    }
}

namespace wobbly
{
    std::ostream &
    operator<< (std::ostream &lhs, Point const &p)
    {
        return lhs << std::setprecision (10)
                   << "x: "
                   << bg::get <0> (p)
                   << " y: "
                   << bg::get <1> (p);
    }

    template <typename NumericType>
    std::ostream &
    operator<< (std::ostream &lhs, PointView <NumericType> const &p)
    {
        Point point;
        bg::assign (point, p);
        return lhs << point;
    }
}

namespace
{
    class SingleObjectStorage
    {
        public:

            SingleObjectStorage ()
            {
                storage.fill (0);
            }

            wobbly::PointView <double> Position ()
            {
                return wobbly::PointView <double> (storage, 0);
            }

            wobbly::PointView <double> Velocity ()
            {
                return wobbly::PointView <double> (storage, 1);
            }

            wobbly::PointView <double> Force ()
            {
                return wobbly::PointView <double> (storage, 2);
            }

        private:

            std::array <double, 6> storage;
    };

    class SingleObjectStorageView
    {
        public:

            SingleObjectStorageView (SingleObjectStorage &storage) :
                position (storage.Position ()),
                velocity (storage.Velocity ()),
                force (storage.Force ())
            {
            }

            wobbly::PointView <double> position;
            wobbly::PointView <double> velocity;
            wobbly::PointView <double> force;
    };

    class LockedObject :
        public ::testing::Test
    {
        public:

            wobbly::Point const Position = wobbly::Point (100, 50);

            LockedObject () :
                view (storage),
                object (storage.Position (),
                        storage.Velocity (),
                        storage.Force (),
                        wobbly::Point (0, 0)),
                grab (new wobbly::Object::AnchorGrab (object.Grab ()))
            {
                bg::assign_point (view.position, Position);
            }

        protected:

            SingleObjectStorage storage;
            SingleObjectStorageView view;
            wobbly::Object object;
            std::unique_ptr <wobbly::Object::AnchorGrab> grab;
    };

    TEST_F (LockedObject, VelocityResetOnStep)
    {
        bg::assign_point (view.velocity, wobbly::Vector (1, 1));
        object.Step (10);

        EXPECT_THAT (view.velocity,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    TEST_F (LockedObject, VelocityAffectedAfterUnlocked)
    {
        wobbly::Vector initialVelocity (1, 1);
        bg::assign_point (view.velocity, initialVelocity);
        grab.reset ();
        object.Step (1);

        EXPECT_THAT (view.velocity,
                     Not (GeometricallyEqual (initialVelocity)));
    }

    /* TODO: I'd like to be able to test here whether or not
     * the velocity function with friction applied has a derivative
     * with a lower slope than the velocity function without friction
     * applied */
    TEST (Object, FrictionVelocityAppliedToPoint)
    {
        std::function <double (int)> horizontalVelocityFunction =
            [](int timestep) -> double {
                SingleObjectStorage storage;
                SingleObjectStorageView view (storage);

                bg::assign_point (view.position, wobbly::Point (1, 1));
                bg::assign_point (view.velocity, wobbly::Point (1, 0));

                wobbly::Object object (storage.Position (),
                                       storage.Velocity (),
                                       storage.Force (),
                                       wobbly::Point (1, 1));
                object.Step (timestep);

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (horizontalVelocityFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    TEST (Object, MoveConstructorNoThrow)
    {
        SingleObjectStorage storage;
        wobbly::Object a (storage.Position (),
                          storage.Velocity (),
                          storage.Force (),
                          wobbly::Point (1, 1));
        EXPECT_NO_THROW ({
            wobbly::Object b (std::move (a));
        });
    }

    class FrictionlessObject :
        public Test
    {
        protected:

            FrictionlessObject () :
                view (storage),
                object (storage.Position (),
                        storage.Velocity (),
                        storage.Force (),
                        wobbly::Point (1, 1)),
                mOldFriction (wobbly::Object::Friction)
            {
                wobbly::Object::Friction = 0.0f;
            }

            ~FrictionlessObject ()
            {
                wobbly::Object::Friction = mOldFriction;
            }

        public:

            SingleObjectStorage     storage;
            SingleObjectStorageView view;
            wobbly::Object          object;

        private:

            float mOldFriction;
    };

    TEST_F (FrictionlessObject, StepFunctionReturnsTrueIfVelocityRemaining)
    {
        bg::assign (view.velocity, wobbly::Vector (5, 0));

        /* Should always return true as a frictionless object can never
         * have its velocity slowed (except by applying external force,
         * which we don't do here) */
        EXPECT_TRUE (object.Step (1));
    }

    TEST_F (FrictionlessObject, StepFunctionReturnsFalseIfNoVelocity)
    {
        bg::assign (view.velocity, wobbly::Vector (0, 0));

        EXPECT_FALSE (object.Step (1));
    }

    TEST_F (FrictionlessObject, VelocityIncreasesLinearlyWithConstantForce)
    {
        std::function <double (int)> frictionlessHorizontalVelocityFunction =
            [this](int timestep) -> double {
                /* Reset velocity and force */
                bg::assign (view.velocity, wobbly::Vector (0, 0));
                bg::assign (view.force, wobbly::Vector (1.0f, 0));

                object.Step (timestep);

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionlessHorizontalVelocityFunction,
                     SatisfiesModel (Linear <double> ()));
    }

    TEST_F (FrictionlessObject, ObjectPositionChangesParabolically)
    {
        std::function <double (int)> frictionlessHorizontalPositionFunction =
            [this](int timestep) -> double {
                /* Reset velocity and force */
                bg::assign (view.position, wobbly::Point (0, 0));
                bg::assign (view.velocity, wobbly::Vector (0, 0));
                bg::assign (view.force, wobbly::Vector (1.0f, 0));

                object.Step (timestep);

                return bg::get <0> (view.position);
            };

        EXPECT_THAT (frictionlessHorizontalPositionFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    /* XXX: Maybe move this out of the FrictionlessObject fixture,
     * or rename it */
    TEST_F (FrictionlessObject, RelationshipBetweenFrictionAndVelocityIsLinear)
    {
        unsigned int nSamples = 10;
        int          range = std::pow (2, nSamples);

        std::function <double (int)> frictionToVelocityFunction =
            [this, range](int frictionAmount) -> double {
                /* Reset velocity and force */
                bg::assign (view.position, wobbly::Point (0, 0));
                bg::assign (view.velocity, wobbly::Vector (0, 0));
                bg::assign (view.force, wobbly::Vector (1.0f, 0));

                float frictionProportion =
                    frictionAmount / static_cast <float> (range);

                /* Step once, but with different frictions, linearly
                 * interpolate between frictionAmount and nSamples
                 * and scale by the default friction value to get
                 * our samples */
                object.Step (16,
                             frictionProportion * wobbly::Object::Friction,
                             wobbly::Object::Mass);

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionToVelocityFunction,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples)));
    }

    /* XXX: Test the relationship between mass and velocity - its hyperbolic,
     * but we don't yet have a matcher for that.
     *

    TEST (Object, ForceResetAfterStep)
    {
        SingleObjectStorage storage;
        SingleObjectStorageView view (storage);

        bg::assign (view.force, wobbly::Vector (1, 1));

        wobbly::Object object (storage.Position (),
                               storage.Velocity (),
                               storage.Force (),
                               storage.Ratio ());

        object.Step (1);

        EXPECT_THAT (view.force,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    } */

    constexpr double FirstPositionX = 50.0f;
    constexpr double FirstPositionY = 50.0f;

    constexpr double SecondPositionX = 100.0f;
    constexpr double SecondPositionY = 100.0f;

    constexpr double SpringConstant = 0.5f;

    TEST (Spring, MoveConstructorNoExcept)
    {
        SingleObjectStorage storageA, storageB;

        EXPECT_NO_THROW ({
            wobbly::Spring a (storageA.Force (),
                              storageB.Force (),
                              storageA.Position (),
                              storageB.Position (),
                              wobbly::Vector (0, 0));
            wobbly::Spring b (std::move (a));
        });
    }

    class Springs :
        public ::testing::Test
    {
        public:

            Springs ():
                desiredDistance (SecondPositionX - FirstPositionX,
                                 SecondPositionY - FirstPositionY),
                firstObject (firstStorage.Position (),
                             firstStorage.Velocity (),
                             firstStorage.Force (),
                             wobbly::Point (0, 0)),
                secondObject (secondStorage.Position (),
                              secondStorage.Velocity (),
                              secondStorage.Force (),
                              wobbly::Point (0, 1)),
                first (firstStorage),
                second (secondStorage),
                spring (firstStorage.Force (),
                        secondStorage.Force (),
                        firstStorage.Position (),
                        secondStorage.Position (),
                        desiredDistance)
            {
                bg::assign_point (first.position,
                                  wobbly::Point (FirstPositionX,
                                                 FirstPositionY));
                bg::assign_point (second.position,
                                  wobbly::Point (SecondPositionX,
                                                 SecondPositionY));
            }

        protected:

            wobbly::Vector desiredDistance;

        private:

            SingleObjectStorage firstStorage;
            SingleObjectStorage secondStorage;

        protected:

            wobbly::Object firstObject;
            wobbly::Object secondObject;
            SingleObjectStorageView first;
            SingleObjectStorageView second;
            wobbly::Spring spring;
    };

    TEST_F (Springs, NoForceAppliedWhenNoDeltaFromDesired)
    {
        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
        EXPECT_THAT (second.force,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    }
/*
    TEST_F (Springs, ForcesNotAppliedToAnchorObjects)
    {
        bg::assign (first.position,
                    wobbly::Vector (FirstPositionX - 10.0f,
                                    FirstPositionY - 10.0f));
        wobbly::Object::AnchorGrab grab (firstObject.Grab ());

        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    }
*/
    template <typename Position>
    wobbly::Vector
    ForceForSpring (Position      const &first,
                    Position      const &second,
                    wobbly::Vector const &desired)
    {
        wobbly::Vector expectedForce;
        bg::assign (expectedForce, second);
        bg::subtract_point (expectedForce, first);
        bg::subtract_point (expectedForce, desired);
        bg::multiply_value (expectedForce, SpringConstant);
        bg::divide_value (expectedForce, 2);

        return expectedForce;
    }

    TEST_F (Springs, ForceAppliedToFirstObjectProportianalToPositiveDistanceSK)
    {
        bg::assign (first.position,
                    wobbly::Vector (FirstPositionX - 10.0f,
                                    FirstPositionY - 10.0f));
        wobbly::Vector expectedForce (ForceForSpring (first.position,
                                                      second.position,
                                                      desiredDistance));

        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force, GeometricallyEqual (expectedForce));
    }

    TEST_F (Springs, ForceAppliedToSecondObjectProportionalToNegativeDistanceSK)
    {
        bg::assign (first.position, wobbly::Vector (FirstPositionX - 10.0f,
                                                    FirstPositionY - 10.0f));
        wobbly::Vector negativeDistance (desiredDistance);
        bg::multiply_value (negativeDistance, -1.0f);

        wobbly::Vector expectedForce (ForceForSpring (second.position,
                                                      first.position,
                                                      negativeDistance));

        spring.applyForces (SpringConstant);

        EXPECT_THAT (second.force, GeometricallyEqual (expectedForce));
    }

    TEST_F (Springs, ForceAccumulatesWithApplications)
    {
        bg::assign (first.position,
                    wobbly::Vector (FirstPositionX - 10.0f,
                                    FirstPositionY - 10.0f));
        wobbly::Vector expectedForce (ForceForSpring (first.position,
                                                      second.position,
                                                      desiredDistance));

        /* Scalar for single spring */
        unsigned int const nApplications = 3;
        bg::multiply_value (expectedForce, nApplications);

        for (unsigned int i = 0; i < nApplications; ++i)
            spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force, GeometricallyEqual (expectedForce));
    }

    TEST_F (Springs, ForceRelationshipIsLinearlyRelatedToDistance)
    {
        std::function <double (int)> forceByDistanceFunction =
            [this](int delta) -> double {
                bg::assign (first.force, wobbly::Vector (0, 0));
                bg::assign (second.force, wobbly::Vector (0, 0));
                bg::assign (first.position,
                            wobbly::Vector (FirstPositionX - delta,
                                            FirstPositionY));
                bg::assign (second.position,
                            wobbly::Vector (SecondPositionX + delta,
                                            SecondPositionY));

                spring.applyForces (SpringConstant);

                return bg::get <0> (first.force);
            };

        EXPECT_THAT (forceByDistanceFunction,
                     SatisfiesModel (Linear <double> ()));
    }

    TEST_F (Springs, ForceClippedWithVerySmallDelta)
    {
        double const justBelowThreshold = FirstPositionX -
                                          wobbly::Spring::ClipThreshold * 1.1;

        bg::assign (first.position,
                    wobbly::Vector (justBelowThreshold, FirstPositionY));
        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force, GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    TEST_F (Springs, ApplyForcesReturnsTrueIfForceRemaining)
    {
        /* Change the position of one object, that will cause forces
         * to be exerted
         */
        bg::assign (first.position, wobbly::Vector (FirstPositionX - 10,
                                                    FirstPositionY));

        EXPECT_TRUE (spring.applyForces (SpringConstant));
    }

    TEST_F (Springs, ApplyForcesReturnsFalseIfNoForceRemaining)
    {
        /* Where there is no delta, there is no force */
        EXPECT_FALSE (spring.applyForces (0.0f));
    }

    float const SpringScaleFactor = 2.0f;

    TEST_F (Springs, ForceExistsAfterLengthScaled)
    {
        spring.scaleLength (wobbly::Vector (SpringScaleFactor,
                                            SpringScaleFactor));
        EXPECT_TRUE (spring.applyForces (SpringConstant));
    }

    TEST_F (Springs, NoForceAfterLengthScaledAndObjectsMoved)
    {
        /* Calculate distance between first and second, then adjust
         * second's position to be distance * scaleFactor */
        wobbly::Vector distance;
        bg::assign (distance, second.position);
        bg::subtract_point (distance, first.position);
        bg::subtract_point (second.position, distance);
        bg::multiply_value (distance, SpringScaleFactor);
        bg::add_point (second.position, distance);

        spring.scaleLength (wobbly::Vector (SpringScaleFactor,
                                            SpringScaleFactor));
        EXPECT_FALSE (spring.applyForces (SpringConstant));
    }

    constexpr float TextureWidth = 50.0f;
    constexpr float TextureHeight = 100.0f;
    wobbly::Point const TextureCenter = wobbly::Point (TextureWidth / 2,
                                                       TextureHeight / 2);

    class SpringBezierModel :
        public ::testing::Test
    {
        public:

            SpringBezierModel () :
                model (wobbly::Vector (0, 0),
                       TextureWidth,
                       TextureHeight)
            {
            }

        protected:

            wobbly::Model model;
    };

    class PointCeilingOperation
    {
        public:

            template <typename P, int I>
            void apply (P &point) const
            {
                bg::set <I> (point, std::ceil (bg::get <I> (point)));
            }
    };

    template <typename Point>
    void PointCeiling (Point &p)
    {
        bg::for_each_coordinate (p, PointCeilingOperation ());
    }

    void MoveModelASmallAmount (wobbly::Model &model)
    {
        model.MoveModelTo (wobbly::Vector (1, 1));
        model.StepModel (1);
    }

    wobbly::Point GetTruncatedDeformedCenter (wobbly::Model const &model)
    {
        auto center (wobbly::Point (0.5, 0.5));
        auto point (model.DeformTexcoords (center));

        /* Not quite accurate, but truncate the returned point
         * so that we can do a reliable comparison */
        PointCeiling (point);
        return point;
    }

    TEST_F (SpringBezierModel, NoDeformationOnMovementWithNoAnchor)
    {
        MoveModelASmallAmount (model);
        auto point (GetTruncatedDeformedCenter (model));

        auto TextureCenterOffsetByOne (TextureCenter);
        bg::add_point (TextureCenterOffsetByOne, wobbly::Vector (1, 1));

        EXPECT_THAT (point,
                     GeometricallyEqual (TextureCenterOffsetByOne));
    }

    TEST_F (SpringBezierModel, NoDeformationOnMovementWithAnchorUngrabbed)
    {
        /* Anchor implicitly released at end of scope */
        {
            model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0));
        }

        MoveModelASmallAmount (model);
        auto point (GetTruncatedDeformedCenter (model));
        auto TextureCenterOffsetByOne (TextureCenter);
        bg::add_point (TextureCenterOffsetByOne, wobbly::Vector (1, 1));


        EXPECT_THAT (point,
                     GeometricallyEqual (TextureCenterOffsetByOne));
    }

    TEST_F (SpringBezierModel, MovingEntireModelCausesNoDeformationWithAnchor)
    {
        auto anchor (model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0)));

        MoveModelASmallAmount (model);
        auto point (GetTruncatedDeformedCenter (model));
        auto TextureCenterOffsetByOne (TextureCenter);
        bg::add_point (TextureCenterOffsetByOne, wobbly::Vector (1, 1));


        EXPECT_THAT (point,
                     GeometricallyEqual (TextureCenterOffsetByOne));
    }

    TEST_F (SpringBezierModel, MovingEntireModelChangesExtremesPositionsExactly)
    {
        unsigned int const x1 = 1;
        unsigned int const y1 = 1;
        unsigned int const x2 = TextureWidth + x1;
        unsigned int const y2 = TextureHeight + y1;

        model.MoveModelTo (wobbly::Point (x1, y1));

        std::array <wobbly::Point, 4> const extremes = model.Extremes ();
        Matcher <wobbly::Point const &> const textureEdges[] =
        {
            GeometricallyEqual (wobbly::Point (x1, y1)),
            GeometricallyEqual (wobbly::Point (x2, y1)),
            GeometricallyEqual (wobbly::Point (x1, y2)),
            GeometricallyEqual (wobbly::Point (x2, y2))
        };

        EXPECT_THAT (extremes,
                     ::testing::ElementsAreArray (textureEdges));
    }

    TEST_F (SpringBezierModel, MovingAnchorCausesDeformation)
    {
        auto anchor (model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0)));

        anchor.MoveBy (wobbly::Vector (1, 1));
        auto point (GetTruncatedDeformedCenter (model));

        EXPECT_THAT (point,
                     Not (GeometricallyEqual (TextureCenter)));
    }

    TEST_F (SpringBezierModel, TransformsCorrectPositionCoord)
    {
        /* This is somewhat difficult to observe. The best thing we can do
         * is grab the closest object, move it to a known position, and then
         * transform the closest object at that position and check if the
         * position co-ordinate matches exactly with the passed in position */
        auto grab (model.GrabAnchor (wobbly::Point (0, 0)));
        wobbly::Point objectPosition (-10, -10);

        /* The point starts at the origin, so this makes any absolute movement
         * an effective delta movement */
        grab.MoveBy (objectPosition);

        auto transformationVerification =
            [&objectPosition](wobbly::PointView <double> &position,
                              wobbly::PointView <double> &velocity,
                              wobbly::PointView <double> &force) -> void
            {
                EXPECT_THAT (position, GeometricallyEqual (objectPosition));
            };

        model.TransformClosestObjectToPosition (transformationVerification,
                                                objectPosition);
    }

    float const ModelScaleFactorX = 2.0f;
    float const ModelScaleFactorY = 3.0f;

    float const TextureWidthAfterResize = ModelScaleFactorX * TextureWidth;
    float const TextureHeightAfterResize = ModelScaleFactorY * TextureHeight;

    TEST_F (SpringBezierModel, ForcesConstantAfterResize)
    {
        wobbly::Vector addedForce (1.0f, 1.0f);
        wobbly::Point  objectPosition (0.0f, 0.0f);
        auto transformation =
            [&addedForce](wobbly::PointView <double> &position,
                          wobbly::PointView <double> &velocity,
                          wobbly::PointView <double> &force) -> void
            {
                bg::add_point (force, addedForce);
            };

        model.TransformClosestObjectToPosition (transformation,
                                                objectPosition);

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        auto verification =
            [&addedForce](wobbly::PointView <double> &position,
                          wobbly::PointView <double> &velocity,
                          wobbly::PointView <double> &force) -> void
            {
                EXPECT_THAT (force,
                             GeometricallyEqual (addedForce));
            };

        model.TransformClosestObjectToPosition (verification,
                                                objectPosition);
    }

    TEST_F (SpringBezierModel, VelocitiesConstantAfterResize)
    {
        wobbly::Vector addedVel (1.0f, 1.0f);
        wobbly::Point  objectPosition (0.0f, 0.0f);
        auto transformation =
            [&addedVel](wobbly::PointView <double> &position,
                        wobbly::PointView <double> &velocity,
                        wobbly::PointView <double> &force) -> void
            {
                bg::add_point (velocity, addedVel);
            };

        model.TransformClosestObjectToPosition (transformation,
                                                objectPosition);

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        auto verification =
            [&addedVel](wobbly::PointView <double> &position,
                        wobbly::PointView <double> &velocity,
                        wobbly::PointView <double> &force) -> void
            {
                EXPECT_THAT (velocity,
                             GeometricallyEqual (addedVel));
            };
        model.TransformClosestObjectToPosition (verification,
                                                objectPosition);
    }

    typedef Matcher <wobbly::Point const &> PointMatcher;

    TEST_F (SpringBezierModel, PositionsScaledAfterResize)
    {
        wobbly::Vector const scaleFactor (ModelScaleFactorX,
                                          ModelScaleFactorY);

        std::array <wobbly::Point, 4> const extremes = model.Extremes ();

        /* Older versions of gmock don't support matching against a vector */
        auto scaledPointMatcher =
            [&scaleFactor](wobbly::Point p) -> PointMatcher {
                bg::multiply_point (p, scaleFactor);
                return GeometricallyEqual (p);
            };

        PointMatcher const scaledExtremes[4] =
        {
            scaledPointMatcher (extremes[0]),
            scaledPointMatcher (extremes[1]),
            scaledPointMatcher (extremes[2]),
            scaledPointMatcher (extremes[3])
        };

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_THAT (model.Extremes (),
                     ElementsAreArray (scaledExtremes));
    }

    TEST_F (SpringBezierModel, PositionsScaledRelativeToModelOrigin)
    {
        wobbly::Vector const scaleFactor (ModelScaleFactorX,
                                          ModelScaleFactorY);
        wobbly::Vector const movement (10.0f, 10.0f);

        model.MoveModelTo (movement);

        std::array <wobbly::Point, 4> const extremes = model.Extremes ();
        auto scaledPointMatcher =
            [&scaleFactor, &movement](wobbly::Point p) -> PointMatcher {
                bg::subtract_point (p, movement);
                bg::multiply_point (p, scaleFactor);
                bg::add_point (p, movement);
                return GeometricallyEqual (p);
            };

        PointMatcher const scaledExtremes[4] =
        {
            scaledPointMatcher (extremes[0]),
            scaledPointMatcher (extremes[1]),
            scaledPointMatcher (extremes[2]),
            scaledPointMatcher (extremes[3])
        };

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_THAT (model.Extremes (),
                     ElementsAreArray (scaledExtremes));
    }

    /* We can verify this by grabbing an anchor at a known position
     * and then resizing the model. If the model has net force, then
     * the anchor did not move */
    TEST_F (SpringBezierModel, AnchorPositionsNotMovedAfterResize)
    {
        wobbly::Vector const grabPoint (TextureWidth,
                                        TextureHeight);
        wobbly::Object::AnchorGrab grab (model.GrabAnchor (grabPoint));

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_TRUE (model.StepModel (1));
    }

    TEST_F (SpringBezierModel, NetForceIsZeroAfterResizingSettledModel)
    {
        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_FALSE (model.StepModel (1));
    }

    TEST_F (SpringBezierModel, PositionIsTopLeftCornerAtSettled)
    {
        wobbly::Vector const position (100, 100);
        model.MoveModelTo (position);

        /* We can assume that Extremes ()[0] is the top-left position as
         * the other tests enforce it being the minimum,minimum position */

        EXPECT_THAT (model.Extremes ()[0], GeometricallyEqual (position));
    }

    TEST_F (SpringBezierModel, ModelMovementTakesIntoAccountTargetPos)
    {
        /* Create an anchor on 0, 0 and then move it to 100, 100, then move
         * it back to 0, 0. The end result should be that the model position
         * will end up back at 0, 0. We can't observe the target positions
         * so we need to do it this way */

        wobbly::Vector const grabPoint (0, 0);
        wobbly::Object::AnchorGrab grab (model.GrabAnchor (grabPoint));

        grab.MoveBy (wobbly::Point (100, 100));
        model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (model.StepModel (1));

        EXPECT_THAT (model.Extremes ()[0],
                     GeometricallyEqual (wobbly::Point (0, 0)));
    }

    void GrabModelMoveAndStepASmallAmount (wobbly::Model &model)
    {
        wobbly::Vector const grabPoint (model.Extremes ()[3]);
        wobbly::Object::AnchorGrab anchor (model.GrabAnchor (grabPoint));

        anchor.MoveBy (wobbly::Point (100, 100));

        /* Five steps is reasonable */
        for (int i = 0; i < 5; ++i)
            model.StepModel (16);
    }

    TEST (SpringBezierModelSettings, ModelWithHigherSpringTakesFasterFirstStep)
    {
        /* We want to create two models each with different spring constants
         * and check that the second one moves faster than the first after
         * taking the first step.
         *
         * We'll test this by reading the top-left hand point and grabbing
         * on the bottom left. The model that moves quicker should have its
         * top-left hand point also move a lot quicker
         */

        wobbly::Model::Settings lowerSpringKSettings =
        {
            wobbly::Model::DefaultSpringConstant - 2.0f,
            wobbly::Object::Friction,
            wobbly::Model::DefaultObjectRange
        };

        wobbly::Model::Settings higherSpringKSettings =
        {
            wobbly::Model::DefaultSpringConstant,
            wobbly::Object::Friction,
            wobbly::Model::DefaultObjectRange
        };

        wobbly::Model lowerSpringKModel (wobbly::Vector (0, 0),
                                         TextureWidth,
                                         TextureHeight,
                                         lowerSpringKSettings);
        wobbly::Model higherSpringKModel (wobbly::Vector (0, 0),
                                          TextureWidth,
                                          TextureHeight,
                                          higherSpringKSettings);

        GrabModelMoveAndStepASmallAmount (lowerSpringKModel);
        GrabModelMoveAndStepASmallAmount (higherSpringKModel);

        EXPECT_GT (bg::get <0> (higherSpringKModel.Extremes ()[0]),
                   bg::get <0> (lowerSpringKModel.Extremes ()[0]));
    }

    TEST (SpringBezierModelSettings, ModelWithLowerFrictionTakesFasterFirstStep)
    {
        wobbly::Model::Settings lowerFrictionSettings =
        {
            wobbly::Model::DefaultSpringConstant,
            wobbly::Object::Friction,
            wobbly::Model::DefaultObjectRange
        };

        wobbly::Model::Settings higherFrictionSettings =
        {
            wobbly::Model::DefaultSpringConstant,
            wobbly::Object::Friction + 2.0f,
            wobbly::Model::DefaultObjectRange
        };

        wobbly::Model lowerFrictionModel (wobbly::Vector (0, 0),
                                          TextureWidth,
                                          TextureHeight,
                                          lowerFrictionSettings);
        wobbly::Model higherFrictionModel (wobbly::Vector (0, 0),
                                           TextureWidth,
                                           TextureHeight,
                                           higherFrictionSettings);

        GrabModelMoveAndStepASmallAmount (lowerFrictionModel);
        GrabModelMoveAndStepASmallAmount (higherFrictionModel);

        EXPECT_GT (bg::get <0> (lowerFrictionModel.Extremes ()[0]),
                   bg::get <0> (higherFrictionModel.Extremes ()[0]));
    }

    TEST_F (SpringBezierModel, StepZeroReturnsTrueOnGrabbingAndMovingAnchor)
    {
        /* Create an anchor and move it. StepModel (0) should return true */
        wobbly::Vector const grabPoint (0, 0);
        wobbly::Object::AnchorGrab grab (model.GrabAnchor (grabPoint));

        grab.MoveBy (wobbly::Point (100, 100));
        EXPECT_TRUE (model.StepModel (0));
    }

    TEST_F (SpringBezierModel, StepZeroReturnsTrueOnNonEquallibriumModel)
    {
        {
            /* Create an anchor and move it. StepModel (0) should return true */
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Object::AnchorGrab grab (model.GrabAnchor (grabPoint));

            grab.MoveBy (wobbly::Point (100, 100));

            /* Step the model once, this will make the model unequal */
            model.StepModel (1);

            /* Grab goes away here but the model is still unequal */
        }

        EXPECT_TRUE (model.StepModel (0));
    }

    class FrictionlessSpringBezierModel :
        public SpringBezierModel
    {
        public:

            FrictionlessSpringBezierModel () :
                mLastFriction (wobbly::Object::Friction)
            {
            }

            ~FrictionlessSpringBezierModel ()
            {
                wobbly::Object::Friction = mLastFriction;
            }

        private:

            double mLastFriction;
    };

    TEST_F (FrictionlessSpringBezierModel, ContinueStepWhenObjectsHaveVelocity)
    {
        auto transformation = [](wobbly::PointView <double> &position,
                                 wobbly::PointView <double> &velocity,
                                 wobbly::PointView <double> &force) -> void
        {
            bg::add_point (velocity, wobbly::Point (10.0f, 0.0f));
        };

        model.TransformClosestObjectToPosition (transformation,
                                                wobbly::Point (0, 0));
        EXPECT_TRUE (model.StepModel (1));
    }

    TEST_F (FrictionlessSpringBezierModel, ContinueStepWhenObjectsHaveForce)
    {
        /* Move a object by -50, -50, this will cause all objects to have
         * force */
        auto transformation = [](wobbly::PointView <double> &position,
                                 wobbly::PointView <double> &velocity,
                                 wobbly::PointView <double> &force) -> void
        {
            bg::subtract_point (position, wobbly::Point (50.0f, 50.0f));
        };

        model.TransformClosestObjectToPosition (transformation,
                                                wobbly::Point (0, 0));
        EXPECT_TRUE (model.StepModel (1));
    }

    class MockImmediatelyMovablePosition :
        public wobbly::ImmediatelyMovablePosition
    {
        public:

            MockImmediatelyMovablePosition (wobbly::Vector const &position) :
                position (position)
            {
                EXPECT_CALL (*this, MoveByDelta (_)).Times (AtLeast (0));
                EXPECT_CALL (*this, DeltaTo (_)).Times (AtLeast (0));
                EXPECT_CALL (*this, Index ()).Times (AtLeast (0));
                EXPECT_CALL (*this, Position ()).Times (AtLeast (0));

                auto deltaToDelegate =
                    &MockImmediatelyMovablePosition::DeltaToDelegate;

                ON_CALL (*this, DeltaTo (_))
                    .WillByDefault (Invoke (this, deltaToDelegate)); 
            }

            MOCK_METHOD1 (MoveByDelta, void (wobbly::Vector const &));
            MOCK_CONST_METHOD1 (DeltaTo,
                                wobbly::Vector (wobbly::Vector const &));
            MOCK_CONST_METHOD0 (Position, wobbly::PointView <double> const & ());
            MOCK_CONST_METHOD0 (Index, wobbly::Point const & ());

        private:

            wobbly::Vector
            DeltaToDelegate (wobbly::Vector const &v)
            {
                wobbly::Vector delta (v);
                bg::subtract_point (delta, position);
                return delta;
            }

            wobbly::Vector position;
    };

    template <typename S, typename Container>
    class AllElementsMatcher :
        public MatcherInterface <Container const &>
    {
        public:

            explicit AllElementsMatcher (Matcher <S> const &matcher) :
                mMatcher (matcher)
            {
            }

            bool MatchAndExplain (Container const &container,
                                  MatchResultListener *listener) const
            {
                size_t const containerSize = container.size ();
                bool someFailed = false;
                ::std::ostream *os = listener->stream ();

                for (size_t i = 0; i < containerSize; ++i)
                {
                    if (os)
                        *os << " - element " << i << ": ";
                    bool result = mMatcher.MatchAndExplain (container[i],
                                                            listener);
                    if (os)
                        *os << (result ? "passed" : "failed") << std::endl;

                    someFailed |= !result;
                }

                return !someFailed;
            }

            void DescribeTo (std::ostream *os) const
            {
                *os << "All elements ";
                mMatcher.DescribeTo (os);
                *os << std::endl;
            }

            void DescribeNegationTo (std::ostream *os) const
            {
                *os << "Not all elements ";
                mMatcher.DescribeTo (os);
                *os << std::endl;
            }

        private:

            Matcher <S> mMatcher;
    };

    template <typename Cont, typename S>
    inline Matcher<Cont const &> AllElementsAre (Matcher <S> const &sub)
    {
        return MakeMatcher (new AllElementsMatcher <S, Cont> (sub));
    }

    /* FIXME: Obviously needs a refactor */
    float TileWidth (float width, unsigned int nHorizontalTiles)
    {
        return width / (nHorizontalTiles - 1);
    }

    float TileHeight (float height, unsigned int nVerticalTiles)
    {
        return height / (nVerticalTiles - 1);
    }

    class BezierMesh :
        public Test
    {
        public:

            wobbly::BezierMesh mesh;

        protected:

            typedef wobbly::Point Point;

            typedef std::function <void (wobbly::PointView <double> &,
                                         size_t,
                                         size_t)> MeshTransform;
            typedef std::function <double (Point const &)> ResultFactory;

            virtual void SetUp ();
            void InitializeWithDimensions (float width, float height);
            void ApplyTransformation (MeshTransform const &);

            std::function <double (int)>
            UnitDeformationFunction (ResultFactory const      &resultFactory,
                                     wobbly::BezierMesh const &mesh,
                                     unsigned int             nSamples);
    };

    void
    BezierMesh::SetUp ()
    {
        InitializeWithDimensions (TextureWidth, TextureHeight);
    }

    void
    BezierMesh::InitializeWithDimensions (float width, float height)
    {
        float tileWidth = TileWidth (width, wobbly::BezierMesh::Width);
        float tileHeight = TileHeight (height, wobbly::BezierMesh::Height);

        for (unsigned int i = 0; i < wobbly::BezierMesh::Height; ++i)
        {
            for (unsigned int j = 0; j < wobbly::BezierMesh::Width; ++j)
            {
                auto pv (mesh.PointForIndex (j, i));
                bg::assign_point (pv, wobbly::Point (tileWidth * j,
                                                     tileHeight * i));
            }
        }
    }

    void
    BezierMesh::ApplyTransformation (MeshTransform const &transform)
    {
        for (size_t i = 0; i < wobbly::BezierMesh::Height; ++i)
        {
            for (size_t j = 0; j < wobbly::BezierMesh::Width; ++j)
            {
                auto pv (mesh.PointForIndex (j, i));
                transform (pv, j, i);
            }
        }
    }

    std::function <double (int)>
    BezierMesh::UnitDeformationFunction (ResultFactory const      &result,
                                         wobbly::BezierMesh const &mesh,
                                         unsigned int             nSamples)
    {
        auto deformation =
            [nSamples, &mesh, &result](int lookupValue) -> double {
                /* Divide by the total number of samples (2^10) in order to get
                 * the unit value. */

                float const unitLookupValue =
                    lookupValue / static_cast <float> (pow (2, nSamples));
                auto const lookup (wobbly::Point (unitLookupValue,
                                                  unitLookupValue));
                auto p (mesh.DeformUnitCoordsToMeshSpace (lookup));

                return result (p);
            };

        return deformation;
    }

    TEST_F (BezierMesh, LinearHorizontalDeformationForUniformScale)
    {
        using namespace std::placeholders;

        ApplyTransformation (std::bind ([](wobbly::PointView <double> &pv) {
                                            bg::multiply_value (pv, 2);
                                        }, _1));

        unsigned int const nSamples = 10;
        std::function <double (int)> horizontalDeformation =
            UnitDeformationFunction ([](wobbly::Point const &p) -> double {
                                        return bg::get <0> (p);
                                     },
            mesh,
            nSamples);

        EXPECT_THAT (horizontalDeformation,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples),
                                     WithToleranace (10e-5)));
    }

    TEST_F (BezierMesh, LinearVerticalDeformationForUniformScale)
    {
        using namespace std::placeholders;

        ApplyTransformation (std::bind ([](wobbly::PointView <double> &pv) {
                                           bg::multiply_value (pv, 2);
                                        }, _1));

        unsigned int const nSamples = 10;
        std::function <double (int)> verticalDeformation =
            UnitDeformationFunction ([](wobbly::Point const &p) -> double {
                                        return bg::get <1> (p);
                                     },
            mesh,
            nSamples);

        EXPECT_THAT (verticalDeformation,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples),
                                     WithToleranace (10e-5)));
    }

    /* Its somewhat difficult to test the non-linear case, considering
     * that the bezier patch evaluation is actually an interpolation
     * between four different parabolic functions. For now just check
     * if the deformation was non-linear */
    TEST_F (BezierMesh, NonLinearHorizontalDeformationForNonUniformScale)
    {
        using namespace std::placeholders;

        typedef wobbly::PointView <double> DoublePointView;

        ApplyTransformation ([](DoublePointView &pv, size_t x, size_t y) {
                                  bg::multiply_point (pv, wobbly::Point (x, y));
                             });

        unsigned int const nSamples = 10;
        std::function <double (int)> horizontalDeformation =
            UnitDeformationFunction ([](wobbly::Point const &p) -> double {
                                         return bg::get <0> (p);
                                     },
                                     mesh,
                                     nSamples);

        EXPECT_THAT (horizontalDeformation,
                     Not (SatisfiesModel (Linear <double> (),
                                          WithSamples (nSamples),
                                          WithToleranace (10e-5))));
    }

    TEST_F (BezierMesh, ExtremesAreTextureEdges)
    {
        std::array <wobbly::Point, 4> const extremes = mesh.Extremes ();
        Matcher <wobbly::Point const &> const textureEdges[] =
        {
            GeometricallyEqual (wobbly::Point (0, 0)),
            GeometricallyEqual (wobbly::Point (TextureWidth, 0)),
            GeometricallyEqual (wobbly::Point (0, TextureHeight)),
            GeometricallyEqual (wobbly::Point (TextureWidth, TextureHeight))
        };

        EXPECT_THAT (extremes,
                     ::testing::ElementsAreArray (textureEdges));
    }

    class BezierMeshPoints :
        public BezierMesh,
        public WithParamInterface <wobbly::Point>
    {
    };

    wobbly::Point TexturePrediction (wobbly::Point const &unit)
    {
        wobbly::Point textureRelative (bg::get <1> (unit),
                                       bg::get <0> (unit));
        bg::multiply_point (textureRelative,
                            wobbly::Point (TextureWidth, TextureHeight));
        PointCeiling (textureRelative);
        return textureRelative;
    }

    wobbly::Point MeshInterpolation (wobbly::BezierMesh const &mesh,
                                     wobbly::Point const &unit)
    {
        wobbly::Point meshRelative (mesh.DeformUnitCoordsToMeshSpace (unit));
        PointCeiling (meshRelative);
        return meshRelative;
    }

    TEST_P (BezierMeshPoints, LinearDefinitionForNoTransformation)
    {
        auto point (GetParam ());
        auto prediction (TexturePrediction (point));
        auto interpolated (MeshInterpolation (mesh, point));

        EXPECT_THAT (interpolated, GeometricallyEqual (prediction));
    }

    wobbly::Point const meshExtremes[] =
    {
        wobbly::Point (0.0, 0.0),
        wobbly::Point (0.0, 1.0),
        wobbly::Point (1.0, 0.0),
        wobbly::Point (1.0, 1.0)
    };

    INSTANTIATE_TEST_CASE_P (MeshExtremes,
                             BezierMeshPoints,
                             ValuesIn (meshExtremes));
}
