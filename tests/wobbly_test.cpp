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
#include <boost/geometry/algorithms/for_each.hpp>
#include <boost/geometry/algorithms/within.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <mathematical_model_matcher.h>

#include <smspillaz/wobbly/wobbly.h>
#include <wobbly_internal.h>

using ::testing::_;
using ::testing::AtLeast;
using ::testing::ElementsAreArray;
using ::testing::Invoke;
using ::testing::MakeMatcher;
using ::testing::MakePolymorphicMatcher;
using ::testing::Matcher;
using ::testing::MatcherCast;
using ::testing::MatcherInterface;
using ::testing::MatchResultListener;
using ::testing::Not;
using ::testing::PolymorphicMatcher;
using ::testing::Test;
using ::testing::Types;
using ::testing::ValuesIn;
using ::testing::WithParamInterface;

using ::wobbly::matchers::Eq;
using ::wobbly::matchers::SatisfiesModel;
using ::wobbly::matchers::WithSamples;
using ::wobbly::matchers::WithTolerance;

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

            SingleObjectStorageView first;
            SingleObjectStorageView second;
            wobbly::Spring spring;
    };

    TEST_F (Springs, NoForceAppliedWhenNoDeltaFromDesired)
    {
        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force,
                     Eq (wobbly::Vector (0, 0)));
        EXPECT_THAT (second.force,
                     Eq (wobbly::Vector (0, 0)));
    }

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

        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (expectedForce));
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

        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (second.force, Eq (expectedForce));
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
            spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (expectedForce));
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

                spring.ApplyForces (SpringConstant);

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
        spring.ApplyForces (SpringConstant);

        EXPECT_THAT (first.force, Eq (wobbly::Vector (0, 0)));
    }

    TEST_F (Springs, ApplyForcesReturnsTrueIfForceRemaining)
    {
        /* Change the position of one object, that will cause forces
         * to be exerted
         */
        bg::assign (first.position, wobbly::Vector (FirstPositionX - 10,
                                                    FirstPositionY));

        EXPECT_TRUE (spring.ApplyForces (SpringConstant));
    }

    TEST_F (Springs, ApplyForcesReturnsFalseIfNoForceRemaining)
    {
        /* Where there is no delta, there is no force */
        EXPECT_FALSE (spring.ApplyForces (0.0f));
    }

    double const SpringScaleFactor = 2.0f;

    TEST_F (Springs, ForceExistsAfterLengthScaled)
    {
        spring.ScaleLength (wobbly::Vector (SpringScaleFactor,
                                            SpringScaleFactor));
        EXPECT_TRUE (spring.ApplyForces (SpringConstant));
    }

    TEST_F (Springs, NoForceAfterLengthScaledAndObjectsMoved)
    {
        /* Calculate distance between first and second, then adjust
         * second object's position to be distance * scaleFactor */
        wobbly::Vector distance;
        bg::assign (distance, second.position);
        bg::subtract_point (distance, first.position);
        bg::subtract_point (second.position, distance);
        bg::multiply_value (distance, SpringScaleFactor);
        bg::add_point (second.position, distance);

        spring.ScaleLength (wobbly::Vector (SpringScaleFactor,
                                            SpringScaleFactor));
        EXPECT_FALSE (spring.ApplyForces (SpringConstant));
    }

    class MockAnchorStorage :
        public wobbly::Anchor::Storage
    {
        public:

            MockAnchorStorage ()
            {
                EXPECT_CALL (*this, Lock (_)).Times (AtLeast (0));
                EXPECT_CALL (*this, Unlock (_)).Times (AtLeast (0));
            }

            MOCK_METHOD1 (Lock, void (size_t));
            MOCK_METHOD1 (Unlock, void (size_t));
    };

    class StubLifetime :
        public wobbly::Anchor::Lifetime
    {
        private:

            bool SameAs (Lifetime const &lifetime) const override
            {
                return static_cast <Lifetime const *> (this) == &lifetime;
            }
    };

    class Anchor :
        public Test
    {
        public:

            wobbly::Anchor::LTH Handle (StubLifetime &lifetime)
            {
                return wobbly::Anchor::LTH (&lifetime,
                                            [](wobbly::Anchor::Lifetime *) {
                                            });
            }

            StubLifetime            lifetime;
            MockAnchorStorage       anchorStorage;
            SingleObjectStorage     object;
    };

    TEST_F (Anchor, ConstructionLocksWithIndex)
    {
        size_t const index = 0;
        EXPECT_CALL (anchorStorage, Lock (index)).Times (1);

        wobbly::Anchor anchor (object.Position (),
                               Handle (lifetime),
                               anchorStorage,
                               index);
    }

    TEST_F (Anchor, DestructionUnlocksWithIndex)
    {
        size_t const index = 0;
        wobbly::Anchor anchor (object.Position (),
                               Handle (lifetime),
                               anchorStorage,
                               index);

        EXPECT_CALL (anchorStorage, Unlock (index)).Times (1);
    }

    TEST_F (Anchor, MoveConstructorNoThrow)
    {
        size_t const index = 0;
        wobbly::Anchor anchor (object.Position (),
                               Handle (lifetime),
                               anchorStorage,
                               index);

        EXPECT_NO_THROW ({
            wobbly::Anchor anchorNext (std::move (anchor));
        });
    }

    TEST_F (Anchor, UnlockNotCalledMultipleTimesAfterMove)
    {
        size_t const index = 0;
        wobbly::Anchor anchor (object.Position (),
                               Handle (lifetime),
                               anchorStorage,
                               index);
        wobbly::Anchor anchorNext (std::move (anchor));

        EXPECT_CALL (anchorStorage, Unlock (index)).Times (1);
    }

    TEST_F (Anchor, MoveByCausesPositionToMove)
    {
        size_t const index = 0;
        wobbly::Anchor anchor (object.Position (),
                               Handle (lifetime),
                               anchorStorage,
                               index);

        wobbly::Point const movement (100, 100);
        anchor.MoveBy (movement);

        EXPECT_THAT (object.Position (), Eq (movement));
    }

    class TrackedAnchors :
        public Test
    {
        public:

            wobbly::TrackedAnchors <3> anchors;
    };

    class MockAnchorAction
    {
        public:

            MockAnchorAction ()
            {
                EXPECT_CALL (*this, Action (_)).Times (AtLeast (0));
            }

            MOCK_METHOD1 (Action, void (size_t));
    };

    TEST_F (TrackedAnchors, WithFirstGrabbedDisabledWithNoAnchorsLocked)
    {
        using namespace std::placeholders;

        MockAnchorAction action;
        EXPECT_CALL (action, Action (_)).Times (0);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, LockingAnchorEnablesWithFirstGrabbed)
    {
        using namespace std::placeholders;

        size_t lockIndex = 1;

        anchors.Lock (lockIndex);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (lockIndex)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, FirstGrabbedTakesPriorityAsMoreLocked)
    {
        using namespace std::placeholders;

        size_t lockIndex = 1;

        anchors.Lock (lockIndex);
        anchors.Lock (lockIndex + 1);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (lockIndex)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, MoveToNextHeavisestWhereFirstGrabbedUnlocked)
    {
        using namespace std::placeholders;

        anchors.Lock (0);

        for (size_t i = 0; i < 2; ++i)
            anchors.Lock (1);

        for (size_t i = 0; i < 3; ++i)
            anchors.Lock (2);

        anchors.Unlock (0);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (2)).Times (1);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    TEST_F (TrackedAnchors, UnsetWhenAllUnlocked)
    {
        using namespace std::placeholders;

        for (size_t i = 0; i < 3; ++i)
            anchors.Lock (i);

        for (size_t i = 0; i < 3; ++i)
            anchors.Unlock (i);

        MockAnchorAction action;
        EXPECT_CALL (action, Action (_)).Times (0);

        anchors.WithFirstGrabbed (std::bind (&MockAnchorAction::Action,
                                             &action, _1));
    }

    constexpr double TextureWidth = 50.0f;
    constexpr double TextureHeight = 100.0f;
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
        model.Step (1);
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
                     Eq (TextureCenterOffsetByOne));
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
                     Eq (TextureCenterOffsetByOne));
    }

    TEST_F (SpringBezierModel, MovingEntireModelCausesNoDeformationWithAnchor)
    {
        auto anchor (model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0)));

        MoveModelASmallAmount (model);
        auto point (GetTruncatedDeformedCenter (model));
        auto TextureCenterOffsetByOne (TextureCenter);
        bg::add_point (TextureCenterOffsetByOne, wobbly::Vector (1, 1));


        EXPECT_THAT (point,
                     Eq (TextureCenterOffsetByOne));
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
            Eq (wobbly::Point (x1, y1)),
            Eq (wobbly::Point (x2, y1)),
            Eq (wobbly::Point (x1, y2)),
            Eq (wobbly::Point (x2, y2))
        };

        EXPECT_THAT (extremes, ElementsAreArray (textureEdges));
    }

    TEST_F (SpringBezierModel, MovingAnchorCausesDeformation)
    {
        auto anchor (model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0)));

        anchor.MoveBy (wobbly::Vector (1, 1));
        auto point (GetTruncatedDeformedCenter (model));

        EXPECT_THAT (point,
                     Not (Eq (TextureCenter)));
    }

    typedef std::tuple <wobbly::Point, wobbly::Point, size_t> SpringGrabParams;

    class SpringBezierModelGrabs :
        public SpringBezierModel,
        public WithParamInterface <SpringGrabParams>
    {
        public:

            SpringBezierModelGrabs () :
                grabPosition (std::get <0> (GetParam ())),
                movement (std::get <1> (GetParam ())),
                extremeIndex (std::get <2> (GetParam ()))
            {
            }

            wobbly::Point const &grabPosition;
            wobbly::Point const &movement;
            size_t              extremeIndex;
    };

    TEST_P (SpringBezierModelGrabs, GrabsCorrectIndex)
    {
        wobbly::Anchor grab (model.GrabAnchor (grabPosition));
        grab.MoveBy (movement);

        wobbly::Point transformed (grabPosition);
        bg::add_point (transformed, movement);

        EXPECT_THAT (model.Extremes ()[extremeIndex],
                     Eq (transformed));
    }

    TEST_P (SpringBezierModelGrabs, AlwaysSettlesAtCurrentlyAnchoredPosition)
    {
        /* Check that when we grab the model from each of the four corners
         * and move that anchor by 100, 100 that the model always settles
         * at exactly 100, 100
         *
         * While exact positioning isn't possible without anchors grabbed,
         * it is almost always desired in this case */
        wobbly::Anchor grab (model.GrabAnchor (grabPosition));
        grab.MoveBy (wobbly::Vector (100, 100));

        /* Wait for model to settle */
        while (model.Step (1));

        EXPECT_THAT (model.Extremes ()[0],
                     Eq (wobbly::Point (100, 100)));
    }

    SpringGrabParams const springGrabParams[] =
    {
        SpringGrabParams (wobbly::Point (0.0, 0.0),
                          wobbly::Point (-1.0, -1.0),
                          0),
        SpringGrabParams (wobbly::Point (TextureWidth, 0.0),
                          wobbly::Point (1.0, -1.0),
                          1),
        SpringGrabParams (wobbly::Point (0.0, TextureHeight),
                          wobbly::Point (-1.0, 1.0),
                          2),
        SpringGrabParams (wobbly::Point (TextureWidth, TextureHeight),
                          wobbly::Point (1.0, 1.0),
                          3)
    };

    INSTANTIATE_TEST_CASE_P (Extremes, SpringBezierModelGrabs,
                             ValuesIn (springGrabParams));
                             

    double const ModelScaleFactorX = 2.0f;
    double const ModelScaleFactorY = 3.0f;

    double const TextureWidthAfterResize = ModelScaleFactorX * TextureWidth;
    double const TextureHeightAfterResize = ModelScaleFactorY * TextureHeight;

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
                return Eq (p);
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

        EXPECT_THAT (model.Extremes (), ElementsAreArray (scaledExtremes));
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
                return Eq (p);
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

        EXPECT_THAT (model.Extremes (), ElementsAreArray (scaledExtremes));
    }

    /* We can verify this by grabbing an anchor at a known position
     * and then resizing the model. If the model has net force, then
     * the anchor did not move */
    TEST_F (SpringBezierModel, AnchorPositionsNotMovedAfterResize)
    {
        wobbly::Vector const grabPoint (TextureWidth,
                                        TextureHeight);
        wobbly::Anchor grab (model.GrabAnchor (grabPoint));

        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_TRUE (model.Step (1));
    }

    TEST_F (SpringBezierModel, NetForceIsZeroAfterResizingSettledModel)
    {
        model.ResizeModel (TextureWidthAfterResize,
                           TextureHeightAfterResize);

        EXPECT_FALSE (model.Step (1));
    }

    TEST_F (SpringBezierModel, PositionIsTopLeftCornerAtSettled)
    {
        wobbly::Vector const position (100, 100);
        model.MoveModelTo (position);

        /* We can assume that Extremes ()[0] is the top-left position as
         * the other tests enforce it being the minimum,minimum position */
        EXPECT_THAT (model.Extremes ()[0], Eq (position));
    }

    template <typename ParentGeometry>
    class WithinGeometryMatcher
    {
        public:

            WithinGeometryMatcher (ParentGeometry const &parent) :
                parent (parent)
            {
            }

            template <typename ChildGeometry>
            bool MatchAndExplain (ChildGeometry const &child,
                                  MatchResultListener *listener) const
            {
                return bg::within (child, parent);
            }

            void DescribeTo (std::ostream *os) const
            {
                *os << "is within :" << std::endl;
                bg::model::polygon <wobbly::Point> poly;
                bg::assign (poly, parent);
                bg::for_each_point (poly, PrintPoint (os));
            }

            void DescribeNegationTo (std::ostream *os) const
            {
                *os << "is not within :" << std::endl;
                bg::model::polygon <wobbly::Point> poly;
                bg::assign (poly, parent);
                bg::for_each_point (poly, PrintPoint (os));
            }

        private:

            class PrintPoint
            {
                public:

                    PrintPoint (std::ostream *os) :
                        os (os)
                    {
                    }

                    template <typename Point>
                    void operator () (Point const &p)
                    {
                        *os << " - " << p << std::endl;
                    }

                private:

                   std::ostream *os;
            };

            ParentGeometry parent;
    };

    template <typename ParentGeometry>
    inline PolymorphicMatcher <WithinGeometryMatcher <ParentGeometry> >
    WithinGeometry (ParentGeometry const &parent)
    {
        WithinGeometryMatcher <ParentGeometry> matcher (parent);
        return MakePolymorphicMatcher (matcher);
    }

    typedef bg::model::box <wobbly::Point> PointBox;

    /* The only way we can test this is to perform operations dependent
     * on a target position and ensure that they are precise to the grab's
     * position */
    TEST_F (SpringBezierModel, TargetPositionWithinRangeGrabbed)
    {
        /* Create an anchor on 0, 0 and then move it to 100, 100, then move
         * it back to 0, 0. The end result should be that the model position
         * will end up back at 0, 0. We can't observe the target positions
         * so we need to do it this way */

        wobbly::Vector const grabPoint (0, 0);
        wobbly::Anchor grab (model.GrabAnchor (grabPoint));

        grab.MoveBy (wobbly::Point (100, 100));
        model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (model.Step (1));

        EXPECT_THAT (model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    /* The only way we can test this is to perform operations dependent
     * on a target position and ensure that they are precise to the grab's
     * position */
    TEST_F (SpringBezierModel, TargetPositionWithinRangeAnchorChange)
    {
        /* Create an anchor on 0, 0, then at TextureWidth,0 and move it by
         * 100, 100, then move the model it back to 0, 0. This checks if
         * the TargetPosition machinery is able to handle different anchor
         * grabs */

        {
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (model.GrabAnchor (grabPoint));
        }

        wobbly::Vector const grabPoint (TextureWidth, 0);
        wobbly::Anchor grab (model.GrabAnchor (grabPoint));

        grab.MoveBy (wobbly::Point (100, 100));
        model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (model.Step (1));

        EXPECT_THAT (model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    TEST_F (SpringBezierModel, TargetPositionWithinRangeNotGrabbed)
    {
        /* This time integrate the model for a short period while grabbed
         * and then move it to a new position. This should still cause its
         * target position to end up roughly in the same place */
        {
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (model.GrabAnchor (grabPoint));

            grab.MoveBy (wobbly::Point (100, 100));
            model.Step (2);
        }

        model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (model.Step (1));

        EXPECT_THAT (model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    void GrabModelMoveAndStepASmallAmount (wobbly::Model &model)
    {
        wobbly::Vector const grabPoint (model.Extremes ()[3]);
        wobbly::Anchor anchor (model.GrabAnchor (grabPoint));

        anchor.MoveBy (wobbly::Point (100, 100));

        /* Twenty steps is reasonable */
        for (int i = 0; i < 20; ++i)
            model.Step (16);
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

        wobbly::Model::Settings lowerK = wobbly::Model::DefaultSettings;
        wobbly::Model::Settings higherK = wobbly::Model::DefaultSettings;

        lowerK.springConstant -= 2.0f;

        wobbly::Model lowerSpringKModel (wobbly::Vector (0, 0),
                                         TextureWidth,
                                         TextureHeight,
                                         lowerK);
        wobbly::Model higherSpringKModel (wobbly::Vector (0, 0),
                                          TextureWidth,
                                          TextureHeight,
                                          higherK);

        GrabModelMoveAndStepASmallAmount (lowerSpringKModel);
        GrabModelMoveAndStepASmallAmount (higherSpringKModel);

        EXPECT_GT (bg::get <0> (higherSpringKModel.Extremes ()[0]),
                   bg::get <0> (lowerSpringKModel.Extremes ()[0]));
    }

    TEST (SpringBezierModelSettings, ModelWithLowerFrictionTakesFasterFirstStep)
    {
        wobbly::Model::Settings lowerF = wobbly::Model::DefaultSettings;
        wobbly::Model::Settings higherF = wobbly::Model::DefaultSettings;

        lowerF.friction -= 2.0f;

        wobbly::Model lowerFrictionModel (wobbly::Vector (0, 0),
                                          TextureWidth,
                                          TextureHeight,
                                          lowerF);
        wobbly::Model higherFrictionModel (wobbly::Vector (0, 0),
                                           TextureWidth,
                                           TextureHeight,
                                           higherF);

        GrabModelMoveAndStepASmallAmount (lowerFrictionModel);
        GrabModelMoveAndStepASmallAmount (higherFrictionModel);

        EXPECT_GT (bg::get <0> (lowerFrictionModel.Extremes ()[0]),
                   bg::get <0> (higherFrictionModel.Extremes ()[0]));
    }

    TEST_F (SpringBezierModel, StepZeroReturnsTrueOnGrabbingAndMovingAnchor)
    {
        /* Create an anchor and move it. Step (0) should return true */
        wobbly::Vector const grabPoint (0, 0);
        wobbly::Anchor grab (model.GrabAnchor (grabPoint));

        grab.MoveBy (wobbly::Point (100, 100));
        EXPECT_TRUE (model.Step (0));
    }

    TEST_F (SpringBezierModel, StepZeroReturnsTrueOnNonEquallibriumModel)
    {
        {
            /* Create an anchor and move it. Step (0) should return true */
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (model.GrabAnchor (grabPoint));

            grab.MoveBy (wobbly::Point (100, 100));

            /* Step the model once, this will make the model unequal */
            model.Step (1);

            /* Grab goes away here but the model is still unequal */
        }

        EXPECT_TRUE (model.Step (0));
    }

    class EulerIntegration :
        public Test
    {
        public:

            EulerIntegration () :
                view (storage)
            {
            }

            typedef wobbly::PointView <double const> DCPV;

            SingleObjectStorage storage;
            SingleObjectStorageView view;
    };

    TEST_F (EulerIntegration, ContinueStepWhenObjectsHaveVelocity)
    {
        bg::set <0> (view.velocity, 1.0);

        /* Integrate without any friction */
        EXPECT_TRUE (wobbly::EulerIntegrate (1,
                                             0,
                                             1,
                                             std::move (view.position),
                                             std::move (view.velocity),
                                             DCPV (view.force)));
    }

    TEST_F (EulerIntegration, NoFurtherStepWhenObjectsHaveNoVelocity)
    {
        bg::set <0> (view.velocity, 0.0);

        /* Integrate without any friction */
        EXPECT_FALSE (wobbly::EulerIntegrate (1,
                                              0,
                                              1,
                                              std::move (view.position),
                                              std::move (view.velocity),
                                              DCPV (view.force)));
    }

    TEST_F (EulerIntegration, VelocityIsParabolicWithFriction)
    {
        std::function <double (int)> horizontalVelocityFunction =
            [this](int timestep) -> double {
                bg::assign_point (view.position, wobbly::Point (1, 1));
                bg::assign_point (view.velocity, wobbly::Point (1, 0));

                wobbly::EulerIntegrate (timestep,
                                        1,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (horizontalVelocityFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    TEST_F (EulerIntegration, VelocityIncreasesLinearlyWithConstantForce)
    {
        std::function <double (int)> frictionlessHorizontalVelocityFunction =
            [this](int timestep) -> double {
                /* Reset velocity and force */
                bg::assign (view.velocity, wobbly::Vector (0, 0));
                bg::assign (view.force, wobbly::Vector (1.0f, 0));

                wobbly::EulerIntegrate (timestep,
                                        0,
                                        1,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionlessHorizontalVelocityFunction,
                     SatisfiesModel (Linear <double> ()));
    }

    TEST_F (EulerIntegration, LinearDecreaseInVelocityWithFriction)
    {
        unsigned int nSamples = 10;
        int          range = std::pow (2, nSamples);

        std::function <double (int)> frictionToVelocityFunction =
            [this, range](int frictionAmount) -> double {
                /* Reset velocity and force */
                bg::assign (view.position, wobbly::Point (0, 0));
                bg::assign (view.velocity, wobbly::Vector (0, 0));
                bg::assign (view.force, wobbly::Vector (1.0f, 0));

                double frictionProportion =
                    frictionAmount / static_cast <double> (range);

                /* Step once, but with different frictions, linearly
                 * interpolate between frictionAmount and nSamples
                 * and scale by the default friction value to get
                 * our samples */
                wobbly::EulerIntegrate (16,
                                        frictionProportion,
                                        1.0,
                                        std::move (view.position),
                                        std::move (view.velocity),
                                        DCPV (view.force));

                return bg::get <0> (view.velocity);
            };

        EXPECT_THAT (frictionToVelocityFunction,
                     SatisfiesModel (Linear <double> (),
                                     WithSamples (nSamples)));
    }

    struct MockIntegration
    {
        MockIntegration ()
        {
            EXPECT_CALL (*this, Reset (_)).Times (AtLeast (0));
            EXPECT_CALL (*this, Step (_, _, _, _, _, _)).Times (AtLeast (0));
        }

        MOCK_METHOD1 (Reset, void (size_t));
        MOCK_METHOD6 (Step, bool (size_t,
                                  double,
                                  double,
                                  double,
                                  wobbly::MeshArray       &,
                                  wobbly::MeshArray const &));
    };

    class AnchoredIntegrationLoop :
        public ::testing::Test
    {
        public:

            AnchoredIntegrationLoop () :
                integrator (strategy)
            {
                positions.fill (0);
                forces.fill (0);
            }

            MockIntegration strategy;
            wobbly::AnchoredIntegration <MockIntegration> integrator;

            wobbly::MeshArray positions;
            wobbly::MeshArray forces;
            wobbly::AnchorArray anchors;
    };

    TEST_F (AnchoredIntegrationLoop, ResetIndicesWithAnchor)
    {
        EXPECT_CALL (strategy, Reset (0)).Times (1);
        anchors.Lock (0);

        integrator (positions, forces, anchors, 0.0);
    }

    TEST_F (AnchoredIntegrationLoop, StepUnanchoredPoints)
    {
        EXPECT_CALL (strategy, Step (0, _, _, _, _, _)).Times (1);

        integrator (positions, forces, anchors, 0.0);
    }

    template <typename Integrator>
    class IntegrationStrategy :
        public Test
    {
        public:

            IntegrationStrategy ()
            {
                points.fill (0.0);
                forces.fill (0.0);
            }

            Integrator integrator;
            wobbly::MeshArray points;
            wobbly::MeshArray forces;
    };

    typedef Types <wobbly::EulerIntegration> IntegrationStrategies;
    TYPED_TEST_CASE (IntegrationStrategy, IntegrationStrategies);

    TYPED_TEST (IntegrationStrategy, NoMotionOnReset)
    {
        /* Call the reset () function on the integrator. No changes
         * should occurr on the position at that index */
        wobbly::PointView <double> pointView (TestFixture::points, 0);

        TestFixture::integrator.Reset (0);

        EXPECT_THAT (pointView, Eq (wobbly::Point (0, 0)));
    }

    TYPED_TEST (IntegrationStrategy, EffectiveVelocityChangedToZeroOnReset)
    {
        /* Apply a force once to a frictionless object and integrate it.
         * Call reset and integrate again without any force. The result is
         * no change in position as the velocity was reset */
        wobbly::PointView <double> forceView (TestFixture::forces, 0);
        wobbly::PointView <double> pointView (TestFixture::points, 0);

        /* First apply a force to an object and integrate */
        bg::set <0> (forceView, 1.0);
        TestFixture::integrator.Step (0,
                                      1.0,
                                      1.0,
                                      1.0,
                                      TestFixture::points,
                                      TestFixture::forces);

        wobbly::Point expectedPosition;
        bg::assign_point (expectedPosition, pointView);

        /* Remove force, reset and integrate again */
        bg::set <0> (forceView, 0.0);
        TestFixture::integrator.Reset (0);
        TestFixture::integrator.Step (0,
                                      1.0,
                                      1.0,
                                      1.0,
                                      TestFixture::points,
                                      TestFixture::forces);

        /* After integration, the point should not have moved because
         * it has no velocity */
        EXPECT_THAT (pointView, Eq (expectedPosition));
    }

    TYPED_TEST (IntegrationStrategy, VelocityAffectedWithNewForcesAfterReset)
    {
        wobbly::PointView <double> forceView (TestFixture::forces, 0);
        wobbly::PointView <double> pointView (TestFixture::points, 0);

        wobbly::Point initialPosition;
        bg::assign_point (initialPosition, pointView);

        /* Reset, apply force and integrate */
        bg::set <0> (forceView, 1.0);
        TestFixture::integrator.Reset (0);
        TestFixture::integrator.Step (0,
                                      1.0,
                                      1.0,
                                      1.0,
                                      TestFixture::points,
                                      TestFixture::forces);

        EXPECT_THAT (pointView,
                     Not (Eq (initialPosition)));
    }

    TYPED_TEST (IntegrationStrategy, PositionChangesParabolicallyOverTime)
    {
        wobbly::PointView <double> forceView (TestFixture::forces, 0);
        wobbly::PointView <double> pointView (TestFixture::points, 0);

        std::function <double (int)> frictionlessHorizontalPositionFunction =
            [this, &pointView, &forceView](int timestep) -> double {
                TypeParam integrator;

                /* Reset velocity and force */
                bg::assign (pointView, wobbly::Point (0, 0));
                bg::assign (forceView, wobbly::Vector (1.0f, 0));

                integrator.Step (0,
                                 timestep,
                                 1.0,
                                 1.0,
                                 TestFixture::points,
                                 TestFixture::forces);

                return bg::get <0> (pointView);
            };

        EXPECT_THAT (frictionlessHorizontalPositionFunction,
                     SatisfiesModel (Parabolic <double> ()));
    }

    TEST (SpringStep, ContinueStepWhenSpringsHaveForces)
    {
        /* All points will start at zero, so a positive spring force
         * will already be exerted */
        wobbly::MeshArray positions;
        wobbly::AnchorArray anchors;
        double const springConstant = 1.0;
        double const springFriction = 1.0;
        wobbly::Vector const springDimensions (1.0, 1.0);

        positions.fill (0.0);

        MockIntegration                      integrator;
        wobbly::SpringStep <MockIntegration> stepper (integrator,
                                                      positions,
                                                      springConstant,
                                                      springFriction,
                                                      springDimensions);

        EXPECT_TRUE (stepper (positions, anchors));
    }

    double TileWidth (double width, unsigned int nHorizontalTiles)
    {
        return width / (nHorizontalTiles - 1);
    }

    double TileHeight (double height, unsigned int nVerticalTiles)
    {
        return height / (nVerticalTiles - 1);
    }

    void
    InitializePositionsWithDimensions (wobbly::MeshArray &positions,
                                       double            width,
                                       double            height)
    {
        double const meshWidth = wobbly::config::Width;
        double const meshHeight = wobbly::config::Height;
        double const tileWidth = TileWidth (width, meshWidth);
        double const tileHeight = TileHeight (height, meshHeight);

        for (unsigned int i = 0; i < meshHeight; ++i)
        {
            for (unsigned int j = 0; j < meshWidth; ++j)
            {
                wobbly::PointView <double> pv (positions,
                                               i * meshWidth + j);
                bg::assign_point (pv, wobbly::Point (tileWidth * j,
                                                     tileHeight * i));
            }
        }
    }

    class ConstrainmentStep :
        public Test
    {
        public:

            ConstrainmentStep () :
                range (10),
                width (TextureWidth),
                height (TextureHeight),
                constrainment (range, width, height)
            {
                InitializePositionsWithDimensions (positions,
                                                   TextureWidth,
                                                   TextureHeight);
            }

            wobbly::MeshArray positions;
            wobbly::AnchorArray anchors;

            double                    range;
            double                    width;
            double                    height;
            wobbly::ConstrainmentStep constrainment;
    };

    TEST_F (ConstrainmentStep, PointsNotAffectedWhereNoAnchorGrabbed)
    {
        /* Make a separate copy of the array and test against it later */
        wobbly::MeshArray expectedPositions;

        InitializePositionsWithDimensions (expectedPositions,
                                           TextureWidth,
                                           TextureHeight);

        constrainment (positions, anchors);

        EXPECT_EQ (expectedPositions, positions);
    }

    TEST_F (ConstrainmentStep, ReturnsTrueWhereConstrainmentTookPlace)
    {
        anchors.Lock (0);
        wobbly::PointView <double> pv (positions, 1);
        bg::add_point (pv, wobbly::Point (range * 2, range * 2));

        EXPECT_TRUE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ReturnsFalseWhereNoConstrainmentTookPlace)
    {
        anchors.Lock (0);
        wobbly::PointView <double> pv (positions, 1);

        /* Not enough to cause constrainment */
        bg::add_point (pv, wobbly::Point (range / 2, 0));

        EXPECT_FALSE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ReferencePointsFromFirstAnchor)
    {
        anchors.Lock (0);
        wobbly::PointView <double> pv (positions, 0);
        wobbly::Point const movement (range * 2, 0);

        /* Move the anchored point right by range * 2, the result should
         * be that every other point is p.x + range */
        bg::add_point (pv, movement);
        constrainment (positions, anchors);

        wobbly::MeshArray expectedPositions;

        InitializePositionsWithDimensions (expectedPositions,
                                           TextureWidth,
                                           TextureHeight);

        /* Add movement to the first point */
        wobbly::PointView <double> epv (expectedPositions, 0);
        bg::add_point (epv, movement);

        /* Add range.x to each point starting from anchor + 1 */
        for (size_t i = 1; i < positions.size () / 2; ++i)
        {
            wobbly::PointView <double> pv (expectedPositions, i);
            bg::add_point (pv, wobbly::Point (range, 0));
        }

        std::vector <Matcher <double>> matchers;
        for (double expected : expectedPositions)
            matchers.push_back (::testing::DoubleEq (expected));

        EXPECT_THAT (positions, ElementsAreArray (&matchers[0],
                                                  matchers.size ()));
    }

    typedef std::tuple <size_t, double> ConstrainmentStepPositionsParam;

    class ConstrainmentStepPositions :
        public ConstrainmentStep,
        public WithParamInterface <ConstrainmentStepPositionsParam>
    {
        public:

            ConstrainmentStepPositions () :
                index (std::get <0> (GetParam ())),
                ratio (std::get <1> (GetParam ()))
            {
                anchors.Lock ((wobbly::config::Width *
                               wobbly::config::Height) / 2);
            }

            size_t index;
            double ratio;
    };

    TEST_P (ConstrainmentStepPositions, NotAffectedWhereWithinRadiusOfRange)
    {
        double const absratio = std::fabs (ratio);

        /* Prevent zero-division. Treat zero as positive */
        double const sign = ratio / (absratio + (absratio == 0.0));
        double const radiusInRange = range - (range / 2);

        wobbly::PointView <double> pv (positions, index);
        wobbly::Point expected;

        bg::add_point (pv,
                       wobbly::Point (radiusInRange * ratio,
                                      radiusInRange * (1 - absratio) * sign));

        /* Expected point is the modified point here, before constrainment */
        bg::assign (expected, pv);

        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    TEST_P (ConstrainmentStepPositions, AffectedWhereOutsideRadiusOfRange)
    {
        double absratio = std::fabs (ratio);

        /* Prevent zero-division. Treat zero as positive */
        double const sign = ratio / (absratio + (absratio == 0.0));
        double const radiusOutOfRange = range * range;

        wobbly::Point outOfRange (radiusOutOfRange * ratio,
                                  radiusOutOfRange * (1 - absratio) * sign);
        wobbly::Point inRange (range * ratio,
                               range * (1 - absratio) * sign);

        wobbly::PointView <double> pv (positions, index);
        wobbly::Point expected;

        /* Expected point is the actual grid point, but at its maximum range */
        bg::assign (expected, pv);
        bg::add_point (expected, inRange);

        bg::add_point (pv, outOfRange);
        constrainment (positions, anchors);

        EXPECT_THAT (pv, Eq (expected));
    }

    ConstrainmentStepPositionsParam const constrainmentStepParams[] =
    {
        ConstrainmentStepPositionsParam (0, -1.0),
        ConstrainmentStepPositionsParam (wobbly::config::Width - 1, 1.0),
        ConstrainmentStepPositionsParam ((wobbly::config::Height - 1) *
                                         wobbly::config::Width, 0.0),
        ConstrainmentStepPositionsParam (wobbly::config::Width *
                                         wobbly::config::Height - 1, 0.0)
    };

    INSTANTIATE_TEST_CASE_P (Extremes, ConstrainmentStepPositions,
                             ValuesIn (constrainmentStepParams));

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
            void ApplyTransformation (MeshTransform const &);

            std::function <double (int)>
            UnitDeformationFunction (ResultFactory const      &resultFactory,
                                     wobbly::BezierMesh const &mesh,
                                     unsigned int             nSamples);
    };

    void
    BezierMesh::SetUp ()
    {
        InitializePositionsWithDimensions (mesh.PointArray (),
                                           TextureWidth,
                                           TextureHeight);
    }

    void
    BezierMesh::ApplyTransformation (MeshTransform const &transform)
    {
        for (size_t i = 0; i < wobbly::config::Height; ++i)
        {
            for (size_t j = 0; j < wobbly::config::Width; ++j)
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
            [nSamples, &mesh, result](int lookupValue) -> double {
                /* Divide by the total number of samples (2^10) in order to get
                 * the unit value. */

                double const unitLookupValue =
                    lookupValue / static_cast <double> (pow (2, nSamples));
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
                                     WithTolerance (10e-5)));
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
                                     WithTolerance (10e-5)));
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
                                          WithTolerance (10e-5))));
    }

    TEST_F (BezierMesh, ExtremesAreTextureEdges)
    {
        std::array <wobbly::Point, 4> const extremes = mesh.Extremes ();
        Matcher <wobbly::Point const &> const textureEdges[] =
        {
            Eq (wobbly::Point (0, 0)),
            Eq (wobbly::Point (TextureWidth, 0)),
            Eq (wobbly::Point (0, TextureHeight)),
            Eq (wobbly::Point (TextureWidth, TextureHeight))
        };

        EXPECT_THAT (extremes, ElementsAreArray (textureEdges));
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

        EXPECT_THAT (interpolated, Eq (prediction));
    }

    wobbly::Point const meshExtremes[] =
    {
        wobbly::Point (0.0, 0.0),
        wobbly::Point (0.0, 1.0),
        wobbly::Point (1.0, 0.0),
        wobbly::Point (1.0, 1.0)
    };

    INSTANTIATE_TEST_CASE_P (Extremes,
                             BezierMeshPoints,
                             ValuesIn (meshExtremes));
}
