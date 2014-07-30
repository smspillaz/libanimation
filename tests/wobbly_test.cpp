/*
 * tests/wobbly_test.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <functional>
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
using ::testing::Combine;
using ::testing::ElementsAreArray;
using ::testing::ExitedWithCode;
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
using ::testing::Values;
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
    class DoublePointView :
        public Test,
        public WithParamInterface <size_t>
    {
    public:

        DoublePointView () :
            pointOffset (GetParam ()),
            arrayOffset (GetParam () * 2)
        {
            array.fill (0);
        }

        std::array <double, 4> array;
        size_t                 pointOffset;
        size_t                 arrayOffset;
    };

    TEST_P (DoublePointView, WriteXWithOffset)
    {
        wobbly::PointView <double> pv (array, pointOffset);
        bg::set <0> (pv, 1);

        EXPECT_EQ (array[arrayOffset], 1);
    }

    TEST_P (DoublePointView, WriteYWithOffset)
    {
        wobbly::PointView <double> pv (array, pointOffset);
        bg::set <1> (pv, 1);

        EXPECT_EQ (array[arrayOffset + 1], 1);
    }

    INSTANTIATE_TEST_CASE_P (DuplexMeshArray, DoublePointView,
                             Values (0, 1));

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

    TEST (Spring, MoveAssignToSelf)
    {
        SingleObjectStorage storageA, storageB;

        EXPECT_EXIT ({
            wobbly::Spring a (storageA.Force (),
                              storageB.Force (),
                              storageA.Position (),
                              storageB.Position (),
                              wobbly::Vector (0, 0));

            a = std::move (a);
            exit (0);
        }, ExitedWithCode (0), "");
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

    double TileWidth (double width)
    {
        return width / (wobbly::config::Width - 1);
    }

    double TileHeight (double height)
    {
        return height / (wobbly::config::Height - 1);
    }

    double const EvenSize = 100.0f;

    class EvenlyDistributedMesh :
        public Test
    {
        public:

            EvenlyDistributedMesh () :
                mesh (GetArray ())
            {
            }

            static wobbly::MeshArray GetArray ()
            {
                wobbly::MeshArray mesh;

                wobbly::Vector size (TileWidth (EvenSize),
                                     TileHeight (EvenSize));
                wobbly::mesh::CalculatePositionArray (wobbly::Point (0, 0),
                                                      mesh,
                                                      size);

                return mesh;
            }

            wobbly::MeshArray mesh;
    };

    struct ClosestIndexToPositionParam
    {
        wobbly::Point point;
        size_t        expectedIndex;
    };

    class ClosestIndexToPosition :
        public EvenlyDistributedMesh,
        public WithParamInterface <ClosestIndexToPositionParam>
    {
    public:

        static std::vector <ParamType> NoTransform ()
        {
            return GetParams ([](wobbly::Point const &p) {
            });
        }

        static std::vector <ParamType> Expanded ()
        {
            wobbly::Vector translation (EvenSize / 2, EvenSize / 2);

            return GetParams ([&translation](wobbly::Point &point) {
                 /* Scale on center */
                bg::subtract_point (point, translation);
                /* Older versions of cppcheck have trouble seeing through
                 * the lambda */
                // cppcheck-suppress unreachableCode
                bg::multiply_value (point, 1.1);
                bg::add_point (point, translation);
            });
        }

        static std::vector <ParamType> Shrinked ()
        {
            wobbly::Vector translation (EvenSize / 2, EvenSize / 2);

            return GetParams ([&translation](wobbly::Point &point) {
                 /* Scale on center */
                bg::subtract_point (point, translation);
                /* Older versions of cppcheck have trouble seeing through
                 * the lambda */
                // cppcheck-suppress unreachableCode
                bg::divide_value (point, 1.1);
                bg::add_point (point, translation);
            });
        }

    private:

        template <typename Transformation>
        static std::vector <ParamType> GetParams (Transformation const &trans)
        {
            std::vector <ParamType> vec;
            auto array (GetArray ());

            auto const total = wobbly::config::TotalIndices;

            for (size_t i = 0; i < total; ++i)
            {
                wobbly::PointView <double> pv (array, i);
                ParamType param;

                bg::assign (param.point, pv);
                param.expectedIndex = i;

                trans (param.point);

                vec.push_back (param);
            }

            return vec;
        }
    };

    TEST_P (ClosestIndexToPosition, Find)
    {
        EXPECT_EQ (GetParam ().expectedIndex,
                   wobbly::mesh::ClosestIndexToPosition (mesh,
                                                         GetParam ().point));
    }

    /* First case is just the mesh points themselves */
    INSTANTIATE_TEST_CASE_P (MeshPoints, ClosestIndexToPosition,
                             ValuesIn (ClosestIndexToPosition::NoTransform ()));

    /* Second case is the mesh points scaled outwards by a little bit */
    INSTANTIATE_TEST_CASE_P (ExpandedPoints, ClosestIndexToPosition,
                             ValuesIn (ClosestIndexToPosition::Expanded ()));

    /* Third case is the mesh points scaled outwards by a little bit */
    INSTANTIATE_TEST_CASE_P (ShrinkedPoints, ClosestIndexToPosition,
                             ValuesIn (ClosestIndexToPosition::Shrinked ()));

    class SpringMesh :
        public EvenlyDistributedMesh
    {
    public:

        SpringMesh () :
            firstPreference ([](wobbly::Spring const &spring) {
                return spring.FirstPosition ();
            }),
            secondPreference ([](wobbly::Spring const &spring) {
                return spring.SecondPosition ();
            }),
            springMesh (mesh,
                        wobbly::Vector (TileWidth (EvenSize),
                                        TileHeight (EvenSize)))
        {
        }

        static void ApplyMovement (std::unique_ptr <double[]> &ptr,
                                   wobbly::Vector const       &movement)
        {
            wobbly::PointView <double> pv (ptr.get (), 0);
            bg::add_point (pv, movement);
        }

        wobbly::SpringMesh::PosPreference firstPreference;
        wobbly::SpringMesh::PosPreference secondPreference;

        wobbly::SpringMesh  springMesh;
    };

    template <typename P1, typename P2>
    class PointsInSameDirectionMatcher
    {
        public:

            PointsInSameDirectionMatcher (P1 const &origin,
                                          P2 const &candidate) :
                origin (origin),
                candidate (candidate)
            {
            }

            template <typename Vector>
            bool MatchAndExplain (Vector        const &vec,
                                  MatchResultListener *listener) const
            {
                /* A vector points towards another point if tan(theta) formed
                 * by the two components is the same as tan(theta) formed by the
                 * component distance from candidate to origin */

                wobbly::Vector delta;
                bg::assign (delta, candidate);
                bg::fixups::subtract_point (delta, origin);

                auto vecTanTheta = bg::get <1> (vec) / bg::get <0> (vec);
                auto distTanTheta = bg::get <1> (delta) / bg::get <0> (delta);

                if (listener)
                    *listener << "vector (" << vec << ")'s tan(theta) is "
                              << vecTanTheta << " and tan(theta) between origin"
                              << " (" << origin << ") and candidate ("
                              << candidate << ")" << " with delta ("
                              << delta << ") is " << distTanTheta;

                namespace btt = boost::test_tools;
                typedef decltype (vecTanTheta) NumericType;
                auto tolerance = btt::percent_tolerance (10e-9);
                auto within  =
                    btt::close_at_tolerance <NumericType> (tolerance);
                return within (vecTanTheta, distTanTheta);
            }

            void DescribeTo (::std::ostream *os) const
            {
                *os << "point in the same direction as the line formed by "
                    << origin
                    << " and "
                    << candidate;
            }

            void DescribeNegationTo (::std::ostream *os) const
            {
                *os << "does not ";
                DescribeTo (os);
            }

        private:

            P1 const &origin;
            P2 const &candidate;
    };

    template <typename P1, typename P2>
    inline PolymorphicMatcher <PointsInSameDirectionMatcher <P1, P2> >
    PointsInSameDirection (P1 const &origin, P2 const &candidate)
    {
        PointsInSameDirectionMatcher <P1, P2> matcher (origin, candidate);
        return MakePolymorphicMatcher (matcher);
    }

    /* We can observe the insertion of temporary anchors by looking at
     * the direction of a first application of force. */

    /* When a temporary anchor is inserted, the points on which
     * the anchor was inserted should have a net force pointing
     * towards the anchor's left or right neighbour */
    TEST_F (SpringMesh, ForceOnPointsTowardsTemporaryAnchor)
    {
        /* Insert a temporary anchor between points (1) and (2) on the grid */
        wobbly::Point const install (EvenSize / 2, 0);
        wobbly::Point const movement (25, -50);
        auto handle (springMesh.InstallAnchorSprings (install,
                                                      firstPreference,
                                                      secondPreference));
        ApplyMovement (handle.data, movement);

        auto result = springMesh.CalculateForces (SpringConstant);

        wobbly::Point anchorPosition (install);
        bg::add_point (anchorPosition, movement);

        wobbly::Vector const leftOffset (-TileWidth (EvenSize) / 2, 0);
        wobbly::Vector const rightOffset (TileWidth (EvenSize) / 2, 0);

        wobbly::Point anchorLeftPoint (anchorPosition);
        bg::add_point (anchorLeftPoint, leftOffset);

        wobbly::Point anchorRightPoint (anchorPosition);
        bg::add_point (anchorRightPoint, rightOffset);

        wobbly::PointView <double const> firstPoint (mesh, 1);
        wobbly::PointView <double const> secondPoint (mesh, 2);

        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 1),
                     PointsInSameDirection (firstPoint, anchorLeftPoint));
        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 2),
                     PointsInSameDirection (secondPoint, anchorRightPoint));
    }

    /* Insert a second temporary anchor in between a temporary
     * anchor and an actual point on the grid. The result should be that
     * upon moving the second temporary anchor and applying forces
     * for the first time that a force will be directed towards the first
     * anchor from the first base point (1) and towards the second anchor
     * from the second base point (2) */
    TEST_F (SpringMesh, InsertTemporaryAnchorOnTemporarySpring)
    {
        /* Insert first temporary anchor between points (1) and (2) on
         * the grid. */
        wobbly::Point const firstInstall (EvenSize / 2, 0);
        auto firstHandle (springMesh.InstallAnchorSprings (firstInstall,
                                                           firstPreference,
                                                           secondPreference));
        /* Insert second temporary anchor between the first temporary
         * spring's anchor and base point (2) */
        wobbly::Point const secondInstall (EvenSize / 2 +
                                               (TileWidth (EvenSize) / 4),
                                           0);
        auto secondHandle (springMesh.InstallAnchorSprings (secondInstall,
                                                            firstPreference,
                                                            secondPreference));

        /* Move first handle to point above point (1) and second anchor
         * to point above point (2) */
        wobbly::Vector const firstMovement (-25, -50);
        wobbly::Vector const secondMovement (25, -50);

        wobbly::Point firstAnchorPoint (firstInstall);
        bg::add_point (firstAnchorPoint, firstMovement);
        ApplyMovement (firstHandle.data, firstMovement);

        wobbly::Point secondAnchorPoint (secondInstall);
        bg::add_point (secondAnchorPoint, secondMovement);
        ApplyMovement (secondHandle.data, secondMovement);

        /* Calculate forces */
        auto result = springMesh.CalculateForces (SpringConstant);

        /* The desired delta between the first point and its base neighbour is
         * TileWidth / 2, 0 */
        wobbly::Point leftOfFirstAnchor (-TileWidth (EvenSize) / 2, 0);
        bg::add_point (leftOfFirstAnchor, firstAnchorPoint);

        /* For the second point, because we inserted it between the first
         * spring and the second base point, the desired delta will be half
         * of the first spring length, eg, TileWidth (EvenSize) / 4, 0 */
        wobbly::Point rightOfSecondAnchor (TileWidth (EvenSize) / 4, 0);
        bg::add_point (rightOfSecondAnchor, secondAnchorPoint);

        wobbly::PointView <double const> firstPoint (mesh, 1);
        wobbly::PointView <double const> secondPoint (mesh, 2);

        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 1),
                     PointsInSameDirection (firstPoint, leftOfFirstAnchor));
        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 2),
                     PointsInSameDirection (secondPoint, rightOfSecondAnchor));
    }

    /* Same setup as the previous test, but this time move the anchors around
     * but let the second one expire. The force should revert back to the way
     * it was in the first test */
    TEST_F (SpringMesh, OnDestructionOfTemporarySpringForceRevertsToFirstAnchor)
    {
        typedef wobbly::SpringMesh::InstallResult IR;
        typedef std::unique_ptr <wobbly::SpringMesh::InstallResult> Handle;

        auto &fp (firstPreference);
        auto &sp (secondPreference);

        /* Insert first temporary anchor between points (1) and (2) on
         * the grid. */
        wobbly::Point const firstInstall (EvenSize / 2, 0);
        Handle first (new IR (springMesh.InstallAnchorSprings (firstInstall,
                                                               fp,
                                                               sp)));

        /* Insert second temporary anchor between the first temporary
         * spring's anchor and base point (2) */
        wobbly::Point const secondInstall (EvenSize / 2 +
                                               (TileWidth (EvenSize) / 4),
                                           0);
        Handle second (new IR (springMesh.InstallAnchorSprings (secondInstall,
                                                                fp,
                                                                sp)));

        /* Move first handle to point above point (1) and second anchor
         * to point above point (2) */
        wobbly::Vector const firstMovement (-25, -50);
        wobbly::Vector const secondMovement (25, -50);

        wobbly::Point firstAnchorPoint (firstInstall);
        bg::add_point (firstAnchorPoint, firstMovement);
        ApplyMovement (first->data, firstMovement);

        wobbly::Point secondAnchorPoint (secondInstall);
        bg::add_point (secondAnchorPoint, secondMovement);
        ApplyMovement (second->data, secondMovement);

        /* Now that both handles have been moved, make the second one expire */
        second.reset ();

        /* Calculate forces */
        auto result = springMesh.CalculateForces (SpringConstant);

        wobbly::Point anchorPosition (firstInstall);
        bg::add_point (anchorPosition, firstMovement);

        wobbly::Vector const leftOffset (-TileWidth (EvenSize) / 2, 0);
        wobbly::Vector const rightOffset (TileWidth (EvenSize) / 2, 0);

        wobbly::Point anchorLeftPoint (firstAnchorPoint);
        bg::add_point (anchorLeftPoint, leftOffset);

        wobbly::Point anchorRightPoint (firstAnchorPoint);
        bg::add_point (anchorRightPoint, rightOffset);

        wobbly::PointView <double const> firstPoint (mesh, 1);
        wobbly::PointView <double const> secondPoint (mesh, 2);

        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 1),
                     PointsInSameDirection (firstPoint, anchorLeftPoint));
        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 2),
                     PointsInSameDirection (secondPoint, anchorRightPoint));
    }

    TEST_F (SpringMesh, HandlesCanExpireInNonReverseOrder)
    {
        EXPECT_EXIT ({
            wobbly::Point const install (EvenSize / 2, 0);
            typedef wobbly::SpringMesh::InstallResult IR;
            typedef std::unique_ptr <wobbly::SpringMesh::InstallResult> Handle;

            auto &fp (firstPreference);
            auto &sp (secondPreference);
            Handle first (new IR (springMesh.InstallAnchorSprings (install,
                                                                   fp,
                                                                   sp)));
            Handle second (new IR (springMesh.InstallAnchorSprings (install,
                                                                    fp,
                                                                    sp)));
            first.reset ();
            second.reset ();
            exit (0);
        }, ExitedWithCode (0), "");
    }

    TEST_F (SpringMesh, ForceBackToNeutralWhenHandlesExpireInNonReverseOrder)
    {
        /* All temporary, handles will expire in reverse order */
        {
            typedef wobbly::SpringMesh::InstallResult IR;
            typedef std::unique_ptr <wobbly::SpringMesh::InstallResult> Handle;

            auto &fp (firstPreference);
            auto &sp (secondPreference);

            /* Insert first temporary anchor between points (1) and (2) on
             * the grid. */
            wobbly::Point const firstInstall (EvenSize / 2, 0);
            Handle first (new IR (springMesh.InstallAnchorSprings (firstInstall,
                                                                   fp,
                                                                   sp)));

            /* Insert second temporary anchor between the first temporary
             * spring's anchor and base point (2) */
            wobbly::Point const secondInst (EvenSize / 2 +
                                                (TileWidth (EvenSize) / 4),
                                            0);
            Handle second (new IR (springMesh.InstallAnchorSprings (secondInst,
                                                                    fp,
                                                                    sp)));

            /* Move first handle to point above point (1) and second anchor
             * to point above point (2) */
            wobbly::Vector const firstMovement (-25, -50);
            wobbly::Vector const secondMovement (25, -50);

            wobbly::Point firstAnchorPoint (firstInstall);
            bg::add_point (firstAnchorPoint, firstMovement);
            ApplyMovement (first->data, firstMovement);

            wobbly::Point secondAnchorPoint (secondInst);
            bg::add_point (secondAnchorPoint, secondMovement);
            ApplyMovement (second->data, secondMovement);

            /* Kill the first handle before the second */
            first.reset ();
            second.reset ();
        }

        /* Calculate forces */
        auto result = springMesh.CalculateForces (SpringConstant);

        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 1),
                     Eq (wobbly::Point (0, 0)));
        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 2),
                     Eq (wobbly::Point (0, 0)));
    }

    /* This tests that if we pass in a different preference for a
     * desired distance for this spring, that force points in accordance
     * with that preference */
    TEST_F (SpringMesh, DesiredDistanceCanBeDifferentToSpringPositionAtGrab)
    {
        /* Insert a temporary anchor between points (1) and (2) on the grid */
        wobbly::Point const install (EvenSize / 2, 0);
        wobbly::Point const movement (25, -50);
        wobbly::Vector const desiredOffset (25, 0);

        std::array <double, 4> points;

        typedef wobbly::PointView <double const> DCPV;
        typedef DCPV const & (wobbly::Spring::*Get) () const;

        auto const prefOffset =
            [this, &desiredOffset, &points](wobbly::Spring const &spring,
                                            Get                  get,
                                            size_t               offset) {
                wobbly::PointView <double> pv (points, offset);
                bg::assign (pv, (spring.*get) ());
                bg::add_point (pv, desiredOffset);

                return wobbly::PointView <double const> (points, offset);
            };

        using namespace std::placeholders;

        wobbly::SpringMesh::PosPreference firstPref =
            std::bind (prefOffset, _1, &wobbly::Spring::FirstPosition, 0);

        wobbly::SpringMesh::PosPreference secondPref =
            std::bind (prefOffset, _1, &wobbly::Spring::SecondPosition, 1);

        auto handle (springMesh.InstallAnchorSprings (install,
                                                      firstPref,
                                                      secondPref));

        ApplyMovement (handle.data, movement);

        auto result = springMesh.CalculateForces (SpringConstant);

        wobbly::Point anchorPosition (install);
        bg::add_point (anchorPosition, movement);

        wobbly::Vector const leftOffset (-TileWidth (EvenSize) / 2, 0);
        wobbly::Vector const rightOffset (TileWidth (EvenSize) / 2, 0);

        wobbly::Point anchorLeftPoint (anchorPosition);
        bg::add_point (anchorLeftPoint, leftOffset);
        bg::add_point (anchorLeftPoint, desiredOffset);

        wobbly::Point anchorRightPoint (anchorPosition);
        bg::add_point (anchorRightPoint, rightOffset);
        bg::add_point (anchorRightPoint, desiredOffset);

        /* We don't add anything to firstPoint and secondPoint here
         * because there will be an offset of desiredOffset from
         * the mesh already in the other direction, eg
         *
         * (newPosition - oldPosition) - delta.
         *
         * If we add delta to these points (eg, oldPosition) then
         * this will not model the equation correctly */
        wobbly::PointView <double const> firstPoint (mesh, 1);
        wobbly::PointView <double const> secondPoint (mesh, 2);

        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 1),
                     PointsInSameDirection (firstPoint, anchorLeftPoint));
        EXPECT_THAT (wobbly::PointView <double const> (result.forces, 2),
                     PointsInSameDirection (secondPoint, anchorRightPoint));
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

        anchor->MoveBy (wobbly::Vector (1, 1));
        auto point (GetTruncatedDeformedCenter (model));

        EXPECT_THAT (point,
                     Not (Eq (TextureCenter)));
    }

    typedef std::tuple <wobbly::Point, wobbly::Point, size_t> SpringGrabParams;

    class SpringBezierModelGrabPositions :
        public SpringBezierModel,
        public WithParamInterface <SpringGrabParams>
    {
        public:

            SpringBezierModelGrabPositions () :
                grabPosition (std::get <0> (GetParam ())),
                movement (std::get <1> (GetParam ())),
                extremeIndex (std::get <2> (GetParam ()))
            {
            }

            wobbly::Point const &grabPosition;
            wobbly::Point const &movement;
            size_t              extremeIndex;
    };

    /* Only tests the GrabIndex strategy */
    TEST_P (SpringBezierModelGrabPositions, GrabsCorrectIndex)
    {
        wobbly::Anchor grab (model.GrabAnchor (grabPosition));
        grab->MoveBy (movement);

        wobbly::Point transformed (grabPosition);
        bg::add_point (transformed, movement);

        EXPECT_THAT (model.Extremes ()[extremeIndex],
                     Eq (transformed));
    }

    TEST_P (SpringBezierModelGrabPositions, SettlesAtCurrentlyAnchoredPosition)
    {
        /* Check that when we grab the model from each of the four corners
         * and move that anchor by 100, 100 that the model always settles
         * at exactly 100, 100
         *
         * While exact positioning isn't possible without anchors grabbed,
         * it is almost always desired in this case */
        wobbly::Anchor grab (model.GrabAnchor (grabPosition));
        grab->MoveBy (wobbly::Vector (100, 100));

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

    INSTANTIATE_TEST_CASE_P (Extremes, SpringBezierModelGrabPositions,
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

    /* Tests for both the InstallPoint and GrabAnchor strategies */
    template <typename AnchorStrategyFactory>
    class SpringBezierModelAnchorStrategy :
        public SpringBezierModel
    {
        public:

            AnchorStrategyFactory createAnchorFor;
    };

    /* GrabAnchorStrategy grab a single point on the mesh and move it */
    struct GrabAnchorStrategyFactory
    {
        wobbly::Anchor operator () (wobbly::Model       &model,
                                    wobbly::Point const &grabPoint)
        {
            return model.GrabAnchor (grabPoint);
        }
    };

    /* InstallAnchorStrategy installs a new anchor on the mesh */
    struct InstallAnchorStrategyFactory
    {
        wobbly::Anchor operator () (wobbly::Model       &model,
                                    wobbly::Point const &grabPoint)
        {
            return model.InsertAnchor (grabPoint);
        }
    };

    typedef ::testing::Types <GrabAnchorStrategyFactory,
                              InstallAnchorStrategyFactory> AnchorStrategyTypes;

    TYPED_TEST_CASE (SpringBezierModelAnchorStrategy, AnchorStrategyTypes);

    /* We can verify this by grabbing an anchor at a known position
     * and then resizing the model. If the model has net force, then
     * the anchor did not move */
    TYPED_TEST (SpringBezierModelAnchorStrategy, AnchorNotMovedAfterResize)
    {
        wobbly::Vector const grabPoint (TextureWidth,
                                        TextureHeight);
        wobbly::Anchor grab (this->createAnchorFor (this->model, grabPoint));

        this->model.ResizeModel (TextureWidthAfterResize,
                                TextureHeightAfterResize);

        EXPECT_TRUE (this->model.Step (1));
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
                *os << "is";
                Describe (*os);
            }

            void DescribeNegationTo (std::ostream *os) const
            {
                *os << "is not";
                Describe (*os);
            }

        private:

            void Describe (std::ostream &os) const
            {
                os << " within :" << std::endl;
                bg::model::polygon <wobbly::Point> poly;
                bg::assign (poly, parent);
                bg::for_each_point (poly, PrintPoint (os));
            }

            class PrintPoint
            {
                public:

                    PrintPoint (std::ostream &os) :
                        os (os)
                    {
                    }

                    template <typename Point>
                    void operator () (Point const &p)
                    {
                        os << " - " << p << std::endl;
                    }

                private:

                   std::ostream &os;
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
    TYPED_TEST (SpringBezierModelAnchorStrategy, TargetWithinRangeGrabbed)
    {
        /* Create an anchor on 0, 0 and then move it to 100, 100, then move
         * it back to 0, 0. The end result should be that the model position
         * will end up back at 0, 0. We can't observe the target positions
         * so we need to do it this way */

        wobbly::Vector const grabPoint (0, 0);
        wobbly::Anchor grab (this->createAnchorFor (this->model, grabPoint));

        grab->MoveBy (wobbly::Point (100, 100));
        this->model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (this->model.Step (1));

        EXPECT_THAT (this->model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    /* The only way we can test this is to perform operations dependent
     * on a target position and ensure that they are precise to the grab's
     * position */
    TYPED_TEST (SpringBezierModelAnchorStrategy, AnchorsChangesDontAffectTarget)
    {
        /* Create an anchor on 0, 0, then at TextureWidth, 0 and move it by
         * 100, 100, then move the model it back to 0, 0. This checks if
         * the TargetPosition machinery is able to handle different anchor
         * grabs */

        {
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (this->createAnchorFor (this->model,
                                                        grabPoint));
        }

        wobbly::Vector const grabPoint (TextureWidth, 0);
        wobbly::Anchor grab (this->createAnchorFor (this->model,
                                                    grabPoint));

        grab->MoveBy (wobbly::Point (100, 100));
        this->model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (this->model.Step (1));

        EXPECT_THAT (this->model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    TYPED_TEST (SpringBezierModelAnchorStrategy, TargetRemainsAfterRelease)
    {
        /* This time integrate the model for a short period while grabbed
         * and then move it to a new position. This should still cause its
         * target position to end up roughly in the same place */
        {
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (this->createAnchorFor (this->model,
                                                        grabPoint));

            grab->MoveBy (wobbly::Point (100, 100));
            this->model.Step (2);
        }

        this->model.MoveModelTo (wobbly::Point (0, 0));

        /* Wait until the model has completely settled */
        while (this->model.Step (1));

        EXPECT_THAT (this->model.Extremes ()[0],
                     WithinGeometry (PointBox (wobbly::Point (-1.5, -1.5),
                                               wobbly::Point (1.5, 1.5))));
    }

    TYPED_TEST (SpringBezierModelAnchorStrategy, ForcesExistAfterMovingAnchor)
    {
        /* Create an anchor and move it. Step (0) should return true */
        wobbly::Vector const grabPoint (0, 0);
        wobbly::Anchor grab (this->createAnchorFor (this->model,
                                                    grabPoint));

        grab->MoveBy (wobbly::Point (100, 100));
        EXPECT_TRUE (this->model.Step (0));
    }

    TYPED_TEST (SpringBezierModelAnchorStrategy, ForcesExistAfterReleaseAnchor)
    {
        {
            /* Create an anchor and move it. Step (0) should return true */
            wobbly::Vector const grabPoint (0, 0);
            wobbly::Anchor grab (this->createAnchorFor (this->model,
                                                        grabPoint));

            grab->MoveBy (wobbly::Point (100, 100));

            /* Step the model once, this will make the model unequal */
            this->model.Step (1);

            /* Grab goes away here but the model is still unequal */
        }

        EXPECT_TRUE (this->model.Step (0));
    }

    void GrabModelMoveAndStepASmallAmount (wobbly::Model &model)
    {
        wobbly::Vector const grabPoint (model.Extremes ()[3]);
        wobbly::Anchor anchor (model.GrabAnchor (grabPoint));

        anchor->MoveBy (wobbly::Point (100, 100));

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

        wobbly::MeshArray & Velocities ()
        {
            static wobbly::MeshArray array;
            return array;
        }
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
        wobbly::Vector const springDimensions (10.0, 10.0);

        positions.fill (0.0);

        MockIntegration                      integrator;
        wobbly::SpringStep <MockIntegration> stepper (integrator,
                                                      positions,
                                                      springConstant,
                                                      springFriction,
                                                      springDimensions);

        EXPECT_TRUE (stepper (positions, anchors));
    }

    void
    InitializePositionsWithDimensions (wobbly::MeshArray &positions,
                                       double            width,
                                       double            height)
    {
        double const meshWidth = wobbly::config::Width;
        double const meshHeight = wobbly::config::Height;
        double const tileWidth = TileWidth (width);
        double const tileHeight = TileHeight (height);

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
                targets ([this](wobbly::MeshArray &mesh){
                    InitializePositionsWithDimensions (mesh,
                                                       TextureWidth,
                                                       TextureHeight);
                }),
                constrainment (range, targets)
            {
                InitializePositionsWithDimensions (positions,
                                                   TextureWidth,
                                                   TextureHeight);
            }

            double                    range;
            double                    width;
            double                    height;

            wobbly::TargetMesh targets;
            wobbly::MeshArray positions;
            wobbly::AnchorArray anchors;

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
        auto handleOwner (targets.Activate ());
        wobbly::PointView <double> pv (positions, 1);
        bg::add_point (pv, wobbly::Point (range * 2, range * 2));

        EXPECT_TRUE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ReturnsFalseWhereNoConstrainmentTookPlace)
    {
        auto handleOwner (targets.Activate ());
        wobbly::PointView <double> pv (positions, 1);

        /* Not enough to cause constrainment */
        bg::add_point (pv, wobbly::Point (range / 2, 0));

        EXPECT_FALSE (constrainment (positions, anchors));
    }

    TEST_F (ConstrainmentStep, ConstrainedToPointsOnTargetMesh)
    {
        wobbly::Point const movement (range * 2, 0);
        auto handleOwner (targets.Activate ());
        wobbly::MoveOnly <wobbly::TargetMesh::Move> &handleWrap (handleOwner);
        wobbly::TargetMesh::Move moveBy (handleWrap);

        moveBy (movement);

        /* Move the anchored point right by range * 2, the result should
         * be that every other point is p.x + range */
        constrainment (positions, anchors);

        wobbly::MeshArray expectedPositions;

        InitializePositionsWithDimensions (expectedPositions,
                                           TextureWidth,
                                           TextureHeight);

        /* Add range.x to each point */
        for (size_t i = 0; i < positions.size () / 2; ++i)
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
                handle (targets.Activate ()),
                index (std::get <0> (GetParam ())),
                ratio (std::get <1> (GetParam ()))
            {
                anchors.Lock ((wobbly::config::Width *
                               wobbly::config::Height) / 2);
            }

            wobbly::TargetMesh::Hnd handle;
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

    wobbly::Point const unitMeshExtremes[] =
    {
        wobbly::Point (0.0, 0.0),
        wobbly::Point (0.0, 1.0),
        wobbly::Point (1.0, 0.0),
        wobbly::Point (1.0, 1.0)
    };

    INSTANTIATE_TEST_CASE_P (UnitExtremes,
                             BezierMeshPoints,
                             ValuesIn (unitMeshExtremes));
}
