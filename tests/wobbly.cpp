/*
 * tests/wobbly.cpp
 *
 * Tests for the "wobbly" spring model.
 *
 * See LICENCE.md for Copyright information.
 */
#include <functional>
#include <iostream>
#include <gmock/gmock.h>
#include <boost/geometry/algorithms/equals.hpp>
#include <boost/geometry/util/for_each_coordinate.hpp>

#include <mathematical_model_matcher.h>

#include "wobbly.h"

using ::testing::Not;
using ::testing::Test;

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
            operator<< (std::ostream &lhs, const point <C, D, S> &p)
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

namespace
{
    class LockedObject :
        public ::testing::Test
    {
        public:

            wobbly::Point const Position = wobbly::Point (100, 50);

            LockedObject () :
                object (Position)
            {
                object.Lock ();
            }

        protected:

            wobbly::Object object;
    };

    TEST_F (LockedObject, VelocityResetOnStep)
    {
        object.velocity = wobbly::Vector (1, 1);
        object.Step (10);

        EXPECT_THAT (object.velocity,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    TEST_F (LockedObject, VelocityAffectedAfterUnlocked)
    {
        wobbly::Vector initialVelocity (1, 1);
        object.velocity = initialVelocity;
        object.Unlock ();
        object.Step (1);

        EXPECT_THAT (object.velocity,
                     Not (GeometricallyEqual (initialVelocity)));
    }

    TEST_F (LockedObject, IsAnchor)
    {
        EXPECT_TRUE (object.IsAnchor ());
    }

    TEST (Object, FrictionVelocityAppliedToPoint)
    {
        std::function <double (int)> horizontalVelocityFunction =
            [](int timestep) -> double {
                wobbly::Object object (wobbly::Point (1.0, 1.0));
                object.velocity = wobbly::Vector (1, 0);
                object.Step (timestep);

                return object.velocity.get <0> ();
            };

        EXPECT_THAT (horizontalVelocityFunction,
                     SatisfiesModel <double> (ExponentialDecayTowards (0.0)));
    }

    class FrictionlessObject :
        public Test
    {
        protected:

            FrictionlessObject () :
                mOldFriction (wobbly::Object::Friction)
            {
                wobbly::Object::Friction = 0.0f;
            }

            ~FrictionlessObject ()
            {
                wobbly::Object::Friction = mOldFriction;
            }

        private:

            float mOldFriction;
    };

    TEST_F (FrictionlessObject, VelocityIncreasesLinearlyWithConstantForce)
    {
        std::function <double (int)> frictionlessHorizontalVelocityFunction =
            [](int timestep) -> double {
               wobbly::Object object (wobbly::Point (0.0, 0.0));

               object.force = 1.0f;
               object.Step (timestep);

               return object.velocity.get <0> ();
            };

        EXPECT_THAT (frictionlessHorizontalVelocityFunction,
                     SatisfiesModel <double> (Linear <double> ()));
    }

    TEST_F (FrictionlessObject, ObjectPositionChangesParabolically)
    {
        std::function <double (int)> frictionlessHorizontalPositionFunction =
            [](int timestep) -> double {
                wobbly::Object object (wobbly::Point (0.0, 0.0));

                object.force = 1.0f;
                object.Step (timestep);

                return bg::get <0> (object.position);
            };

        EXPECT_THAT (frictionlessHorizontalPositionFunction,
                     SatisfiesModel <double> (Parabolic <double> ()));
    }

    TEST (Object, ForceResetAfterStep)
    {
        wobbly::Object object;

        object.force = wobbly::Vector (1, 1);

        object.Step (1);

        EXPECT_THAT (object.force,
                     GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    constexpr double FirstPositionX = 50.0f;
    constexpr double FirstPositionY = 50.0f;

    constexpr double SecondPositionX = 100.0f;
    constexpr double SecondPositionY = 100.0f;

    constexpr double SpringConstant = 0.5f;

    class Springs :
        public ::testing::Test
    {
        public:

            Springs ():
                first (wobbly::Vector (FirstPositionX, FirstPositionY)),
                second (wobbly::Vector (SecondPositionX, SecondPositionY)),
                desiredDistance (SecondPositionX - FirstPositionX,
                                 SecondPositionY - FirstPositionY),
                spring (first,
                        second,
                        desiredDistance)
            {
            }

        protected:

            wobbly::Object first;
            wobbly::Object second;
            wobbly::Vector desiredDistance;
            wobbly::Spring spring;
    };

    TEST_F (Springs, NoForceAppliedWhenNoDeltaFromDesired)
    {
        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force, GeometricallyEqual (wobbly::Vector (0, 0)));
        EXPECT_THAT (second.force, GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    TEST_F (Springs, ForcesNotAppliedToAnchorObjects)
    {
        /* Move first object slightly, but make it an anchor too */
        bg::assign (first.position,
                    wobbly::Vector (FirstPositionX - 10.0f,
                                    FirstPositionY - 10.0f));
        first.Lock ();

        spring.applyForces (SpringConstant);

        EXPECT_THAT (first.force, GeometricallyEqual (wobbly::Vector (0, 0)));
    }

    wobbly::Vector
    ForceForSpring (wobbly::Object const &first,
                    wobbly::Object const &second,
                    wobbly::Vector const &desired)
    {
        wobbly::Vector expectedForce (bg::get <0> (second.position) -
                                      bg::get <0> (first.position),
                                      bg::get <1> (second.position) -
                                      bg::get <1> (first.position));
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
        wobbly::Vector expectedForce (ForceForSpring (first,
                                                      second,
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

        wobbly::Vector expectedForce (ForceForSpring (second,
                                                      first,
                                                      negativeDistance));

        spring.applyForces (SpringConstant);

        EXPECT_THAT (second.force, GeometricallyEqual (expectedForce));
    }

    TEST_F (Springs, ForceRelationshipIsLinearlyRelatedToDistance)
    {
        std::function <double (int)> forceByDistanceFunction =
            [this](int delta) -> double {
                bg::assign (first.force, wobbly::Vector (0, 0));
                bg::assign (second.force, wobbly::Vector (0, 0));
                bg::assign (first.position,
                            wobbly::Vector (FirstPositionX - delta * 0.5,
                                            FirstPositionY));
                bg::assign (second.position,
                            wobbly::Vector (SecondPositionX + delta * 0.5,
                                            SecondPositionY));

                spring.applyForces (SpringConstant);

                return first.force.get <0> ();
            };

        EXPECT_THAT (forceByDistanceFunction,
                     SatisfiesModel <double> (Linear <double> ()));
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

    constexpr float TextureWidth = 50.0f;
    constexpr float TextureHeight = 100.0f;

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
                point.template set <I> (std::ceil (point.template get <I> ()));
            }
    };

    template <typename C, std::size_t D, typename S>
    void PointCeiling (bg::model::point <C, D, S> &p)
    {
        bg::for_each_coordinate (p, PointCeilingOperation ());
    }

    TEST_F (SpringBezierModel, NoNormalizedDeformationOnMovementWithNoAnchor)
    {
        model.MoveModelTo (wobbly::Vector (1, 1));
        model.StepModel (1);

        auto center (wobbly::Point (0.5, 0.5));
        auto point (model.DeformTexcoords (center));

        /* Not quite accurate, but truncate the returned point
         * so that we can do a reliable comparison */
        PointCeiling (point);

        EXPECT_THAT (point,
                     GeometricallyEqual (wobbly::Point (TextureWidth / 2,
                                                        TextureHeight / 2)));
    }

    TEST_F (SpringBezierModel, DeformationOccurrsWhenModelMovedWithAnchor)
    {
        model.GrabAnchor (wobbly::Point (TextureWidth / 2, 0));
        model.MoveModelTo (wobbly::Vector (1, 1));
        model.StepModel (1);

        auto center (wobbly::Point (0.5, 0.5));
        auto point (model.DeformTexcoords (center));

        /* Not quite accurate, but truncate the returned point
         * so that we can do a reliable comparison */
        PointCeiling (point);

        auto textureCenter (wobbly::Point (TextureWidth / 2,
                                           TextureHeight / 2));

        EXPECT_THAT (point,
                     Not (GeometricallyEqual (textureCenter)));
    }

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
                auto const lookup (wobbly::Point (unitLookupValue, 0.0));
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
                     SatisfiesModel <double> (Linear <double> (),
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
                     SatisfiesModel <double> (Linear <double> (),
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
                     Not (SatisfiesModel <double> (Linear <double> (),
                                                   WithSamples (nSamples),
                                                   WithToleranace (10e-5))));
    }
}
