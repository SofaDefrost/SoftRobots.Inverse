
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/defaulttype/RigidTypes.h>
using sofa::defaulttype::Rigid3Types;
#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types;

#include <SoftRobots.Inverse/component/constraint/BarycentricCenterEffector.h>

namespace softrobotsinverse
{

using softrobotsinverse::constraint::BarycentricCenterEffector;

template <typename _DataTypes>
struct BarycentricCenterEffectorTest : public BaseTest, BarycentricCenterEffector<_DataTypes>
{

    using BarycentricCenterEffector<_DataTypes>::getTarget;
    using BarycentricCenterEffector<_DataTypes>::d_effectorGoal;
    using BarycentricCenterEffector<_DataTypes>::d_limitShiftToTarget;
    using BarycentricCenterEffector<_DataTypes>::d_maxShiftToTarget;

    typedef _DataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;

    void testTargetLimit()
    {
        d_limitShiftToTarget.setValue(true);
        d_maxShiftToTarget.setValue(2.);
        EXPECT_EQ(getTarget(10., 5.), 7.);
        EXPECT_EQ(getTarget(10., 8.), 10.);
        EXPECT_EQ(getTarget(10., 12.), 10.);
        EXPECT_EQ(getTarget(10., 13.), 11.);
        EXPECT_EQ(getTarget(-10., -5.), -7.);
        EXPECT_EQ(getTarget(-10., -8.), -10.);
        EXPECT_EQ(getTarget(-10., -12.), -10.);
        EXPECT_EQ(getTarget(-10., -13.), -11.);
    }
};

using ::testing::Types;

typedef Types<
Vec3Types,
Rigid3Types
> DataTypes;
TYPED_TEST_SUITE(BarycentricCenterEffectorTest, DataTypes);

TYPED_TEST(BarycentricCenterEffectorTest, TargetLimit) {
    ASSERT_NO_THROW( this->testTargetLimit() );
}

}

