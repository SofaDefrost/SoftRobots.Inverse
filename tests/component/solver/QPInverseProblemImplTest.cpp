#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;

#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>
using softrobotsinverse::solver::module::QPInverseProblemImpl ;

#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types;

using std::vector;


namespace softrobotsinverse::test
{

template <typename _DataTypes>
struct QPInverseProblemImplTest : public BaseTest, QPInverseProblemImpl
{
    typedef QPInverseProblemImpl ThisClass ;
    typedef _DataTypes DataTypes;

    void isInTest()
    {
        vector<int> list;
        list.push_back(1);
        list.push_back(3);

        EXPECT_TRUE(isIn(list,1));
        EXPECT_FALSE(isIn(list,2));
    }


    void isCyclingOnPivotTest()
    {
        m_currentSequence.clear();
        m_sequence.clear();

        m_currentSequence.push_back(1);
        EXPECT_TRUE(isCycling(1));

        m_currentSequence.clear();
        m_sequence.clear();

        m_currentSequence.push_back(1);
        m_currentSequence.push_back(2);

        EXPECT_TRUE(isCycling(2));
    }


    void isCyclingOnSequenceTest1()
    {
        m_sequence.clear();
        m_sequence.push_back(1);
        m_sequence.push_back(2);
        m_sequence.push_back(3);

        m_currentSequence.clear();
        m_currentSequence.push_back(1);
        m_currentSequence.push_back(2);

        EXPECT_TRUE(isCycling(3));
        EXPECT_FALSE(isCycling(4));
    }


    void isCyclingOnSequenceTest2()
    {
        m_previousSequence.clear();
        m_previousSequence.push_back(4);
        m_previousSequence.push_back(5);

        m_sequence.clear();
        m_sequence.push_back(1);
        m_sequence.push_back(2);

        m_currentSequence.clear();
        m_currentSequence.push_back(1);
        EXPECT_TRUE(isCycling(2));

        m_currentSequence.clear();
        m_currentSequence.push_back(4);
        EXPECT_TRUE(isCycling(5));
    }


    void isCyclingOnSequenceTest3()
    {
        m_currentSequence.clear();
        m_previousSequence.clear();
        m_sequence.clear();

        m_sequence.push_back(1);
        m_sequence.push_back(2);
        m_sequence.push_back(3);

        m_previousSequence.push_back(1);
        m_previousSequence.push_back(3);

        EXPECT_TRUE(isCycling(1));
        EXPECT_FALSE(isCycling(3));
    }


    void leaveCurrentSequenceUnchangedTest()
    {
        m_currentSequence.clear();
        m_sequence.clear();

        m_currentSequence.push_back(1);
        m_currentSequence.push_back(2);
        m_currentSequence.push_back(3);

        isCycling(1);

        EXPECT_TRUE(m_currentSequence.size()==3);
    }


    void leaveSequenceUnchangedTest()
    {
        m_currentSequence.clear();
        m_sequence.clear();

        m_currentSequence.push_back(1);
        m_currentSequence.push_back(2);
        m_currentSequence.push_back(3);

        isCycling(1);

        EXPECT_TRUE(m_sequence.size() == 0);
    }


    void updateLambdaTest()
    {
        m_qpSystem->dim = 5;
        std::vector<double> lambda = {1,2,3,4,5};
        updateLambda(lambda);
        EXPECT_EQ(lambda,m_qpSystem->lambda);
    }

};


using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(QPInverseProblemImplTest, DataTypes);

TYPED_TEST(QPInverseProblemImplTest, isInTest) {
    ASSERT_NO_THROW( this->isInTest() );
}

TYPED_TEST(QPInverseProblemImplTest, isCyclingOnPivotTest) {
    ASSERT_NO_THROW( this->isCyclingOnPivotTest() );
}

TYPED_TEST(QPInverseProblemImplTest, isCyclingOnSequenceTest) {
    ASSERT_NO_THROW( this->isCyclingOnSequenceTest1() );
    ASSERT_NO_THROW( this->isCyclingOnSequenceTest2() );
    ASSERT_NO_THROW( this->isCyclingOnSequenceTest3() );
}

TYPED_TEST(QPInverseProblemImplTest, leaveSequenceUnchangedTest) {
    ASSERT_NO_THROW( this->leaveSequenceUnchangedTest() );
}

TYPED_TEST(QPInverseProblemImplTest, leaveCurrentSequenceUnchangedTest) {
    ASSERT_NO_THROW( this->leaveCurrentSequenceUnchangedTest() );
}

TYPED_TEST(QPInverseProblemImplTest, updateLambdaTest) {
    ASSERT_NO_THROW( this->updateLambdaTest() );
}


} // namespace

