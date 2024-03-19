#include <string>
using std::string ;
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/core/objectmodel/BaseObject.h>
using sofa::core::objectmodel::BaseObject;
#include <sofa/helper/BackTrace.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

#include <sofa/linearalgebra/FullVector.h>
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

using sofa::helper::WriteAccessor ;
using sofa::defaulttype::Vec3Types ;

#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;

#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::statecontainer::MechanicalObject ;

#include <SoftRobots.Inverse/component/constraint/CableActuator.h>
using softrobotsinverse::constraint::CableActuator ;

using sofa::core::objectmodel::ComponentState;

namespace softrobotsinverse
{

template <typename _DataTypes>
struct CableActuatorTest : public BaseTest,
        CableActuator<_DataTypes>
{
    typedef CableActuator<_DataTypes> ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

    typedef BaseMeshTopology::Quad Quad;

    ////////////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using CableActuator<_DataTypes>::m_hasSlidingPoint ;
    using CableActuator<_DataTypes>::m_hasDeltaMax ;
    using CableActuator<_DataTypes>::m_hasDeltaMin ;
    using CableActuator<_DataTypes>::m_hasLambdaMax ;
    using CableActuator<_DataTypes>::m_hasLambdaMin ;
    using CableActuator<_DataTypes>::d_pullPoint ;
    using CableActuator<_DataTypes>::d_indices ;
    using CableActuator<_DataTypes>::d_constrainAtTime ;
    using CableActuator<_DataTypes>::d_maxDispVariation ;
    using CableActuator<_DataTypes>::d_minForce ;
    using CableActuator<_DataTypes>::d_maxForce ;
    /////////////////////////////////////////////////////////////////////


    void testUpdateLimits()
    {
        // Actual time is 0.0

        //////////// Tests for constraint on delta
        d_maxDispVariation.setValue(1.0);

        // atTime > time
        d_constrainAtTime.setValue(3.0);
        this->updateLimit();

        EXPECT_EQ(m_hasDeltaMax, false) << "constrainAtTime 3.0";
        EXPECT_EQ(m_hasDeltaMin, false) << "constrainAtTime 3.0";

        // atTime == time
        d_constrainAtTime.setValue(0.0);
        this->updateLimit();

        EXPECT_EQ(m_hasDeltaMax, true) << "constrainAtTime 0.0";
        EXPECT_EQ(m_hasDeltaMin, true) << "constrainAtTime 0.0";

        //////////// Tests for constraint on lambda
        d_minForce.setValue(0.0);
        d_maxForce.setValue(1.0);

        // atTime > time
        d_constrainAtTime.setValue(3.0);
        this->updateLimit();

        EXPECT_EQ(m_hasLambdaMax, false) << "constrainAtTime 3.0";
        EXPECT_EQ(m_hasLambdaMin, false) << "constrainAtTime 3.0";

        // atTime == time
        d_constrainAtTime.setValue(0.0);
        this->updateLimit();

        EXPECT_EQ(m_hasLambdaMax, true) << "constrainAtTime 0.0";
        EXPECT_EQ(m_hasLambdaMin, true) << "constrainAtTime 0.0";
    }


    void normalTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;
        mecaobject->init() ;

        node->addObject(mecaobject) ;
        node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_TRUE( thisobject->findData("indices") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxPositiveDisp") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxNegativeDisp") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("minForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("constrainAtTime") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("pullPoint") != nullptr ) ;

        EXPECT_NO_THROW( thisobject->init() ) ;
        EXPECT_NO_THROW( thisobject->bwdInit() ) ;
        EXPECT_NO_THROW( thisobject->reinit() ) ;
        EXPECT_NO_THROW( thisobject->reset() ) ;

        thisobject->findData("indices")->read("0") ;
        return ;
    }

};

using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(CableActuatorTest, DataTypes);


TYPED_TEST(CableActuatorTest, UpdateLimits) {
    ASSERT_NO_THROW(this->testUpdateLimits()) ;
}

TYPED_TEST(CableActuatorTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}


}
