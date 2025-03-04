#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <SoftRobots.Inverse/component/constraint/PositionEffector.h>
#include <sofa/linearalgebra/FullVector.h>
using sofa::linearalgebra::FullVector;
#include <sofa/component/statecontainer/MechanicalObject.h>
using softrobotsinverse::constraint::PositionEffector;
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

using sofa::helper::WriteAccessor ;
using sofa::core::ConstraintParams ;
using sofa::defaulttype::Vec3Types ;
using sofa::defaulttype::Rigid3Types ;
using sofa::type::Vec ;

#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::graph::DAGSimulation;
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::statecontainer::MechanicalObject ;
using sofa::type::vector;
using sofa::core::objectmodel::ComponentState;

namespace softrobotsinverse::test
{

template <typename _DataTypes>
struct PositionEffectorTest : public BaseTest, PositionEffector<_DataTypes>
{

    typedef _DataTypes DataTypes;
    typedef PositionEffector<_DataTypes> ThisClass ;

    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

    typedef sofa::core::topology::BaseMeshTopology::Quad Quad;

    void doSetUp() override
    {
        this->d_componentState = ComponentState::Valid;
    }


    void normalBehaviorTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaObject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisObject = New<ThisClass >() ;
        mecaObject->init() ;

        node->addObject(mecaObject) ;
        node->addObject(thisObject) ;

        thisObject->setName("myname") ;
        EXPECT_TRUE(thisObject->getName() == "myname") ;

        EXPECT_TRUE( thisObject->findData("indices") != nullptr ) ;
        EXPECT_TRUE( thisObject->findData("effectorGoal") != nullptr ) ;
        EXPECT_TRUE( thisObject->findData("directions") != nullptr ) ;
        EXPECT_TRUE( thisObject->findData("useDirections") != nullptr ) ;

        EXPECT_NO_THROW( thisObject->init() ) ;
        EXPECT_NO_THROW( thisObject->bwdInit() ) ;
        EXPECT_NO_THROW( thisObject->reinit() ) ;
        EXPECT_NO_THROW( thisObject->reset() ) ;

        thisObject->findData("indices")->read("0");
        EXPECT_NO_THROW(thisObject->reinit()) ;
        thisObject->findData("indices")->read("100") ;
        thisObject->findData("indices")->read("-100") ;
        thisObject->findData("indices")->read("0 1 -1") ;
        EXPECT_NO_THROW(thisObject->reinit()) ;

        thisObject->findData("indices")->read("0") ;
        return ;
    }


    void defaultValuesTest(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaObject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisObject = New<ThisClass >() ;
        mecaObject->init() ;

        node->addObject(mecaObject) ;
        node->addObject(thisObject) ;
        thisObject->init();

        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"1 1 1");
        EXPECT_EQ(thisObject->findData("directions")->getValueString(),"1 0 0 0 1 0 0 0 1");
    }


    void initTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaObject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisObject = New<ThisClass >() ;
        mecaObject->init() ;

        node->addObject(mecaObject) ;
        node->addObject(thisObject) ;

        // Should set default value
        thisObject->findData("useDirections")->read("0 0 0");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"1 1 1");

        // Should remain the same
        thisObject->findData("useDirections")->read("0 0 1");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"0 0 1");

        // Should normalize
        thisObject->findData("directions")->read("3 0 0   0 0 0   0 1 0");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("directions")->getValueString(),"1 0 0 0 0 0 0 1 0");
    }
};


    template <>
    void PositionEffectorTest<Rigid3Types>::defaultValuesTest()
    {
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaObject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisObject = New<ThisClass >() ;
        mecaObject->init() ;

        node->addObject(mecaObject) ;
        node->addObject(thisObject) ;
        thisObject->init();

        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"1 1 1 1 1 1");
        EXPECT_EQ(thisObject->findData("directions")->getValueString(),"1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 1");
    }


    template <>
    void PositionEffectorTest<Rigid3Types>::initTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaObject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisObject = New<ThisClass >() ;
        mecaObject->init() ;

        node->addObject(mecaObject) ;
        node->addObject(thisObject) ;

        // Should set default value
        thisObject->findData("useDirections")->read("0 0 0 0 0 0");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"1 1 1 1 1 1");

        // Should remain the same
        thisObject->findData("useDirections")->read("0 0 0 0 0 1");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("useDirections")->getValueString(),"0 0 0 0 0 1");

        // Should normalize
        thisObject->findData("directions")->read("3 0 0 0 0 0   0 4 0 0 0 0   0 0 6 0 0 0    0 0 0 0 0 0    1 0 0 0 0 0   0 0 0 0 0 1");
        thisObject->init();
        EXPECT_EQ(thisObject->findData("directions")->getValueString(),"1 0 0 0 0 0 0 1 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 1");
    }


    using ::testing::Types;

    typedef Types<
    Vec3Types,
    Rigid3Types
    > DataTypes;
    TYPED_TEST_SUITE(PositionEffectorTest, DataTypes);

    TYPED_TEST(PositionEffectorTest, NormalBehavior) {
        ASSERT_NO_THROW(this->normalBehaviorTests()) ;
    }

    // Default values should be the same for each template
    TYPED_TEST(PositionEffectorTest, CheckDefaultValue) {
        ASSERT_NO_THROW(this->defaultValuesTest()) ;
    }

    TYPED_TEST(PositionEffectorTest, InitTests) {
        ASSERT_NO_THROW(this->initTests()) ;
    }

}

