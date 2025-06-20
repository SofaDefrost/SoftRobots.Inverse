#include <string>
#include <sofa/simpleapi/SimpleApi.h>
using std::string ;
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/helper/BackTrace.h>
#include <sofa/linearalgebra/FullVector.h>

#include <sofa/helper/accessor.h>
using sofa::helper::WriteAccessor ;
#include <sofa/defaulttype/VecTypes.h>
using sofa::defaulttype::Vec3Types ;

#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

#include <sofa/component/statecontainer/MechanicalObject.h>
using sofa::component::statecontainer::MechanicalObject ;

#include <sofa/component/solidmechanics/fem/elastic/TetrahedronFEMForceField.h>
using sofa::component::solidmechanics::fem::elastic::TetrahedronFEMForceField ;

#include <SoftRobots.Inverse/component/constraint/YoungModulusActuator.h>
using softrobotsinverse::constraint::YoungModulusActuator ;


namespace softrobotsinverse
{

template <typename _DataTypes>
struct YoungModulusActuatorTest : public BaseTest,
        YoungModulusActuator<_DataTypes>
{
    typedef _DataTypes DataTypes;
    typedef YoungModulusActuator<_DataTypes> ThisClass ;

    void simpleScene(){
        sofa::simpleapi::importPlugin("Sofa.Component.SolidMechanics.FEM.Elastic");
        sofa::simpleapi::importPlugin("SoftRobots");
        sofa::simpleapi::importPlugin("SoftRobots.Inverse");
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <RequiredPlugin name='Sofa.Component.StateContainer'/>  "
                "   <MechanicalObject/>              "
                "   <TetrahedronFEMForceField/>     "
                "   <YoungModulusActuator/>       "
                "</Node>                             " ;
        EXPECT_NO_THROW(SceneLoaderXML::loadFromMemory ( "test1", scene.c_str())) ;
    }

    void normalBehavior(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");

        typename TetrahedronFEMForceField<DataTypes>::SPtr forcefield = New<TetrahedronFEMForceField<DataTypes> >() ;
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass>() ;

        node->addObject(mecaobject) ;
        node->addObject(forcefield) ;
        node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_TRUE( thisobject->findData("minYoung") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxYoungVariationRatio") != nullptr ) ;

        EXPECT_NO_THROW( thisobject->init() ) ;
        EXPECT_NO_THROW( thisobject->bwdInit() ) ;
        EXPECT_NO_THROW( thisobject->reinit() ) ;
        EXPECT_NO_THROW( thisobject->reset() ) ;

        thisobject->findData("minYoung")->read("0");
        EXPECT_NO_THROW(thisobject->reinit()) ;

        thisobject->findData("maxYoungVariationRatio")->read("1");
        EXPECT_NO_THROW(thisobject->reinit()) ;
    }

};

using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(YoungModulusActuatorTest, DataTypes);

TYPED_TEST(YoungModulusActuatorTest, SimpleScene) {
    this->simpleScene() ;
}

TYPED_TEST(YoungModulusActuatorTest, NormalBehavior) {
    this->normalBehavior() ;
}

}

