#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/component/statecontainer/MechanicalObject.h>
#include <sofa/core/objectmodel/BaseObject.h>
using sofa::core::objectmodel::BaseObject;
using sofa::core::objectmodel::ComponentState;

#include <sofa/linearalgebra/FullVector.h>
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

using sofa::helper::WriteAccessor ;
using sofa::core::ConstraintParams ;
using sofa::defaulttype::Vec3Types ;


#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::statecontainer::MechanicalObject ;

#include <SoftRobots.Inverse/component/constraint/SurfacePressureActuator.h>

namespace softrobotsinverse {

    template <typename _DataTypes>
    struct SurfacePressureActuatorTest : public BaseTest, softrobotsinverse::constraint::SurfacePressureActuator<_DataTypes>{

        typedef softrobotsinverse::constraint::SurfacePressureActuator<_DataTypes> ThisClass ;
        typedef _DataTypes DataTypes;
        typedef typename DataTypes::Deriv Deriv;
        typedef typename DataTypes::VecDeriv VecDeriv;
        typedef typename DataTypes::MatrixDeriv MatrixDeriv;
        typedef typename DataTypes::Coord Coord;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

        typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
        typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

        typedef sofa::core::topology::BaseMeshTopology::Quad Quad;


        void normalTests(){
            Simulation* simu;
            setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

            Node::SPtr node = simu->createNewGraph("root");
            typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
            typename ThisClass::SPtr thisobject = New<ThisClass >() ;
            node->addObject(thisobject) ;
            mecaobject->init() ;
            node->addObject(mecaobject) ;

            thisobject->setName("myname") ;

            EXPECT_TRUE(thisobject->findData("triangles") != nullptr) ;
            EXPECT_TRUE(thisobject->findData("quads") != nullptr) ;
            EXPECT_TRUE(thisobject->findData("maxPressure") != nullptr) ;
            EXPECT_TRUE(thisobject->findData("minPressure") != nullptr) ;

        }

        void stressTests(){
            Simulation* simu;
            setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

            Node::SPtr node = simu->createNewGraph("root");
            typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
            typename ThisClass::SPtr thisobject = New<ThisClass >() ;
            node->addObject(thisobject) ;
            mecaobject->init() ;
            node->addObject(mecaobject) ;

        }

    };

    using ::testing::Types;
    typedef Types<sofa::defaulttype::Vec3Types> DataTypes;

    TYPED_TEST_SUITE(SurfacePressureActuatorTest, DataTypes);


    TYPED_TEST(SurfacePressureActuatorTest, NormalBehavior) {
        this->normalTests() ;
    }

    TYPED_TEST(SurfacePressureActuatorTest, StressBehavior) {
        this->stressTests() ;
    }

}

