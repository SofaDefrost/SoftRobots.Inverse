#include <string>
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/simulation/Node.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

#include <sofa/linearalgebra/FullVector.h>
#include <sofa/simulation/common/SceneLoaderXML.h>
#include <sofa/simulation/graph/DAGSimulation.h>
#include <SoftRobots.Inverse/component/constraint/SlidingActuator.h>

namespace softrobotsinverse
{

using std::string ;
using sofa::component::statecontainer::MechanicalObject ;
using sofa::core::objectmodel::BaseObject;
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;
using sofa::helper::WriteAccessor ;
using sofa::defaulttype::Vec3Types ;
using sofa::simulation::SceneLoaderXML ;
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using softrobotsinverse::constraint::SlidingActuator ;
using sofa::core::objectmodel::ComponentState;


template <typename _DataTypes>
struct SlidingActuatorTest : public BaseTest, SlidingActuator<_DataTypes>
{
    typedef SlidingActuator<_DataTypes> ThisClass ;
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


    //////////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using SlidingActuator<_DataTypes>::d_indices ;
    using SlidingActuator<_DataTypes>::d_direction ;
    using SlidingActuator<_DataTypes>::m_constraintIndex ;
    using SlidingActuator<_DataTypes>::m_state ;
    using BaseObject::d_componentState ;
    //////////////////////////////////////////////////////////////////



    void setUp(Data<VecCoord> &x)
    {
        m_constraintIndex.setValue(0);

        d_direction.setValue(Deriv(1.,0.,0.));
        d_componentState = ComponentState::Valid;

        WriteAccessor<Data<VecCoord> > positions = x;
        positions.clear();
        positions.push_back(Coord(0.,0.,0.));

        WriteAccessor<Data<sofa::type::vector<unsigned int> > > indices = d_indices;
        indices.clear();
        indices.push_back(0);
    }


    void normalTests()
    {
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
        EXPECT_TRUE( thisobject->findData("direction") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxPositiveDisp") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxNegativeDisp") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("minForce") != nullptr ) ;

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

TYPED_TEST_SUITE(SlidingActuatorTest, DataTypes);


TYPED_TEST(SlidingActuatorTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

}

