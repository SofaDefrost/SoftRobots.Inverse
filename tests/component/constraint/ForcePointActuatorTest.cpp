#include <string>
using std::string ;
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
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

#include <SoftRobots.Inverse/component/constraint/ForcePointActuator.h>
using softrobotsinverse::constraint::ForcePointActuator ;

using sofa::type::vector;


namespace softrobotsinverse
{

template <typename _DataTypes>
struct ForcePointActuatorTest : public BaseTest, ForcePointActuator<_DataTypes>
{
    typedef ForcePointActuator<_DataTypes> ThisClass ;
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
    using ForcePointActuator<_DataTypes>::d_indices ;
    using ForcePointActuator<_DataTypes>::d_maxForce ;
    using ForcePointActuator<_DataTypes>::d_minForce ;
    using ForcePointActuator<_DataTypes>::d_maxForceVariation ;
    using ForcePointActuator<_DataTypes>::d_displacement ;
    using ForcePointActuator<_DataTypes>::d_direction ;
    using ForcePointActuator<_DataTypes>::d_showForce ;
    using ForcePointActuator<_DataTypes>::d_visuScale ;
    using ForcePointActuator<_DataTypes>::m_lambdaMax ;
    using ForcePointActuator<_DataTypes>::m_lambdaMin ;
    /////////////////////////////////////////////////////////////////////


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
        EXPECT_TRUE( thisobject->findData("maxForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("minForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxForceVariation") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("force") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("displacement") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("direction") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("showForce") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("visuScale") != nullptr ) ;

        EXPECT_NO_THROW( thisobject->init() ) ;
        EXPECT_NO_THROW( thisobject->bwdInit() ) ;
        EXPECT_NO_THROW( thisobject->reinit() ) ;
        EXPECT_NO_THROW( thisobject->reset() ) ;

        return ;
    }


    void limitsTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;
        mecaobject->init() ;

        node->addObject(mecaobject) ;
        node->addObject(thisobject) ;

        thisobject->findData("direction")->read("1 1 1");
        thisobject->findData("maxForce")->read("10");
        thisobject->findData("minForce")->read("-10");
        thisobject->findData("maxForceVariation")->read("1");

        thisobject->init();
        thisobject->bwdInit();
        thisobject->reinit();
        thisobject->reset();

        EXPECT_TRUE(thisobject->findData("maxForce")->getValueString() == "10") ;
        EXPECT_TRUE(thisobject->findData("minForce")->getValueString() == "-10") ;
        EXPECT_TRUE(thisobject->findData("maxForceVariation")->getValueString() == "1") ;

        EXPECT_TRUE(thisobject->getLambdaMax(0) == 1.);
        EXPECT_TRUE(thisobject->getLambdaMin(0) == -1.);

        vector<double> lambda;
        vector<double> delta;

        lambda.resize(1, 1.);
        delta.resize(1, 0.);
        thisobject->storeResults(lambda, delta);

        EXPECT_TRUE(thisobject->getLambdaMax(0) == 2.);
        EXPECT_TRUE(thisobject->getLambdaMin(0) == 0.);

        lambda[0]= -1.;
        thisobject->storeResults(lambda, delta);

        EXPECT_TRUE(thisobject->getLambdaMax(0) == 0.);
        EXPECT_TRUE(thisobject->getLambdaMin(0) == -2.);

        lambda[0]= 10.;
        thisobject->storeResults(lambda, delta);

        EXPECT_TRUE(thisobject->getLambdaMax(0) == 10.);
        EXPECT_TRUE(thisobject->getLambdaMin(0) == 9.);

        lambda[0]= -10.;
        thisobject->storeResults(lambda, delta);

        EXPECT_TRUE(thisobject->getLambdaMax(0) == -9.);
        EXPECT_TRUE(thisobject->getLambdaMin(0) == -10.);

        return ;
    }


    bool buildMatrixTests(){
        auto simu = sofa::simulation::getSimulation();

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;

        node->addObject(mecaobject) ;
        mecaobject->findData("position")->read("0. 0. 0.   0. 0. 0.   0. 0. 0.   0. 0. 0.");
        mecaobject->init();
        node->addObject(thisobject) ;
        thisobject->findData("direction")->read("1. 0. 0.");
        thisobject->findData("indices")->read("0 2");
        thisobject->init();

        sofa::core::ConstraintParams* cparams = NULL;
        sofa::core::objectmodel::Data<MatrixDeriv> columns;
        unsigned int columnsIndex;
        sofa::core::objectmodel::Data<VecCoord> x;

        columnsIndex = 0;

        MatrixDeriv& column = *columns.beginEdit();
        MatrixDerivRowIterator rowIterator = column.writeLine(columnsIndex);

        rowIterator.setCol(0, Deriv(0,0,0));
        rowIterator.setCol(1, Deriv(0,0,0));
        rowIterator.setCol(2, Deriv(0,0,0));
        rowIterator.setCol(3, Deriv(0,0,0));
        columns.endEdit();

        thisobject->buildConstraintMatrix(cparams, columns, columnsIndex, x);

        MatrixDeriv columnExpected;
        MatrixDerivRowIterator rowIteratorExpected = columnExpected.writeLine(0);

        rowIteratorExpected.setCol(0, Deriv(1,0,0));
        rowIteratorExpected.setCol(1, Deriv(0,0,0));
        rowIteratorExpected.setCol(2, Deriv(1,0,0));
        rowIteratorExpected.setCol(3, Deriv(0,0,0));

        MatrixDerivRowConstIterator rowIt2 = columnExpected.readLine(0);
        MatrixDerivColConstIterator rowEnd2 = rowIt2.end();

        MatrixDerivRowConstIterator rowIt1 = column.readLine(0);
        MatrixDerivColConstIterator rowEnd1 = rowIt1.end();

        for (MatrixDerivColConstIterator colIt1 = rowIt1.begin(), colIt2 = rowIt2.begin(); colIt1 != rowEnd1 && colIt2 != rowEnd2; ++colIt1, colIt2++)
        {
            if(colIt1.val() != colIt2.val())
                return false;
        }

        return true;
    }

};

using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(ForcePointActuatorTest, DataTypes);

TYPED_TEST(ForcePointActuatorTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(ForcePointActuatorTest, LimitsTests) {
    ASSERT_NO_THROW(this->limitsTests()) ;
}

TYPED_TEST(ForcePointActuatorTest, BuildMatrixTests) {
    EXPECT_TRUE(this->buildMatrixTests()) ;
}


}
