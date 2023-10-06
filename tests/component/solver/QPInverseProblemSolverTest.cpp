#include <string>
using std::string ;
#include <sofa/testing/BaseTest.h>
using sofa::testing::BaseTest ;
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/Locale.h>
#include <sofa/component/statecontainer/MechanicalObject.h>


using sofa::helper::WriteAccessor ;
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

#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
using softrobotsinverse::solver::QPInverseProblemSolver ;

using sofa::type::vector;
using std::fabs;
using std::stof;


namespace sofa
{

template <typename _DataTypes>
struct QPInverseProblemSolverTest : public BaseTest,
        QPInverseProblemSolver
{
    typedef QPInverseProblemSolver ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Real Real;


    simulation::Node::SPtr m_root;                 ///< Root of the scene graph, created by the constructor an re-used in the tests
    simulation::Simulation* m_simulation;          ///< created by the constructor an re-used in the tests


    void normalTests()
    {
        Simulation* simu;
        setSimulation(simu = new simulation::graph::DAGSimulation());

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;
        mecaobject->init() ;

        node->addObject(mecaobject) ;
        node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_NO_THROW( thisobject->init() ) ;

        return ;
    }


    void SetUp()
    {
        simulation::setSimulation(m_simulation = new simulation::graph::DAGSimulation());

        /// Load the scene
        string sceneName = "Finger.scn";

        string fileName  = string(SOFTROBOTS_TEST_DIR) + "/component/solver/scenes/" + sceneName;

        m_root = core::objectmodel::SPtr_dynamic_cast<simulation::Node>( simulation::getSimulation()->load(fileName.c_str()));

        /// Test if load has succeededls
        simulation::SceneLoaderXML scene;

        if(!m_root || !scene.loadSucceed)
            ADD_FAILURE() << "Error while loading the scene: " << sceneName << std::endl;
    }


    // Test the behavior of the algorithm
    void behaviorTests()
    {
        helper::system::TemporaryLocale locale(LC_NUMERIC, "C");

        SetUp();

        m_simulation->init(m_root.get());

        int nbTimeStep = 10;
        string deltaString;
        double tolerance = 1e-10;
        m_root->getObject("QPInverseProblemSolver")->findData("epsilon")->read("0.0");

        // Test inverse resolution (effector == target)

        // State1: Test normal behavior at init
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 7.5 7.5");

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        deltaString  = m_root->getChild("finger")->getChild("controlledPoints")->getObject("effector")->findData("delta")->getValueString();

        EXPECT_LE( fabs(stof(deltaString.c_str())), tolerance);


        // State2: Test normal behavior
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 10 7.5");

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        deltaString  = m_root->getChild("finger")->getChild("controlledPoints")->getObject("effector")->findData("delta")->getValueString();

        EXPECT_LE( fabs(stof(deltaString.c_str())), tolerance);

        // State3: Test normal behavior
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 15 7.5");

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        deltaString  = m_root->getChild("finger")->getChild("controlledPoints")->getObject("effector")->findData("delta")->getValueString();

        EXPECT_LE( fabs(stof(deltaString.c_str())), tolerance);
    }


    void regressionTests()
    {
        SetUp();

        m_simulation->init(m_root.get());
        string deltaString, lambdaString;

        int nbTimeStep = 10;

        // State1: Test lambda >= 0
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 -10 7.5");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("minForce")->read("0");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->reinit();

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        lambdaString  =  m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("force")->getValueString();
        EXPECT_GE( stof(lambdaString.c_str()), 0.);


        // State2: Test lambda >= -20
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 -10 7.5");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("minForce")->read("-20");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->reinit();

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        lambdaString =  m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("force")->getValueString();
        EXPECT_GE( stof(lambdaString.c_str()), -20.);


        // State3: Test delta <= 15
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 50 7.5");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("maxPositiveDisp")->read("15");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->reinit();

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        deltaString  =  m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("displacement")->getValueString();
        EXPECT_LE( stof(deltaString.c_str()), 15.);


        // State4: Test lambda <= 3000
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 7.5 7.5");

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 50 7.5");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("maxForce")->read("3000");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->reinit();

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        lambdaString =  m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("force")->getValueString();
        EXPECT_LE( stof(lambdaString.c_str()), 3000.);


        // State5: Test delta >= -1
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("-110 -10 7.5");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("maxNegativeDisp")->read("1");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("minForce")->read("0.");
        m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->reinit();

        for(int i=0; i<nbTimeStep; i++)
            m_simulation->animate(m_root.get());

        deltaString  = m_root->getChild("finger")->getChild("controlledPoints")->getObject("cable")->findData("displacement")->getValueString();
        EXPECT_GE( stof(deltaString.c_str()), -1.);
    }

};



using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(QPInverseProblemSolverTest, DataTypes);

TYPED_TEST(QPInverseProblemSolverTest, normalTests) {
    ASSERT_NO_THROW( this->normalTests() );
}

TYPED_TEST(QPInverseProblemSolverTest, regressionTests) {
    ASSERT_NO_THROW( this->regressionTests() );
}

TYPED_TEST(QPInverseProblemSolverTest, behaviorTests) {
    ASSERT_NO_THROW( this->behaviorTests() );
}


}

