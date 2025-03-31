#include <string>
using std::string ;

#include <gtest/gtest.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/Locale.h>

#include <SofaPython3/PythonEnvironment.h>
#include <SofaPython3/SceneLoaderPY3.h>
using sofapython3::SceneLoaderPY3 ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;

#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
using softrobotsinverse::solver::QPInverseProblemSolver;

using sofa::type::vector;
using sofa::helper::rabs;
using std::stof;
using std::istringstream;


namespace sofa
{


struct QPInverseProblemSolverWithContactTest : public ::testing::Test,  QPInverseProblemSolver
{
protected:
    typedef QPInverseProblemSolver ThisClass ;

    simulation::Node::SPtr m_root;                 ///< Root of the scene graph, created by the constructor an re-used in the tests

    void SetUp()
    {
        sofapython3::PythonEnvironment::Init();

        // sofa init
        static const string scenePath  = string(SOFTROBOTSINVERSE_TEST_DIR) + std::string("/component/solver/scenes/CubeOnFloor.pyscn");
        m_root = core::objectmodel::SPtr_dynamic_cast<simulation::Node>( sofa::simulation::node::load(scenePath.c_str()));

        sofa::simulation::node::initRoot(m_root.get());

        if(!m_root)
            ADD_FAILURE() << "Error while loading the scene: " << scenePath << std::endl;
    }


    void testFallingCube()
    {
        helper::system::TemporaryLocale locale(LC_NUMERIC, "C");
        sofa::simulation::node::reset(m_root.get());

        int nbTimeStep = 10;
        m_root->getObject("QPInverseProblemSolver")->findData("epsilon")->read("0.0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.0");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.0");


        //The floor should stop the cube from falling (gravity allong -y direction)
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("10 0 0");

        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());

        string posString = m_root->getChild("model")->getChild("effector")->getObject("effectorPoint")->findData("position")->getValueString();
        float x, y, z;
        getValueFromString(posString, x, y, z);

        EXPECT_GE(y,-0.1);
    }

    void testFallingCubeWithFriction()
    {
        helper::system::TemporaryLocale locale(LC_NUMERIC, "C");
        sofa::simulation::node::reset(m_root.get());

        int nbTimeStep = 10;
        m_root->getObject("QPInverseProblemSolver")->findData("epsilon")->read("0.0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.6");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.6");

        //The floor should stop the cube from falling (gravity allong -y direction)
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("10 0 0");

        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());

        string posString = m_root->getChild("model")->getChild("effector")->getObject("effectorPoint")->findData("position")->getValueString();
        float x, y, z;
        getValueFromString(posString, x, y, z);

        EXPECT_GE(y,-0.1);
    }

    void testForceWithFriction()
    {
        helper::system::TemporaryLocale locale(LC_NUMERIC, "C");

        int nbTimeStep = 1;
        m_root->getObject("QPInverseProblemSolver")->findData("allowSliding")->read("1");

        //The force required to move the cube should be higher with friction
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("11 0 0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.0");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.0");


        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());
        string force1 = m_root->getChild("model")->getObject("act")->findData("force")->getValueString();

        sofa::simulation::node::reset(m_root.get());
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("11 0 0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.5");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.5");


        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());
        string force2 = m_root->getChild("model")->getObject("act")->findData("force")->getValueString();

        sofa::simulation::node::reset(m_root.get());
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("11 0 0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=1.");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("1.");


        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());
        string force3 = m_root->getChild("model")->getObject("act")->findData("force")->getValueString();

        EXPECT_LE(stof(force1),stof(force2));
        EXPECT_LE(stof(force2),stof(force3));
    }

    void testAllowSliding()
    {
        helper::system::TemporaryLocale locale(LC_NUMERIC, "C");

        int nbTimeStep = 25;

        sofa::simulation::node::reset(m_root.get());
        m_root->getObject("QPInverseProblemSolver")->findData("allowSliding")->read("1"); // Enable sliding
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.8");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.8");


        double position=0.;
        for(int i=0; i<nbTimeStep; i++)
        {
            position += 2./nbTimeStep*i;
            m_root->getChild("goal")->getObject("goalMO")->findData("position")->read(std::to_string(position)+" 0 0");
            sofa::simulation::node::animate(m_root.get());
        }

        string deltaString = m_root->getChild("model")->getChild("effector")->getObject("PositionEffector")->findData("delta")->getValueString();
        float x, y, z;
        getValueFromString(deltaString, x, y, z);
        double norm1 = sqrt(x*x+y*y+z*z);

        sofa::simulation::node::reset(m_root.get());
        m_root->getObject("QPInverseProblemSolver")->findData("allowSliding")->read("0"); // Disable sliding
        m_root->getChild("goal")->getObject("goalMO")->findData("position")->read("12 0 0");
        m_root->getObject("CollisionResponse")->findData("responseParams")->read("mu=0.8");
        m_root->getObject("QPInverseProblemSolver")->findData("responseFriction")->read("0.8");


        for(int i=0; i<nbTimeStep; i++)
            sofa::simulation::node::animate(m_root.get());

        deltaString = m_root->getChild("model")->getChild("effector")->getObject("PositionEffector")->findData("delta")->getValueString();
        getValueFromString(deltaString, x, y, z);
        double norm2 = sqrt(x*x+y*y+z*z);

        EXPECT_LE(helper::rabs(norm1),1e-1); // With sliding allowed, effector should meet target
        EXPECT_LE(helper::rabs(norm2-2),1e-1); // With sliding disabled, effector should stay fix
    }

    void getValueFromString(const string positionStr, float& x, float& y, float& z)
    {
        istringstream iss(positionStr);
        string xStr, yStr, zStr;
        iss >> xStr;
        iss >> yStr;
        iss >> zStr;

        x = stof(xStr);
        y = stof(yStr);
        z = stof(zStr);
    }

};


TEST_F(QPInverseProblemSolverWithContactTest, testFallingCube)
{
    testFallingCube();
}

TEST_F(QPInverseProblemSolverWithContactTest, testFallingCubeWithFriction)
{
    testFallingCubeWithFriction();
}

TEST_F(QPInverseProblemSolverWithContactTest, testForceWithFriction)
{
    testForceWithFriction();
}

//TEST_F(QPInverseProblemSolverWithContactTest, testAllowSliding)
//{
//    testAllowSliding();
//}

}

