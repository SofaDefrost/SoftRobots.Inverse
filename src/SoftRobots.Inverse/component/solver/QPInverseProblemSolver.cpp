/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
*                       Plugin SoftRobots.Inverse                             *
*                                                                             *
* This plugin is distributed under the GNU AGPL v3 (Affero General            *
* Public License) license.                                                    *
*                                                                             *
* Authors: Christian Duriez, Eulalie Coevoet, Yinoussa Adagolodjo             *
*                                                                             *
* (c) 2023 INRIA                                                              *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
******************************************************************************/

#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <sofa/component/constraint/lagrangian/solver/visitors/ConstraintStoreLambdaVisitor.h>

#include <SoftRobots.Inverse/component/config.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/core/visual/VisualParams.h>
#include <sofa/core/behavior/MultiVec.h>
#include <sofa/simulation/BehaviorUpdatePositionVisitor.h>
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/simulation/SolveVisitor.h>
#include <sofa/simulation/VectorOperations.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalResetConstraintVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalVOpVisitor.h>
#include <sofa/simulation/mechanicalvisitor/MechanicalProjectJacobianMatrixVisitor.h>
#include <sofa/simulation/DefaultTaskScheduler.h>
#include <sofa/simulation/MainTaskSchedulerFactory.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/ScopedAdvancedTimer.h>
#include <sofa/helper/map.h>
#include <sofa/helper/system/thread/CTime.h>

#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>
#include <SoftRobots.Inverse/component/solver/modules/QPMechanicalSetConstraint.h>
#include <SoftRobots.Inverse/component/solver/modules/QPMechanicalAccumulateConstraint.h>

#ifdef SOFTROBOTSINVERSE_ENABLE_QPOASES
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemQPOases.h>
#endif

#ifdef SOFTROBOTSINVERSE_ENABLE_PROXQP
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemProxQP.h>
#endif

using sofa::simulation::mechanicalvisitor::MechanicalProjectJacobianMatrixVisitor;
using sofa::simulation::mechanicalvisitor::MechanicalResetConstraintVisitor;
using sofa::component::constraint::lagrangian::solver::ConstraintStoreLambdaVisitor ;
using sofa::simulation::common::VectorOperations;
using sofa::simulation::mechanicalvisitor::MechanicalVOpVisitor;
using sofa::simulation::Node ;

using sofa::core::behavior::BaseConstraintCorrection ;
using sofa::core::objectmodel::BaseContext ;

using sofa::helper::system::thread::CTime ;
using sofa::type::vector;
using sofa::helper::WriteAccessor;
using sofa::helper::AdvancedTimer ;

using sofa::core::ConstraintParams ;
using sofa::core::MechanicalParams ;
using sofa::core::MultiVecDerivId ;
using sofa::core::MultiVecCoordId ;
using sofa::core::MatrixDerivId ;
using sofa::core::ConstMultiVecDerivId ;
using sofa::core::behavior::MultiVecDeriv ;
using sofa::core::RegisterObject ;
using sofa::core::ExecParams ;

using std::pair ;
using std::string;
using sofa::type::Vec;
using std::map;


namespace softrobotsinverse::solver
{

namespace
{
template< typename TMultiVecId >
void clearMultiVecId(BaseContext* ctx, const ConstraintParams* cParams, const TMultiVecId& vid)
{
    MechanicalVOpVisitor clearVisitor(cParams, vid, ConstMultiVecDerivId::null(), ConstMultiVecDerivId::null(), 1.0);
    clearVisitor.setMapped(true);
    ctx->executeVisitor(&clearVisitor);
}
}

QPInverseProblemSolver::QPInverseProblemSolver()
    : d_displayTime(initData(&d_displayTime, false, "displayTime",
                             "Display time for each important step of QPInverseProblemSolver."))

    , d_multithreading(initData(&d_multithreading, false, "multithreading", "Build compliances concurrently"))

    , d_reverseAccumulateOrder(initData(&d_reverseAccumulateOrder, false, "reverseAccumulateOrder",
                                        "True to accumulate constraints from nodes in reversed order \n"
                                        "(can be necessary when using multi-mappings or interaction constraints \n"
                                        "not following the node hierarchy)"))

    , d_countdownFilterStartPerturb(initData(&d_countdownFilterStartPerturb, 1,"countdown",
                                             "Number of initial transient state steps omitted for the computations \n"
                                             "of the infinity norm variation of the QP matrix"))

    , d_saveMatrices(initData(&d_saveMatrices, false, "saveMatrices",
                              "If true, saves problem matrices in a file."))

    , d_maxIterations(initData(&d_maxIterations, 250, "maxIterations", "Maximum iterations for LCP solver"))

    , d_tolerance(initData(&d_tolerance, 1e-10, "tolerance", "Tolerance for LCP solver"))

    , d_responseFriction(initData(&d_responseFriction, 0., "responseFriction", "Response friction for contact resolution"))

    , d_qpSolver(initData(&d_qpSolver, "qpSolver", "QP solver implementation to be used"))

    , d_epsilon(initData(&d_epsilon, 1e-3, "epsilon",
                         "An energy term is added in the minimization process. \n"
                         "Epsilon has to be chosen sufficiently small so that the deformation \n"
                         "energy does not disrupt the quality of the effector positioning. "
                         "Default value 1e-3."))

    , d_actuatorsOnly(initData(&d_actuatorsOnly, false, "actuatorsOnly",
                         "An energy term is added in the minimization process. \n"
                         "If true, only for actuators."
                         "Default value false."))

    , d_allowSliding(initData(&d_allowSliding,false,"allowSliding",
                              "In case of friction, this option enable/disable sliding contact."))

    , d_graph(initData(&d_graph,"info","") )

    , d_minContactForces(initData(&d_minContactForces, "minContactForces",
                                  "If set, will contraints the sum of contact forces \n"
                                  "to be greater or equal to the given value (grasping option).") )

    , d_maxContactForces(initData(&d_maxContactForces, "maxContactForces",
                                  "If set, will contraints the sum of contact forces \n"
                                  "to be lesser or equal to the given value.") )

    , d_objective(initData(&d_objective, 250.0, "objective", "Erreur between the target and the end effector "))

    , m_lastCP(NULL)
    , m_CP1(nullptr)
    , m_CP2(nullptr)
    , m_CP3(nullptr)
{
    sofa::helper::OptionsGroup qpSolvers{"qpOASES" , "proxQP"};
#if defined SOFTROBOTSINVERSE_ENABLE_PROXQP && !defined SOFTROBOTSINVERSE_ENABLE_QPOASES
    qpSolvers.setSelectedItem(QPSolverImpl::PROXQP);
#else
    qpSolvers.setSelectedItem(QPSolverImpl::QPOASES);
#endif

    d_qpSolver.setValue(qpSolvers);

    d_graph.setWidget("graph");
    createProblems();

    m_qpSolverCB.addInput(&d_qpSolver);
    m_qpSolverCB.addCallback([this]()
    {
        deleteProblems();
        createProblems();
    });
}

void QPInverseProblemSolver::createProblems()
{
    switch(d_qpSolver.getValue().getSelectedId())
    {
#ifdef SOFTROBOTSINVERSE_ENABLE_PROXQP
    case QPSolverImpl::PROXQP :
        msg_info() << "Using proxQP solver";
        m_CP1 = new module::QPInverseProblemProxQP();
        m_CP2 = new module::QPInverseProblemProxQP();
        m_CP3 = new module::QPInverseProblemProxQP();
        break;
#endif
#ifdef SOFTROBOTSINVERSE_ENABLE_QPOASES
    case QPSolverImpl::QPOASES :
        msg_info() << "Using qpOASES solver";
        m_CP1 = new module::QPInverseProblemQPOases();
        m_CP2 = new module::QPInverseProblemQPOases();
        m_CP3 = new module::QPInverseProblemQPOases();
        break;
#endif
    default :
        msg_error() << "Unkown specified solver: " << d_qpSolver.getValue();
        sofa::core::objectmodel::BaseObject::d_componentState.setValue(sofa::core::objectmodel::ComponentState::Invalid);
        break;
    }


    m_currentCP = m_CP1;
}

void QPInverseProblemSolver::deleteProblems()
{
    if(m_CP1)
    {
      delete m_CP1;
      m_CP1 = nullptr;
    }
     if(m_CP2)
    {
      delete m_CP2;
      m_CP2 = nullptr;
    }
     if(m_CP3)
    {
      delete m_CP3;
      m_CP3 = nullptr;
    }
}

QPInverseProblemSolver::~QPInverseProblemSolver()
{
    deleteProblems();
}

void QPInverseProblemSolver::init()
{
    ConstraintSolver::init();
    deleteProblems();
    createProblems();
    m_currentCP->init();

    // Prevents ConstraintCorrection accumulation due to multiple AnimationLoop initialization on
    // dynamic components Add/Remove operations.
    if (!m_constraintsCorrections.empty())
        m_constraintsCorrections.clear();

    getContext()->get<BaseConstraintCorrection>(&m_constraintsCorrections, BaseContext::SearchDown);
    m_isConstraintCorrectionActive.resize(m_constraintsCorrections.size());
    m_context = (Node*) getContext();

    VectorOperations vop(ExecParams::defaultInstance(), this->getContext());
    {
        MultiVecDeriv lambda(&vop, m_lambdaId);
        lambda.realloc(&vop,false,true);
        m_lambdaId = lambda.id();
    }
    {
        MultiVecDeriv dx(&vop, m_dxId);
        dx.realloc(&vop,false,true);
        m_dxId = dx.id();
    }

    if(d_multithreading.getValue())
        sofa::simulation::MainTaskSchedulerFactory::createInRegistry()->init();

    for (const auto& constraintCorrection : m_constraintsCorrections)
    {
        constraintCorrection->addConstraintSolver(this);
    }
}

void QPInverseProblemSolver::reinit()
{
    deleteProblems();
    createProblems();
    m_currentCP->init();
}

void QPInverseProblemSolver::cleanup()
{
    if (!m_constraintsCorrections.empty())
    {
        for (unsigned int i = 0; i < m_constraintsCorrections.size(); i++)
            m_constraintsCorrections[i]->removeConstraintSolver(this);
        m_constraintsCorrections.clear();
    }

    VectorOperations vop(ExecParams::defaultInstance(), this->getContext());
    vop.v_free(m_lambdaId, false, true);
    vop.v_free(m_dxId, false, true);
    ConstraintSolver::cleanup();
}

void QPInverseProblemSolver::removeConstraintCorrection(sofa::core::behavior::BaseConstraintCorrection *s)
{
    m_constraintsCorrections.erase(std::remove(m_constraintsCorrections.begin(), m_constraintsCorrections.end(), s), m_constraintsCorrections.end());
}

bool QPInverseProblemSolver::prepareStates(const ConstraintParams *cParams, MultiVecId res1, MultiVecId res2)
{
    SOFA_UNUSED(res1);
    SOFA_UNUSED(res2);

    AdvancedTimer::StepVar vtimer("PrepareStates");

    m_lastCP = m_currentCP;

    m_time = 0.0;
    m_timeTotal = 0.0;
    m_timeScale = 1000.0 / (double)CTime::getTicksPerSec();

    sofa::simulation::common::VectorOperations vop(cParams, this->getContext());

    {
        MultiVecDeriv lambda(&vop, m_lambdaId);
        lambda.realloc(&vop,false,true);
        m_lambdaId = lambda.id();

        clearMultiVecId(getContext(), cParams, m_lambdaId);
    }

    {
        MultiVecDeriv dx(&vop, m_dxId);
        dx.realloc(&vop,false,true);
        m_dxId = dx.id();

        clearMultiVecId(getContext(), cParams, m_dxId);
    }

    if ( d_displayTime.getValue() )
    {
        m_time = (double) m_timer.getTime();
        m_timeTotal = (double) m_timerTotal.getTime();
    }

    return true;
}

bool QPInverseProblemSolver::buildSystem(const ConstraintParams *cParams, MultiVecId res1, MultiVecId res2)
{
    SOFA_UNUSED(res1);
    SOFA_UNUSED(res2);

    unsigned int nbLinesTotal = 0;

    accumulateConstraint(cParams, nbLinesTotal);
    setConstraintProblemSize(nbLinesTotal);
    computeConstraintViolation(cParams);

    if (f_printLog.getValue())
        msg_info() <<nbLinesTotal<<" lines of constraint";

    getConstraintCorrectionState();

    buildCompliance(cParams);

    if (d_displayTime.getValue())
    {
        msg_info() <<"build system in " << ( (double) m_timer.getTime() - m_time)*m_timeScale<<" ms";
        m_time = (double) m_timer.getTime();
    }

    return true;
}

inline void QPInverseProblemSolver::accumulateConstraint(const ConstraintParams *cParams,
                                                         unsigned int & nbLinesTotal)
{
    AdvancedTimer::stepBegin("Accumulate Constraint");

    // Mechanical action executed from root node to propagate the constraints
    MechanicalResetConstraintVisitor(cParams).execute(m_context);

    m_currentCP->clearProblem();

    module::QPMechanicalSetConstraint(cParams,
                              sofa::core::vec_id::write_access::constraintJacobian,
                              nbLinesTotal,
                              m_currentCP).execute(m_context);

    module::QPMechanicalAccumulateConstraint(cParams,
                                     sofa::core::vec_id::write_access::constraintJacobian,
                                     d_reverseAccumulateOrder.getValue()).execute(m_context);

    MechanicalParams mparams = MechanicalParams(*cParams);
    MechanicalProjectJacobianMatrixVisitor(&mparams).execute(m_context);

    AdvancedTimer::stepEnd("Accumulate Constraint");
}

inline void QPInverseProblemSolver::setConstraintProblemSize(const unsigned int &nbLinesTotal)
{
    AdvancedTimer::valSet("numConstraints", nbLinesTotal);
    m_currentCP->clear(nbLinesTotal);
}

inline void QPInverseProblemSolver::computeConstraintViolation(const ConstraintParams *cParams)
{
    sofa::helper::ScopedAdvancedTimer timer("Get Constraint Value");
    sofa::component::constraint::lagrangian::solver::MechanicalGetConstraintViolationVisitor(cParams, &m_currentCP->dFree).execute(m_context);
}

inline void QPInverseProblemSolver::getConstraintCorrectionState()
{
    for (unsigned int i = 0; i < m_constraintsCorrections.size(); i++)
        m_isConstraintCorrectionActive[i] = !m_constraintsCorrections[i]->getContext()->isSleeping();
}

inline void QPInverseProblemSolver::buildCompliance(const ConstraintParams *cParams)
{
    AdvancedTimer::stepBegin("Get Compliance");
    msg_info() << "computeCompliance in "  << m_constraintsCorrections.size()<< " constraintCorrections";

    if(d_multithreading.getValue()){

        sofa::simulation::TaskScheduler* taskScheduler = sofa::simulation::MainTaskSchedulerFactory::createInRegistry();
        sofa::simulation::CpuTask::Status status;

        sofa::type::vector<QPInverseProblemSolver::ComputeComplianceTask> tasks;
        sofa::Index nbTasks = m_constraintsCorrections.size();
        tasks.resize(nbTasks, QPInverseProblemSolver::ComputeComplianceTask(&status));
        sofa::Index dim = m_currentCP->W.rowSize();

        for (sofa::Index i=0; i<nbTasks; i++)
        {
            sofa::core::behavior::BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            tasks[i].set(cc, *cParams, dim);
            taskScheduler->addTask(&tasks[i]);
        }
        taskScheduler->workUntilDone(&status);

        auto & W = m_currentCP->W;

        // Accumulate the contribution of each constraints
        // into the system's compliant matrix W
        for (sofa::Index i = 0; i < nbTasks; i++) {
            sofa::core::behavior::BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            const auto & Wi = tasks[i].W;

            for (sofa::Index j = 0; j < dim; ++j)
                for (sofa::Index l = 0; l < dim; ++l)
                    W.add(j, l, Wi.element(j,l));
        }

    } else {
        for (unsigned int i=0; i<m_constraintsCorrections.size(); i++)
        {
            sofa::core::behavior::BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            sofa::helper::AdvancedTimer::stepBegin("Object name: "+cc->getName());
            cc->addComplianceInConstraintSpace(cParams, &m_currentCP->W);
            sofa::helper::AdvancedTimer::stepEnd("Object name: "+cc->getName());
        }
    }

    AdvancedTimer::stepEnd("Get Compliance");
}

void QPInverseProblemSolver::rebuildSystem(double massFactor, double forceFactor)
{
    d_graph.beginEdit()->clear();
    d_graph.endEdit();

    //rebuildConstraintCorrectionSystem()
    for (unsigned int i=0; i<m_constraintsCorrections.size(); i++)
    {
        BaseConstraintCorrection* CC = m_constraintsCorrections[i];
        CC->rebuildSystem(massFactor, forceFactor);
    }
}

bool QPInverseProblemSolver::solveSystem(const ConstraintParams * cParams,
                                         MultiVecId res1,
                                         MultiVecId res2)
{

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(res1);
    SOFA_UNUSED(res2);


    double time = getContext()->getTime();
    m_currentCP->setTime(time);
    m_currentCP->setEpsilon(d_epsilon.getValue());
    m_currentCP->setEnergyActuatorsOnly(d_actuatorsOnly.getValue());
    m_currentCP->setTolerance(d_tolerance.getValue());
    m_currentCP->setMaxIterations(d_maxIterations.getValue());
    m_currentCP->setFrictionCoeff(d_responseFriction.getValue());
    m_currentCP->allowSliding(d_allowSliding.getValue());
    if(d_minContactForces.isSet()) m_currentCP->setMinContactForces(d_minContactForces.getValue());
    if(d_maxContactForces.isSet()) m_currentCP->setMaxContactForces(d_maxContactForces.getValue());

    double objective;
    int iterations;
    {
        sofa::helper::ScopedAdvancedTimer("ConstraintsQP");
        m_currentCP->solve(objective, iterations);

    }

    module::QPInverseProblem::QPConstraintLists* qpCLists = m_currentCP->getQPConstraintLists();

    map<string, vector<SReal>>& graph = *d_graph.beginEdit();
    vector<SReal>& graph_effectors = graph[string("#Effectors:")];
    graph_effectors.clear();
    graph_effectors.push_back(qpCLists->effectorRowIds.size());

    vector<SReal>& graph_actuators = graph[string("#Actuators:")];
    graph_actuators.clear();
    graph_actuators.push_back(qpCLists->actuatorRowIds.size());

    vector<SReal>& graph_equality = graph[string("#Equality:")];
    graph_equality.clear();
    graph_equality.push_back(qpCLists->equalityRowIds.size());

    vector<SReal>& graph_contacts = graph[string("#Contacts:")];
    graph_contacts.clear();
    graph_contacts.push_back(qpCLists->contactRowIds.size());

    vector<SReal>& graph_error = graph[string("Last Objective:")];
    graph_error.clear();
    graph_error.push_back(objective);
    d_objective.setValue(objective);

    vector<SReal>& graph_it = graph[string("Last Iterations:")];
    graph_it.clear();
    graph_it.push_back(iterations);

    if (f_printLog.getValue())
    {
        int count = d_countdownFilterStartPerturb.getValue();
        m_currentCP->displayQNormVariation(count);
        d_countdownFilterStartPerturb.setValue(count);
        m_currentCP->displayQPSystem();
    }

    if ( d_displayTime.getValue() )
    {
        msg_info() <<" TOTAL solve QP " <<( (double) m_timer.getTime() - m_time)*m_timeScale<<" ms";
        m_time = (double) m_timer.getTime();
    }

    if(f_printLog.getValue())
        m_currentCP->displayResult();

    if(d_saveMatrices.getValue())
        m_currentCP->saveMatricesToFile();


    return true;
}


void QPInverseProblemSolver::computeResidual(const ExecParams* eparam)
{
    for (unsigned int i=0; i<m_constraintsCorrections.size(); i++)
    {
        BaseConstraintCorrection* CC = m_constraintsCorrections[i];
        CC->computeResidual(eparam,&m_currentCP->f);
    }
}

bool QPInverseProblemSolver::applyCorrection(const ConstraintParams *cParams, MultiVecId res1, MultiVecId res2)
{
    AdvancedTimer::stepBegin("Compute And Apply Motion Correction");

    if (cParams->constOrder() == sofa::core::ConstraintOrder::POS_AND_VEL)
    {
        MultiVecCoordId xId(res1);
        MultiVecDerivId vId(res2);
        for (unsigned int i = 0; i < m_constraintsCorrections.size(); i++)
        {
            if (!m_isConstraintCorrectionActive[i])
                continue;

            BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            cc->computeMotionCorrectionFromLambda(cParams, getDx(), &m_currentCP->f);
            cc->applyMotionCorrection(cParams, xId, vId, cParams->dx(), getDx());
        }
    }
    else if (cParams->constOrder() == sofa::core::ConstraintOrder::POS)
    {
        MultiVecCoordId xId(res1);
        for (unsigned int i = 0; i < m_constraintsCorrections.size(); i++)
        {
            if (!m_isConstraintCorrectionActive[i])
                continue;

            BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            cc->computeMotionCorrectionFromLambda(cParams, getDx(), &m_currentCP->f);
            cc->applyPositionCorrection(cParams, xId, cParams->dx(), getDx());
        }
    }
    else if (cParams->constOrder() == sofa::core::ConstraintOrder::VEL)
    {
        MultiVecDerivId vId(res1);
        for (unsigned int i = 0; i < m_constraintsCorrections.size(); i++)
        {
            if (!m_isConstraintCorrectionActive[i])
                continue;

            BaseConstraintCorrection* cc = m_constraintsCorrections[i];
            if (!cc->isActive())
                continue;

            cc->computeMotionCorrectionFromLambda(cParams, getDx(), &m_currentCP->f);
            cc->applyVelocityCorrection(cParams, vId, cParams->dx(), getDx());
        }
    }

    AdvancedTimer::stepEnd("Compute And Apply Motion Correction");

    AdvancedTimer::stepBegin("Store Constraint Lambdas");
    /// Some constraint correction schemes may have written the constraint motion space lambda in the lambdaId VecId.
    /// In order to be sure that we are not accumulating things twice, we need to clear.
    clearMultiVecId(getContext(), cParams, m_lambdaId);

    /// Store lambda and accumulate.
    ConstraintStoreLambdaVisitor v(cParams, &m_currentCP->f);
    this->getContext()->executeVisitor(&v);
    AdvancedTimer::stepEnd("Store Constraint Lambdas");


    if (d_displayTime.getValue())
        msg_info() << "TotalTime " << ((double) m_timerTotal.getTime() - m_timeTotal) * m_timeScale << " ms" ;
    return true;
}


sofa::component::constraint::lagrangian::solver::ConstraintProblem* QPInverseProblemSolver::getConstraintProblem()
{
    return m_lastCP;
}

void QPInverseProblemSolver::lockConstraintProblem(sofa::core::objectmodel::BaseObject *from,
    sofa::component::constraint::lagrangian::solver::ConstraintProblem* p1,
    sofa::component::constraint::lagrangian::solver::ConstraintProblem* p2)
{
    SOFA_UNUSED(from);
    if( (m_currentCP != p1) && (m_currentCP != p2) ) //The current ConstraintProblem is not locked
        return;

    if( (m_CP1 != p1) && (m_CP1 != p2) ) //cp1 is not locked
        m_currentCP = m_CP1;
    else if( (m_CP2 != p1) && (m_CP2 != p2) ) //cp2 is not locked
        m_currentCP = m_CP2;
    else
        m_currentCP = m_CP3; //cp1 et cp2 are locked, thus cp3 is not locked
}


////////////////////////////////////////////// FACTORY /////////////////////////////////////////////////

using namespace sofa::core;

void registerQPInverseProblemSolver(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("An inverse problem solver using the "
                                                    "Quadratic Programing formulation")
                                 .add< QPInverseProblemSolver >(true));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

} // namespace
