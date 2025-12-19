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
#pragma once

#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <sofa/core/behavior/BaseLagrangianConstraint.h>
#include <sofa/core/behavior/ConstraintSolver.h>
#include <sofa/core/behavior/BaseConstraintCorrection.h>
#include <sofa/core/objectmodel/DataCallback.h>
#include <sofa/core/objectmodel/KeypressedEvent.h>
#include <sofa/simulation/TaskScheduler.h>
#include <sofa/simulation/InitTasks.h>
#include <sofa/helper/map.h>
#include <sofa/helper/OptionsGroup.h>

#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>
#include <SoftRobots.Inverse/component/config.h>

using sofa::core::objectmodel::KeypressedEvent ;

namespace softrobotsinverse::solver
{

using sofa::helper::system::thread::CTime;
using sofa::type::vector;
using sofa::core::behavior::ConstraintSolver;
using sofa::core::behavior::BaseConstraintCorrection;
using sofa::core::MultiVecId;
using sofa::core::ConstraintParams;
using sofa::core::ExecParams;
using std::map;
using std::string;
using sofa::simulation::Node;
using sofa::core::MultiVecDerivId;


/**
 * This component solves an inverse problem set by actuator and effector constraints. The method
 * is based on the formulation of a quadratic program (QP).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/

class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemSolver : public sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl
{
public:

    SOFA_CLASS(QPInverseProblemSolver, ConstraintSolverImpl);

    typedef vector<BaseConstraintCorrection*> list_cc;
    typedef vector<list_cc> VecListcc;

public:
    QPInverseProblemSolver();
    ~QPInverseProblemSolver() override;

    ////////////////////// Inherited from BaseObject /////////////////////////////////
    void init() override;
    void reinit() override;
    void cleanup() override;
    ////////////////////////////////////////////////////////////////////////////////

    ////////////////////// Inherited from ConstraintSolver ///////////////////////////
    bool prepareStates(const ConstraintParams * cParams,
                       MultiVecId res1,
                       MultiVecId res2=MultiVecId::null()) override;

    bool buildSystem(const ConstraintParams * cParams,
                     MultiVecId res1,
                     MultiVecId res2=MultiVecId::null()) override;

    void rebuildSystem(double massFactor,
                       double forceFactor) override;

    bool solveSystem(const ConstraintParams* cParams,
                     MultiVecId res1, MultiVecId res2=MultiVecId::null()) override;

    bool applyCorrection(const ConstraintParams * cParams,
                         MultiVecId res1,
                         MultiVecId res2=MultiVecId::null()) override;

    void computeResidual(const ExecParams* params) override;

    void removeConstraintCorrection(BaseConstraintCorrection *s) override;

    MultiVecDerivId getLambda() const override
    {
        return m_lambdaId;
    }

    MultiVecDerivId getDx() const override
    {
        return m_dxId;
    }


    ////////////////////// Inherited from ConstraintSolverImpl ////////////////////////
    sofa::component::constraint::lagrangian::solver::ConstraintProblem* getConstraintProblem() override;

    void lockConstraintProblem(BaseObject *from,
                               sofa::component::constraint::lagrangian::solver::ConstraintProblem* CP1,
                               sofa::component::constraint::lagrangian::solver::ConstraintProblem* CP2=nullptr) override;
    /////////////////////////////////////////////////////////////////////////////////


    sofa::Data<bool>      d_displayTime;
    sofa::Data<bool>      d_multithreading;
    sofa::Data<bool>      d_reverseAccumulateOrder;

    sofa::Data<int>       d_countdownFilterStartPerturb;
    sofa::Data<bool>      d_saveMatrices;

    sofa::Data<int>       d_maxIterations;
    sofa::Data<double>    d_tolerance;
    sofa::Data<double>    d_responseFriction;

    sofa::Data<sofa::helper::OptionsGroup> d_qpSolver;

    sofa::Data<double>    d_epsilon;
    sofa::Data<bool>      d_actuatorsOnly;
    sofa::Data<bool>      d_allowSliding;
    sofa::Data<map <string, vector<SReal> > > d_graph;
    sofa::Data<double>    d_minContactForces;
    sofa::Data<double>    d_maxContactForces;
    sofa::Data<SReal >    d_objective;

protected:

    MultiVecDerivId m_lambdaId;
    MultiVecDerivId m_dxId;

    module::QPInverseProblemImpl *m_CP1, *m_CP2, *m_CP3;
    module::QPInverseProblemImpl *m_lastCP, *m_currentCP;
    vector<BaseConstraintCorrection*> m_constraintsCorrections;
    vector<char> m_isConstraintCorrectionActive;
    sofa::core::objectmodel::DataCallback m_qpSolverCB;

    Node *m_context;

    CTime m_timer;
    CTime m_timerTotal;

    double m_time;
    double m_timeTotal;
    double m_timeScale;

    virtual void createProblems();
    void deleteProblems();

private:
    void accumulateConstraint(const ConstraintParams *cParams, unsigned int & nbLinesTotal);
    void setConstraintProblemSize(const unsigned int &nbLinesTotal);
    void computeConstraintViolation(const ConstraintParams *cParams);
    void getConstraintCorrectionState();
    void buildCompliance(const ConstraintParams *cParams);

    class ComputeComplianceTask : public sofa::simulation::CpuTask
    {
    public:
        ComputeComplianceTask(sofa::simulation::CpuTask::Status* status): CpuTask(status) {}
        ~ComputeComplianceTask() override {}

        MemoryAlloc run() final {
            cc->addComplianceInConstraintSpace(&cparams, &W);
            return MemoryAlloc::Stack;
        }

        void set(sofa::core::behavior::BaseConstraintCorrection* _cc, sofa::core::ConstraintParams _cparams, int dim){
            cc = _cc;
            cparams = _cparams;
            W.resize(dim,dim);
        }

    private:
        sofa::core::behavior::BaseConstraintCorrection* cc;
        sofa::linearalgebra::LPtrFullMatrix<double> W;
        sofa::core::ConstraintParams cparams;
        friend class QPInverseProblemSolver;
    };
};

} // namespace

