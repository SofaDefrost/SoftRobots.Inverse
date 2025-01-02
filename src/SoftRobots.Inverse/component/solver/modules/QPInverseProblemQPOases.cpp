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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMQPOASES_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMQPOASES_CPP


#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemQPOases.h>

#include <sofa/helper/AdvancedTimer.h>
#include <qpOASES.hpp>

namespace softrobotsinverse::solver::module
{

using sofa::helper::AdvancedTimer;

void QPInverseProblemQPOases::solveInverseProblem(double& objective,
                                               vector<double> &result,
                                               vector<double> &dual)
{
    m_qpSystem->previousResult = result;

    AdvancedTimer::stepBegin("QP resolution (qpOASES)");

    int ASize = m_qpSystem->A.size();
    int AeqSize = m_qpSystem->Aeq.size();
    int nbVariables = m_qpSystem->dim;
    int nbConstraints = ASize+AeqSize;

    real_t* Q = new real_t[nbVariables*nbVariables];
    real_t* c = new real_t[nbVariables];
    real_t* lambda = new real_t[nbVariables];
    real_t* l = new real_t[nbVariables];
    real_t* u = new real_t[nbVariables];
    real_t* A  = new real_t[nbConstraints*nbVariables];
    real_t* bu = new real_t[nbConstraints];
    real_t* bl = new real_t[nbConstraints];

    updateQPMatrices(Q, c, l, u, A, bl, bu);

    qpOASES::int_t nWSR = 500;
    //    Eulalie.C (01/19): QProblemB of qpOASES for simply bounded problem does not work (constraints are not satisfied). We should either find why or remove this commented block of code
    //    if(nbConstraints==0)
    //    {
    //        //only a bounded QP problem (without additional inequalities or equalities)
    //        //From pqOASES manual: This special form can be exploited within the solution algorithm for speeding up
    //        //the computation, typically by a factor of three to five.
    //        QProblemB problem(nbVariables);
    //        problem.setPrintLevel(qpOASES::PL_LOW);
    //        problem.init(Q, c, l, u, nWSR);
    //        problem.getPrimalSolution(lambda);
    //    }
    //    else
    //    {
    qpOASES::QProblem problem = getNewQProblem(nWSR);

    problem.init(Q, c, A, l, u, bl, bu, nWSR);

    if(problem.isInfeasible())
    {
        if(m_qpCLists->contacts.size()>0)
        {
            msg_warning("QPInverseProblemImpl") << "QP infeasible at time = " << m_time << " with " << m_qpCParams->contactStates.size() << " contacts, check constraint on actuators." ;
            m_constraintHandler->checkAndUpdateActuatorConstraints(result, m_qpSystem, m_qpCLists);
            updateQPMatrices(Q, c, l, u, A, bl, bu);

            problem = getNewQProblem(nWSR);
            problem.init(Q, c, A, l, u, bl, bu, nWSR);
        }

        if(problem.isInfeasible() || !problem.isSolved())
        {
            msg_warning("QPInverseProblemImpl") << "QP infeasible at time = " << m_time << ", try with option HST_INDEF." ;
            m_constraintHandler->buildInequalityConstraintMatrices(result, m_qpSystem, m_qpCLists);
            m_constraintHandler->getConstraintOnLambda(result, m_qpSystem, m_qpCLists);
            updateQPMatrices(Q, c, l, u, A, bl, bu);

            problem = getNewQProblem(nWSR);
            problem.setHessianType(qpOASES::HST_INDEF);
            problem.init(Q, c, A, l, u, bl, bu, nWSR);

            if(problem.isInfeasible())
                msg_error("QPInverseProblemImpl") << "QP infeasible at time = " << m_time << ", iteration = " << m_iteration << ", and final nWSR = " << nWSR;
        }
    }

    {
        const qpOASES::returnValue primalResult = problem.getPrimalSolution(lambda);
        msg_error_when(primalResult != qpOASES::SUCCESSFUL_RETURN, "QPInverseProblemImpl") << "getPrimalSolution failed";
    }

    objective = problem.getObjVal();

    real_t * slack = new real_t[nbVariables+nbConstraints]; // dual solution: slack[0:nV-1] => corresponds to lambda, slack[nV:nC+1] => corresponds to dual variables
    {
        const qpOASES::returnValue dualResult = problem.getDualSolution(slack);
        msg_error_when(dualResult != qpOASES::SUCCESSFUL_RETURN, "QPInverseProblemImpl") << "getDualSolution failed";
    }

    dual.resize(nbConstraints);
    for (int i=0; i<nbConstraints; i++)
        dual[i]=slack[nbVariables+i];
    delete[] slack;
    //    }

    result.clear();
    result.resize(nbVariables);
    for (int i=0; i<nbVariables; i++){
        result[i]=lambda[i];
    }

    delete[] u;
    delete[] l;
    delete[] A;
    delete[] bl;
    delete[] bu;
    delete[] Q;
    delete[] c;
    delete[] lambda;

    AdvancedTimer::stepEnd("QP resolution (qpOASES)");
}


qpOASES::QProblem QPInverseProblemQPOases::getNewQProblem(int& nWSR)
{
    int nbVariables = m_qpSystem->dim;
    int nbConstraints = m_qpSystem->A.size()+m_qpSystem->Aeq.size();

    qpOASES::QProblem problem(nbVariables, nbConstraints);

    qpOASES::Options options;
    problem.setOptions(options);

    problem.setPrintLevel(qpOASES::PL_LOW);

    nWSR = 500; // problem.init() changes the variable nWSR with the number of working set recalculation it took to solve the problem. So we have to update it.

    return problem;
}

void QPInverseProblemQPOases::updateQPMatrices(real_t * Q, real_t * c, real_t * l, real_t * u,
                                               real_t * A, real_t * bl, real_t * bu)
{
    for (unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        for (unsigned int j=0; j<m_qpSystem->dim; j++)
            Q[i*m_qpSystem->dim+j] = m_qpSystem->Q[i][j];
        c[i] = m_qpSystem->c[i];
    }

    int ASize = m_qpSystem->A.size();
    int AeqSize = m_qpSystem->Aeq.size();
    for (int i=0; i<ASize; i++)
    {
        for (unsigned int j=0; j<m_qpSystem->dim; j++)
            A[i*m_qpSystem->dim+j] = m_qpSystem->A[i][j];

        bu[i]=m_qpSystem->bu[i];
        if (m_qpSystem->hasBothSideInequalityConstraint) bl[i]=m_qpSystem->bl[i];
        else bl[i]=-1e99;
    }

    if(m_qpSystem->Aeq.size())
        for (int i=0; i<AeqSize; i++)
        {
            for (unsigned int j=0; j<m_qpSystem->dim; j++)
                A[(i+ASize)*m_qpSystem->dim+j] = m_qpSystem->Aeq[i][j];

            bu[ASize+i]=m_qpSystem->beq[i];
            bl[ASize+i]=m_qpSystem->beq[i];
        }

    for (unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        u[i]=m_qpSystem->u[i];
        l[i]=m_qpSystem->l[i];
    }
}

} // namespace

#endif