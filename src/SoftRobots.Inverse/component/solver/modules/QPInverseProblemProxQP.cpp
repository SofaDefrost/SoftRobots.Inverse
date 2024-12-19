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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMPROXQP_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMPROXQP_CPP


#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemProxQP.h>

#include <sofa/helper/AdvancedTimer.h>

#include <proxsuite/proxqp/dense/dense.hpp>

namespace softrobotsinverse::solver::module
{

using sofa::helper::AdvancedTimer;

void QPInverseProblemProxQP::solveInverseProblem(double& objective,
                                               vector<double> &result,
                                               vector<double> &dual)
{
    m_qpSystem->previousResult = result;

    AdvancedTimer::stepBegin("QP resolution (proxQP)");

    const int n_eq = m_qpSystem->Aeq.size();
    const int n_in = m_qpSystem->A.size();
    const int n_constraints = n_eq + n_in;
    const int d = m_qpSystem->dim;
    Eigen::MatrixXd H(d, d);
    Eigen::VectorXd g(d);
    Eigen::MatrixXd A(n_eq, d);
    Eigen::VectorXd b(n_eq);
    Eigen::MatrixXd C(n_in, d);
    Eigen::VectorXd l(n_in);
    Eigen::VectorXd u(n_in);
    Eigen::VectorXd lbox(d);
    Eigen::VectorXd ubox(d);

    updateQPMatrices(H, g, A, b, C, l, u, lbox, ubox);

    proxsuite::proxqp::dense::QP<double> qp(d, n_eq, n_in, true); // create the QP object with box constraints
    
    const double rho = 1e-14; // default is 1e-6, but has to be set smaller due to low values in Hessian when using data in meters (e.g. from URDF models)
    qp.init(H, g, A, b, C, l, u, lbox, ubox, true, rho); // initialize the model

    qp.settings.eps_rel = 0.;
    qp.settings.eps_abs = 1e-9; // default is 1e-5
    qp.settings.check_duality_gap = true;
    qp.settings.verbose = false;
    qp.settings.max_iter = 3000;
    // same here, default is 1e-4 for these epsilons, but has to be set smaller due to low values in Hessian when using data in meters (e.g. from URDF models)
    qp.settings.eps_primal_inf = 1e-12;
    qp.settings.eps_dual_inf = 1e-12;

    qp.solve();

    switch(qp.results.info.status)
    {
      case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED:
        msg_info("QPInverseProblemImpl") << "Solver status: PROXQP_SOLVED";
      break;
      case proxsuite::proxqp::QPSolverOutput::PROXQP_MAX_ITER_REACHED:
        msg_warning("QPInverseProblemImpl") << "Solver status: PROXQP_MAX_ITER_REACHED";
      break;
      case proxsuite::proxqp::QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE:
        msg_warning("QPInverseProblemImpl") << "Solver status: PROXQP_PRIMAL_INFEASIBLE";
      break;
      case proxsuite::proxqp::QPSolverOutput::PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE:
        msg_warning("QPInverseProblemImpl") << "Solver status: PROXQP_SOLVED_CLOSEST_PRIMAL_FEASIBLE";
      break;
      case proxsuite::proxqp::QPSolverOutput::PROXQP_DUAL_INFEASIBLE:
        msg_warning("QPInverseProblemImpl") << "Solver status: PROXQP_DUAL_INFEASIBLE";
      break;
      case proxsuite::proxqp::QPSolverOutput::PROXQP_NOT_RUN:
        msg_warning("QPInverseProblemImpl") << "Solver status: PROXQP_NOT_RUN";
      break;
      default:
        msg_error("QPInverseProblemImpl") << "Unknown solver status";
    }

    objective = qp.results.info.objValue;

    // dual is the vector stacking dual solution for inequalities then equalities, i.e. dual = [z; y]^T
    dual.clear();
    dual.resize(n_constraints);
    for (int i = 0; i < n_in; ++i)
    {
        dual[i] = qp.results.z[i];  // lagrange optimal multiplier for inequalities constraints
    }
    for (int i = 0; i < n_eq; ++i)
    {
        dual[n_in + i] = qp.results.y[i]; // lagrange optimal multiplier for equalities constraints
    }

    // primal solution
    result.clear();
    result.resize(d);
    for (int i = 0; i < d; ++i)
    {
        result[i] = qp.results.x[i];
    }

    AdvancedTimer::stepEnd("QP resolution (proxQP)");
}

void QPInverseProblemProxQP::updateQPMatrices(Eigen::MatrixXd& H, Eigen::VectorXd& g, Eigen::MatrixXd& A, Eigen::VectorXd& b, Eigen::MatrixXd& C, Eigen::VectorXd& l, Eigen::VectorXd& u, Eigen::VectorXd& lbox, Eigen::VectorXd& ubox) const
{
    for (unsigned int i = 0; i < m_qpSystem->dim; ++i)
    {
        for (unsigned int j = 0; j < m_qpSystem->dim; ++j)
        {
            H(i, j) = m_qpSystem->Q[i][j];
        }
        g[i] = m_qpSystem->c[i];
    }

    const int n_in = m_qpSystem->A.size();
    const int n_eq = m_qpSystem->Aeq.size();
    for (int i = 0; i < n_in; ++i)
    {
        for (unsigned int j = 0; j < m_qpSystem->dim; ++j)
        {
            C(i, j) = m_qpSystem->A[i][j];
        }
        u[i] = m_qpSystem->bu[i];
        if (m_qpSystem->hasBothSideInequalityConstraint)
        {
          l[i] = m_qpSystem->bl[i];
        }
        else
        {
          l[i] = -1e99; // XXX
        }
    }

    if(n_eq > 0)
    {
        for (int i = 0; i < n_eq; ++i)
        {
            for (unsigned int j = 0; j < m_qpSystem->dim; ++j)
            {
                A(i, j) = m_qpSystem->Aeq[i][j];
            }
            b[i] = m_qpSystem->beq[i];
        }
    }
    for (unsigned int i = 0; i < m_qpSystem->dim; i++)
    {
        ubox[i] = m_qpSystem->u[i];
        lbox[i] = m_qpSystem->l[i];
    }
}

} // namespace

#endif