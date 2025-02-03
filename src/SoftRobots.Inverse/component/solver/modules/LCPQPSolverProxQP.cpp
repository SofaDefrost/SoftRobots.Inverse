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

#include <sofa/helper/logging/Messaging.h>
#include <SoftRobots.Inverse/component/solver/modules/LCPQPSolverProxQP.h>

#include <proxsuite/proxqp/dense/dense.hpp>

#include <optional>

namespace softrobotsinverse::solver::module
{

void LCPQPSolverProxQP::solve(int dim, double*q, double**M, double*res)
{
  const int n_in = dim;
  const int n_eq = 0;

  //////////////////////////////////////////
  // Constraints on lambda
  //////////////////////////////////////////
  Eigen::VectorXd lbox(dim); // l
  Eigen::VectorXd ubox(dim); // u
  for (int i=0; i<dim; ++i)
  {
      ubox[i] = 1e99;
      lbox[i] = 0.;
  }


  //////////////////////////////////////////
  // Conversion to Eigen type
  // TODO: try Eigen Mapping instead
  //////////////////////////////////////////
  Eigen::MatrixXd H(dim, dim); // Q
  Eigen::VectorXd g(dim); // c

  for (int i = 0; i < dim; ++i)
  {
      for (int j = 0; j < dim; ++j)
          H(i, j) = M[i][j];
      g[i] = q[i];
  }


  //////////////////////////////////////////
  // Inequality constraint matrices
  // TODO: try Eigen Mapping instead
  //////////////////////////////////////////
  Eigen::VectorXd l(n_in); // bl
  Eigen::VectorXd u(n_in); // bu
  Eigen::MatrixXd C(n_in, n_in); // A

  for (int i = 0; i < n_in; ++i)
  {
      for (int j = 0; j < n_in; ++j)
          C(i, j) = M[i][j];

      u[i] = 1e99;
      l[i] = -q[i];
  }


  /////////////////////////////////////////
  // Solve
  /////////////////////////////////////////

  
  proxsuite::proxqp::dense::QP<double> qp(dim, n_eq, n_in, true); // create the QP object with box constraints
  
  const double rho = 1e-14; // default is 1e-6, but has to be set smaller due to low values in Hessian when using data in meters (e.g. from URDF models)
  
  qp.init(H, g, std::nullopt, std::nullopt, C, l, u, lbox, ubox, true, rho); // initialize the model
  // qp.init(H, g, A, b, C, l, u, lbox, ubox, true, rho); // initialize the model

  qp.settings.eps_rel = 0.;
  qp.settings.eps_abs = 1e-12; // default is 1e-5
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
  
  // get primal solution
  for (int i = 0; i < dim; ++i)
  {
      res[i] = qp.results.x[i];
  }
}

} // namespace
