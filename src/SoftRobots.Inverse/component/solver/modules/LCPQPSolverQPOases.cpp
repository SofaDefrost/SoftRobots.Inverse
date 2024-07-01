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

#include <qpOASES.hpp>
#include <sofa/type/vector.h>
#include <sofa/helper/logging/Messaging.h>
#include <SoftRobots.Inverse/component/solver/modules/LCPQPSolverQPOases.h>


namespace softrobotsinverse::solver::module
{

using qpOASES::QProblemB;
using qpOASES::QProblem;

using qpOASES::Options;
using qpOASES::real_t;
using qpOASES::int_t;

using sofa::type::vector;


void LCPQPSolverQPOases::solve(int dim, double*q, double**M, double*res)
{
    //////////////////////////////////////////
    // Constraints on lambda
    //////////////////////////////////////////
    real_t * l = new real_t[dim];
    real_t * u = new real_t[dim];

    for (int i=0; i<dim; i++)
    {
        u[i] = 1e99;
        l[i] = 0.;
    }


    //////////////////////////////////////////
    // Convertion to real_t for qpOASES
    //////////////////////////////////////////
    real_t * Q = new real_t[dim*dim];
    real_t * c = new real_t[dim];
    real_t * lambda = new real_t[dim];

    for (int i=0; i<dim; i++)
    {
        for (int j=0; j<dim; j++)
            Q[i*dim+j] = M[i][j];
        c[i] = q[i];
    }


    //////////////////////////////////////////
    // Inequality constraint matrices
    //////////////////////////////////////////
    real_t * A  = new real_t[dim*dim];
    real_t * bu = new real_t[dim];
    real_t * bl = new real_t[dim];

    for (int i=0; i<dim; i++)
    {
        for (int j=0; j<dim; j++)
            A[i*dim+j] = M[i][j];

        bu[i]=1e99;
        bl[i]=-q[i];
    }


    /////////////////////////////////////////
    // Solve
    /////////////////////////////////////////

    int_t nWSR = 500;
    QProblem problem(dim, dim);
    Options options;
    problem.setOptions(options);
    problem.setPrintLevel(qpOASES::PL_LOW);
    problem.init(Q, c, A, l, u, bl, bu, nWSR);
    problem.getPrimalSolution(lambda);

    if(problem.isInfeasible())
        msg_warning("LCPQPSolver") << "QP infeasible.";


    /////////////////////////////////////////
    // Store results
    /////////////////////////////////////////
    for (int i=0; i<dim; i++)
        res[i]=lambda[i];

    delete[] u;
    delete[] l;
    delete[] A;
    delete[] bl;
    delete[] bu;
    delete[] Q;
    delete[] c;
    delete[] lambda;
}

} // namespace
