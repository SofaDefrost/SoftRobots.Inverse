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

#include <pybind11/pybind11.h>

#include <SoftRobots.Inverse/component/solver/QPInverseProblemSolver.h>

namespace softrobotsinverse::python3 {

class QPInverseProblemSolver_Trampoline : public softrobotsinverse::solver::QPInverseProblemSolver
{
public:
    SOFA_CLASS(QPInverseProblemSolver_Trampoline, softrobotsinverse::solver::QPInverseProblemSolver);

    /* Inherit the constructors */
    using softrobotsinverse::solver::QPInverseProblemSolver::QPInverseProblemSolver;

    bool solveSystem();

    bool solveSystem(const sofa::core::ConstraintParams* cParams,
                     sofa::core::MultiVecId res1,
                     sofa::core::MultiVecId res2=sofa::core::MultiVecId::null()) override;

    std::string getClassName() const override;

protected:
    void storeResults();
};

void moduleAddQPInverseProblemSolver(pybind11::module &m);

} /// namespace 

