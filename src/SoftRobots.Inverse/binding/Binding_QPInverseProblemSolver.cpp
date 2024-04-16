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

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>

#include <SofaPython3/PythonFactory.h>
#include <SofaPython3/PythonEnvironment.h>
#include <SofaPython3/Sofa/Core/Binding_Base.h>

#include <SoftRobots.Inverse/binding/Binding_QPInverseProblemSolver.h>
#include <SoftRobots.Inverse/binding/Binding_QPInverseProblemSolver_doc.h>

SOFAPYTHON3_BIND_ATTRIBUTE_ERROR()

/// Makes an alias for the pybind11 namespace to increase readability.
namespace py { using namespace pybind11; }

namespace softrobotsinverse::python3
{
using sofa::core::objectmodel::Event;
using sofa::core::objectmodel::BaseObject;
using softrobotsinverse::solver::QPInverseProblemSolver;
using EigenDenseMatrix = Eigen::Matrix<SReal, Eigen::Dynamic, Eigen::Dynamic>;
using EigenMatrixMap = Eigen::Map<EigenDenseMatrix>;

std::string QPInverseProblemSolver_Trampoline::getClassName() const
{
    sofapython3::PythonEnvironment::gil acquire {"getClassName"};
    // Get the actual class name from python.
    return py::str(py::cast(this).get_type().attr("__name__"));
}

bool QPInverseProblemSolver_Trampoline::solveSystem()
{
    PYBIND11_OVERLOAD_PURE(bool,
                          QPInverseProblemSolver_Trampoline,
                          solveSystem,
                          );
}

bool QPInverseProblemSolver_Trampoline::solveSystem(const sofa::core::ConstraintParams* cParams,
                                                    sofa::core::MultiVecId res1,
                                                    sofa::core::MultiVecId res2)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(res1);
    SOFA_UNUSED(res2);

    const bool result = solveSystem();
    storeResults();

    return result;
}

void QPInverseProblemSolver_Trampoline::storeResults()
{
    SReal *lambda = m_currentCP->getF();
    SReal **w = m_currentCP->getW();
    SReal *dfree = m_currentCP->getDfree();

    solver::module::QPInverseProblem::QPConstraintLists* qpCLists = m_currentCP->getQPConstraintLists();

    const std::size_t nbActuatorRows = qpCLists->actuatorRowIds.size();
    const std::size_t nbEffectorRows = qpCLists->effectorRowIds.size();
    const std::size_t nbSensorRows   = qpCLists->sensorRowIds.size();
    const std::size_t nbContactRows  = qpCLists->contactRowIds.size();
    const std::size_t nbEqualityRows = qpCLists->equalityRowIds.size();
    const std::size_t nbRows = nbEffectorRows + nbActuatorRows + nbContactRows + nbSensorRows + nbEqualityRows;

    solver::module::QPInverseProblemImpl::QPSystem* qpSystem = m_currentCP->getQPSystem();
    qpSystem->delta.resize(nbRows);
    for(std::size_t i=0; i<nbRows; i++)
    {
        qpSystem->delta[i] = dfree[i];
        for(std::size_t j=0; j<nbRows; j++)
            qpSystem->delta[i] += lambda[j]*w[i][j];
    }

    m_currentCP->sendResults();
}

void moduleAddQPInverseProblemSolver(py::module &m)
{
    py::class_<softrobotsinverse::solver::QPInverseProblemSolver,
               sofa::component::constraint::lagrangian::solver::ConstraintSolverImpl,
               QPInverseProblemSolver_Trampoline,
               sofapython3::py_shared_ptr<softrobotsinverse::solver::QPInverseProblemSolver>> s(m, "QPInverseProblemSolver",
                                         py::dynamic_attr(),
                                         softrobotsinverse::python3::doc::QPInverseProblemSolver);

    s.def(py::init<>());
    s.def("solveSystem", &softrobotsinverse::solver::QPInverseProblemSolver::solveSystem);
}

}
