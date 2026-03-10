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

#include <Eigen/Core>

#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

#include <SoftRobots.Inverse/component/solver/modules/ConstraintHandler.h>
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblem.h>
#include <SoftRobots.Inverse/component/solver/modules/LCPQPSolver.h>

#include <SoftRobots.Inverse/component/config.h>


namespace softrobotsinverse::solver::module
{

using sofa::type::vector;


class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemImpl : public QPInverseProblem
{

public:

    typedef Eigen::Ref<Eigen::MatrixXd> RefMat;
    typedef const Eigen::Ref<const Eigen::MatrixXd> ConstRefMat;
    typedef Eigen::Ref<Eigen::VectorXd> RefVec;
    typedef const Eigen::Ref<const Eigen::VectorXd> ConstRefVec;

    QPInverseProblemImpl();
    virtual ~QPInverseProblemImpl();

    void init();
    void solve(double &objective, int &iterations);
    void setMinContactForces(const double& minContactForces) {m_qpCParams->minContactForces = minContactForces; m_qpCParams->hasMinContactForces = true;}
    void setMaxContactForces(const double& maxContactForces) {m_qpCParams->maxContactForces = maxContactForces; m_qpCParams->hasMaxContactForces = true;}

protected:

    ConstraintHandler* m_constraintHandler;
    ConstraintHandler::QPConstraintParams* m_qpCParams;
    std::unique_ptr<LCPQPSolver> m_lcpQpSolver;

    // Utils to prevent cycling in pivot algorithm
    vector<int>   m_currentSequence;
    vector<int>   m_previousSequence;
    vector<int>   m_sequence;

    int m_iteration{0};
    int m_step{0};

    // must be reimplemented in each qp solver specific child class
    virtual void solveInverseProblem(double &objective,
                             vector<double> &result,
                             vector<double> &dual) = 0;

    void computeEnergyScalingFactor(double& weight);
    void buildQPMatrices();

    void solveWithContact(vector<double>& result, double &objective, int &iterations);
    void solveContacts(vector<double>& res);

    void updateLambda(const vector<double>& x);
    bool isFeasible(const vector<double>& x);

    bool checkAndUpdatePivot(const vector<double>&result, const vector<double>&dual);
    bool isCycling(const int pivot);
    double getDelta(const vector<double> &result, const int& index);
    bool isIn(const vector<int> list, const int elem);
    std::string getContactsState();
};

} //namespace
