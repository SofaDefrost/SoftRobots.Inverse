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

#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>

#include <SoftRobots.Inverse/component/config.h>


namespace softrobotsinverse::solver::module
{

using sofa::type::vector;

class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemProxQP : public QPInverseProblemImpl
{

public:
    QPInverseProblemProxQP();
    virtual ~QPInverseProblemProxQP() = default;

protected:
    virtual void solveInverseProblem(double &objective,
                             vector<double> &result,
                             vector<double> &dual) override;

private:
    void updateQPMatrices(Eigen::MatrixXd& H, Eigen::VectorXd& g, Eigen::MatrixXd& A,
       Eigen::VectorXd& b, Eigen::MatrixXd& C, Eigen::VectorXd& l, Eigen::VectorXd& u, 
       Eigen::VectorXd& lbox, Eigen::VectorXd& ubox) const;
};

} //namespace
