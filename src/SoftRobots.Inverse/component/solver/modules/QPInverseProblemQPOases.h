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

#include <qpOASES/Types.hpp>

//When the macro is defined by qpOASES/Types.hpp it has side effects on the
//rest of the code, in particular in boost (source_location). This macro is
//no longer necessary if a modern compiler is used.
#ifdef snprintf
    #undef snprintf
#endif

#include <qpOASES/QProblem.hpp>

#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>

#include <SoftRobots.Inverse/component/config.h>


namespace softrobotsinverse::solver::module
{

using sofa::type::vector;
using qpOASES::real_t;


class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblemQPOases : public QPInverseProblemImpl
{

public:
    QPInverseProblemQPOases();
    virtual ~QPInverseProblemQPOases() = default;

protected:
    virtual void solveInverseProblem(double &objective,
                             vector<double> &result,
                             vector<double> &dual) override;

private:
    void updateQPMatrices(real_t * Q, real_t * c, real_t * l, real_t * u,
                          real_t * A, real_t * bl, real_t * bu);

    qpOASES::QProblem getNewQProblem(int &nWSR);
};

} //namespace
