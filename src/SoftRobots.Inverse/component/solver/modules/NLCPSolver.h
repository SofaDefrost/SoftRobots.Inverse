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

#include <sofa/helper/system/thread/CTime.h>
#include <sofa/type/vector.h>
#include <sofa/helper/LCPcalc.h>


// NLCP solver for friction contact
// Mainly based on LCPCalc implementation
// But with different discretisation of Coulomb's friction cone


namespace softrobotsinverse::solver::module
{

class NLCPSolver
{

protected:
    bool m_allowSliding;

public:
    NLCPSolver(){}
    ~NLCPSolver(){}

    int solve(int dim, double *dfree, double**W, double *f, double mu, double tol, int numItMax,
              bool useInitialF, bool verbose = false, double minW=0.0, double maxF=0.0,
              sofa::type::vector<double>* residuals = NULL, sofa::type::vector<double>* violations = NULL);

    void setAllowSliding(bool allowSliding) {m_allowSliding=allowSliding;}
};

class NLCPSolverMatrix33
{

public:
    double m_w[6];
    bool m_stored;

public:
    NLCPSolverMatrix33() {m_stored=false;}
    ~NLCPSolverMatrix33() {}

    void storeW(double &w11, double &w12, double &w13, double &w22, double &w23, double &w33);
    void GSState(double &mu, double &dn, double &dt, double &ds, double &fn, double &ft, double &fs, bool allowSliding);
};

} // namespace
