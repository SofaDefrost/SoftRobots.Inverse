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

#include <SoftRobots.Inverse/component/behavior/Actuator.h>

namespace softrobotsinverse::behavior
{

template<class DataTypes>
Actuator<DataTypes>::Actuator(sofa::core::behavior::MechanicalState<DataTypes> *mm)
    : softrobots::behavior::SoftRobotsConstraint<DataTypes>(mm),
      d_applyForce(initData(&d_applyForce, true, "applyForce", "If false, no force will be applied. Default value is true."))
{
    m_constraintType = ACTUATOR;
}

template<class DataTypes>
Actuator<DataTypes>::~Actuator()
{
}

template<class DataTypes>
void Actuator<DataTypes>::storeResults(sofa::type::vector<double> &lambda, sofa::type::vector<double> &delta)
{
    SOFA_UNUSED(delta);

    if (!d_applyForce.getValue())
        for (unsigned int i=0; i<m_nbLines; i++)
            lambda[i] = 0.;
}


} // namespace

