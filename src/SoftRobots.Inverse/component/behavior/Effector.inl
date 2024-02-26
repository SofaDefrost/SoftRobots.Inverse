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

#include <SoftRobots.Inverse/component/behavior/Effector.h>

namespace softrobotsinverse::behavior
{

template<class DataTypes>
Effector<DataTypes>::Effector(sofa::core::behavior::MechanicalState<DataTypes> *mm)
    : softrobots::behavior::SoftRobotsConstraint<DataTypes>(mm)
    , d_limitShiftToTarget(initData(&d_limitShiftToTarget, false, "limitShiftToTarget", "If true will limit the effector goal to be at \n"
                                                                                        "maxShiftToTarget."))

    , d_maxShiftToTarget(initData(&d_maxShiftToTarget, Real(1.), "maxShiftToTarget", "Maximum shift to effector goal if limitShiftToTarget \n"
                                                                                     "is set to true."))

    , d_maxSpeed(initData(&d_maxSpeed, Real(0.), "maxSpeed", "Limit the effector motion to a maximum speed."))
{
    m_constraintType = EFFECTOR;
}

template<class DataTypes>
Effector<DataTypes>::~Effector()
{
}

template<class DataTypes>
SReal Effector<DataTypes>:: getTarget(const Real& target, const Real& current)
{
    Real newTarget = target;
    if(d_limitShiftToTarget.getValue())
    {
        Real shift = abs(target-current);
        if(shift>d_maxShiftToTarget.getValue())
        {
            if(target>current)
                newTarget = current + d_maxShiftToTarget.getValue();
            else
                newTarget = current - d_maxShiftToTarget.getValue();
        }
    }

    return newTarget;
}

template<class DataTypes>
typename DataTypes::Coord Effector<DataTypes>:: getTarget(const Coord& target, const Coord& current)
{
    Coord newTarget = target;

    auto direction = DataTypes::coordDifference(target, current);
    Real displacement = direction.norm();
    Real stepMaxDisplacement = d_maxSpeed.getValue() * this->getContext()->getDt();

    if (d_maxSpeed.isSet() && displacement > stepMaxDisplacement)
    {
        for(sofa::Size i=0; i<DataTypes::Coord::total_size; i++)
        {
            newTarget[i] = current[i] + direction[i] / displacement * stepMaxDisplacement;
            newTarget[i] = getTarget(newTarget[i], current[i]);
        }
    }
    else
    {
        for(sofa::Size i=0; i<DataTypes::Coord::total_size; i++)
        {
            newTarget[i] = getTarget(target[i], current[i]);
        }
    }

    return newTarget;
}

} // namespace

