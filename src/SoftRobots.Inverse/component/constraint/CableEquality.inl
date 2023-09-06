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

#include <SoftRobots.Inverse/component/constraint/CableEquality.h>

namespace softrobotsinverse::constraint
{

using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;

template<class DataTypes>
CableEquality<DataTypes>::CableEquality(MechanicalState* object)
    : softrobotsinverse::behavior::Equality<DataTypes>(object)
    , softrobots::constraint::CableModel<DataTypes>(object)
    , d_constrainAtTime(initData(&d_constrainAtTime,Real(0.0), "constrainAtTime",
                          "No constraints will be applied before this time. \n"
                          "Example of use: to allow the cable to reach an \n"
                          "initial configuration before optimizing."))

    , d_displayCableLimit(initData(&d_displayCableLimit,false, "displayCableLimit",
                          "Display cable in red when the limit set by user has been reach."))
{
    d_displayCableLimit.setGroup("Visualization");
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaEqual.resize(1);
    m_deltaEqual.resize(1);
}


template<class DataTypes>
CableEquality<DataTypes>::~CableEquality()
{
}

template<class DataTypes>
void CableEquality<DataTypes>::init()
{
    softrobots::constraint::CableModel<DataTypes>::init();
    updateConstraint();
}


template<class DataTypes>
void CableEquality<DataTypes>::reinit()
{
    softrobots::constraint::CableModel<DataTypes>::reinit();
    updateConstraint();
}

template<class DataTypes>
void CableEquality<DataTypes>::reset()
{
    d_displacement.setValue(0.0);
    d_force.setValue(0.0);
    updateConstraint();
}

template<class DataTypes>
void CableEquality<DataTypes>::updateConstraint()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    ReadAccessor<sofa::Data<Real>> eqDisplacement = d_eqDisplacement;
    ReadAccessor<sofa::Data<Real>> eqForce = d_eqForce;


    if (d_eqDisplacement.isSet())
    {
        m_hasDeltaEqual = true;
        m_deltaEqual[0] = eqDisplacement;
    }

    if(d_eqForce.isSet())
    {
        m_hasLambdaEqual = true;
        m_lambdaEqual[0] = eqForce;
    }
}

template<class DataTypes>
void CableEquality<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                           sofa::core::MultiVecDerivId res,
                                           const BaseVector* lambda)
{
    softrobots::constraint::CableModel<DataTypes>::storeLambda(cParams,res,lambda);
    updateConstraint();
}

} // namespace
