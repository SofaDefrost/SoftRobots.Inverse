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

#include <sofa/core/visual/VisualParams.h>

#include <SoftRobots.Inverse/component/constraint/CableEffector.h>

namespace softrobotsinverse::constraint
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;

template<class DataTypes>
CableEffector<DataTypes>::CableEffector(MechanicalState* object)
    : softrobotsinverse::behavior::Effector<DataTypes>(object)
    , softrobots::constraint::CableModel<DataTypes>(object)
    , d_desiredLength(initData(&d_desiredLength, "desiredLength", ""))
{
    setUpData();
}


template<class DataTypes>
CableEffector<DataTypes>::~CableEffector()
{
}

template<class DataTypes>
void CableEffector<DataTypes>::setUpData()
{
    // These data from CableModel have no sense for effector
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_eqDisplacement.setDisplayed(false);
    d_eqForce.setDisplayed(false);
}

template<class DataTypes>
void CableEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                      BaseVector *resV,
                                                      const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    d_cableLength.setValue(getCableLength(m_state->readPositions().ref()));
    Real desiredLength = getTarget(d_desiredLength.getValue(), d_cableLength.getValue());
    Real dfree = Jdx->element(0) + desiredLength - d_cableLength.getValue();
    resV->set(m_constraintIndex.getValue(), dfree);
}

} // namespace
