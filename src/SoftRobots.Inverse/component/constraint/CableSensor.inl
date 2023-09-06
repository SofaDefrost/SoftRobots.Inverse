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

#include <SoftRobots.Inverse/component/constraint/CableSensor.h>

namespace softrobotsinverse::constraint
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::type::vector;

template<class DataTypes>
CableSensor<DataTypes>::CableSensor(MechanicalState* object)
    : softrobotsinverse::behavior::Sensor<DataTypes>(object)
    , softrobots::constraint::CableModel<DataTypes>(object)
{
    setUpData();
}


template<class DataTypes>
CableSensor<DataTypes>::~CableSensor()
{
}


template<class DataTypes>
void CableSensor<DataTypes>::setUpData()
{
    // These datas from CableModel have no sense for sensor
    d_maxForce.setDisplayed(false);
    d_minForce.setDisplayed(false);
    d_eqForce.setDisplayed(false);
    d_force.setDisplayed(false);
    d_maxPositiveDisplacement.setDisplayed(false);
    d_maxNegativeDisplacement.setDisplayed(false);
    d_maxDispVariation.setDisplayed(false);
    d_eqDisplacement.setDisplayed(false);
    d_displacement.setDisplayed(false);
}


} // namespace
