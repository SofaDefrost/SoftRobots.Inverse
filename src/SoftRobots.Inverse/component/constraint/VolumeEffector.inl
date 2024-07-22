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

#include <SoftRobots.Inverse/component/constraint/VolumeEffector.h>

namespace softrobotsinverse::constraint
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;

template<class DataTypes>
VolumeEffector<DataTypes>::VolumeEffector(MechanicalState* object)
    : Effector<DataTypes>(object)
    , softrobots::constraint::SurfacePressureModel<DataTypes>(object)
    , d_desiredVolume(initData(&d_desiredVolume, "desiredVolume",""))
{
    // These data from SurfacePressureModel have no sense for effector
    d_maxPressure.setDisplayed(false);
    d_minPressure.setDisplayed(false);
    d_maxPressureVariation.setDisplayed(false);
    d_maxVolumeGrowth.setDisplayed(false);
    d_minVolumeGrowth.setDisplayed(false);
    d_maxVolumeGrowthVariation.setDisplayed(false);
    d_eqPressure.setDisplayed(false);
    d_eqVolumeGrowth.setDisplayed(false);
}

template<class DataTypes>
VolumeEffector<DataTypes>::~VolumeEffector()
{
}


template<class DataTypes>
void VolumeEffector<DataTypes>::init()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::init();
    if (!d_desiredVolume.isSet())
        d_desiredVolume.setValue(d_initialCavityVolume.getValue()); //set the desired volume to the actual value as a sensible initial state
}

template<class DataTypes>
void VolumeEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                       BaseVector *resV,
                                                       const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    d_cavityVolume.setValue(getCavityVolume(m_state->readPositions().ref()));
    Real desiredVolume = getTarget(d_desiredVolume.getValue(), d_cavityVolume.getValue());
    Real dfree = Jdx->element(0) + d_cavityVolume.getValue() - desiredVolume;
    resV->set(d_constraintIndex.getValue(), dfree);
}

template<class DataTypes>
void VolumeEffector<DataTypes>::storeResults(sofa::type::vector<double> &delta)
{
    d_volumeGrowth.setValue(delta[0]);
}

} // namespace
