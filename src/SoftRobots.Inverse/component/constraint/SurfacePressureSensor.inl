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

#include <SoftRobots.Inverse/component/constraint/SurfacePressureSensor.h>

namespace softrobotsinverse::constraint
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::type::vector;


DoNothingConstraintResolution::DoNothingConstraintResolution()
    : ConstraintResolution(1)
{
}


void DoNothingConstraintResolution::init(int line, double** w, double *lambda)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(lambda);
}
void DoNothingConstraintResolution::resolution(int line, double** w, double* d, double* lambda, double* dfree)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(dfree);
}


template<class DataTypes>
SurfacePressureSensor<DataTypes>::SurfacePressureSensor(MechanicalState* object)
    : Sensor<DataTypes>(object)
    , softrobots::constraint::SurfacePressureModel<DataTypes>(object)
{
    // These datas from SurfacePressureModel have no sense for sensor
    d_eqPressure.setDisplayed(false);
    d_eqVolumeGrowth.setDisplayed(false);
    d_minPressure.setDisplayed(false);
    d_maxPressure.setDisplayed(false);
    d_maxPressureVariation.setDisplayed(false);
    d_minVolumeGrowth.setDisplayed(false);
    d_maxVolumeGrowth.setDisplayed(false);
    d_maxVolumeGrowthVariation.setDisplayed(false);
}

template<class DataTypes>
SurfacePressureSensor<DataTypes>::~SurfacePressureSensor()
{
}

template<class DataTypes>
void SurfacePressureSensor<DataTypes>::getConstraintResolution(std::vector<ConstraintResolution*>& resTab,unsigned int& offset)
{
    DoNothingConstraintResolution *cr=  new DoNothingConstraintResolution();
    resTab[offset++] = cr;
}


} // namespace

