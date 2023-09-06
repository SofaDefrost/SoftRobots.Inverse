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

#include <SoftRobots.Inverse/component/constraint/SurfacePressureEquality.h>

namespace softrobotsinverse::constraint
{

using sofa::type::Vec3d;
using sofa::type::vector;
using sofa::helper::rabs;

template<class DataTypes>
SurfacePressureEquality<DataTypes>::SurfacePressureEquality(MechanicalState* object)
    : Equality<DataTypes>(object)
    , softrobots::constraint::SurfacePressureModel<DataTypes>(object)
{
    d_maxVolumeGrowthVariation.setDisplayed(false);
    d_maxVolumeGrowth.setDisplayed(false);
    d_minVolumeGrowth.setDisplayed(false);
    d_maxPressure.setDisplayed(false);
    d_minPressure.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaEqual.resize(1);
    m_deltaEqual.resize(1);
}

template<class DataTypes>
SurfacePressureEquality<DataTypes>::~SurfacePressureEquality()
{
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::init()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::init();
    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::reinit()
{
    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::reset()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::reset();
    d_volumeGrowth.setValue(0.0);
    d_pressure.setValue(0.0);

    updateConstraint();
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::updateConstraint()
{
    sofa::helper::ReadAccessor<sofa::Data<Real>> eqPressure = d_eqPressure;
    sofa::helper::ReadAccessor<sofa::Data<Real>> eqVolumeGrowth = d_eqVolumeGrowth;

    if(d_eqPressure.isSet())
    {
        m_hasLambdaEqual= true;
        m_lambdaEqual[0] = eqPressure;
    }

    if(d_eqVolumeGrowth.isSet())
    {
        m_hasDeltaEqual = true;
        m_deltaEqual[0] = eqVolumeGrowth;
    }
}

template<class DataTypes>
void SurfacePressureEquality<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                     sofa::core::MultiVecDerivId res,
                                                     const BaseVector* lambda)
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::storeLambda(cParams,res,lambda);
    updateConstraint();
}


} // namespace

