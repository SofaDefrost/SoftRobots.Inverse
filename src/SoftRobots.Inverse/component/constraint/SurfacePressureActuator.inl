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

#include <SoftRobots.Inverse/component/constraint/SurfacePressureActuator.h>

#include <sofa/helper/logging/Messaging.h>

namespace softrobotsinverse::constraint
{

using sofa::type::Vec3d;
using sofa::type::vector;
using sofa::helper::rabs;

template<class DataTypes>
SurfacePressureActuator<DataTypes>::SurfacePressureActuator(MechanicalState* object)
    : Actuator<DataTypes>(object)
    , softrobots::constraint::SurfacePressureModel<DataTypes>(object)
    , d_initPressure(initData(&d_initPressure,Real(0.0), "initPressure",
                          "Initial pressure if any. Default is 0."))
{
    // These data from SurfacePressureModel have no sense for actuator
    d_eqPressure.setDisplayed(false);
    d_eqVolumeGrowth.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaInit.resize(1);
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}

template<class DataTypes>
SurfacePressureActuator<DataTypes>::~SurfacePressureActuator()
{
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::init()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::init();
    initData();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::reinit()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::reinit();
    initData();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::reset()
{
    softrobots::constraint::SurfacePressureModel<DataTypes>::reset();
    initData();
    initLimits();
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::initData()
{
    d_volumeGrowth.setValue(0.0);
    d_pressure.setValue(d_initPressure.getValue());

    if(d_initPressure.isSet()){
        m_hasLambdaInit = true;
        m_lambdaInit[0] = d_initPressure.getValue();
    }
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::initLimits()
{
    sofa::helper::ReadAccessor<sofa::Data<Real>> maxVolumeGrowthVariation = d_maxVolumeGrowthVariation;
    sofa::helper::ReadAccessor<sofa::Data<Real>> maxVolumeGrowth = d_maxVolumeGrowth;
    sofa::helper::ReadAccessor<sofa::Data<Real>> minVolumeGrowth = d_minVolumeGrowth;
    sofa::helper::ReadAccessor<sofa::Data<Real>> maxPressure = d_maxPressure;
    sofa::helper::ReadAccessor<sofa::Data<Real>> minPressure = d_minPressure;

    if(d_maxPressure.isSet())
    {
        m_hasLambdaMax = true;
        m_lambdaMax[0] = maxPressure;
    }

    if(d_minPressure.isSet())
    {
        m_hasLambdaMin = true;
        m_lambdaMin[0] = minPressure;
    }

    if(!m_hasLambdaMin || m_lambdaMin[0]<0)
        msg_info() << "A negative pressure will empty/drain the cavity. If you do not want this feature set 'minPressure=0'.";

    if(d_maxVolumeGrowth.isSet())
    {
        m_hasDeltaMax = true;
        m_deltaMax[0] = maxVolumeGrowth;
    }

    if(d_minVolumeGrowth.isSet())
    {
        m_hasDeltaMin = true;
        m_deltaMin[0] = minVolumeGrowth;
    }

    if (d_maxVolumeGrowthVariation.isSet())
    {
        m_hasDeltaMax = true;
        m_hasDeltaMin = true;
        if (m_deltaMax[0] >= maxVolumeGrowthVariation || !d_maxVolumeGrowth.isSet())
            m_deltaMax[0] = maxVolumeGrowthVariation;
        if (m_deltaMin[0] <= -maxVolumeGrowthVariation || !d_minVolumeGrowth.isSet())
            m_deltaMin[0] = -maxVolumeGrowthVariation;
    }
}


template<class DataTypes>
void SurfacePressureActuator<DataTypes>::updateLimits()
{
    sofa::helper::ReadAccessor<sofa::Data<double>> volumeGrowth = d_volumeGrowth;
    sofa::helper::ReadAccessor<sofa::Data<Real>> maxVolumeGrowthVariation = d_maxVolumeGrowthVariation;
    sofa::helper::ReadAccessor<sofa::Data<Real>> maxVolumeGrowth = d_maxVolumeGrowth;
    sofa::helper::ReadAccessor<sofa::Data<Real>> minVolumeGrowth = d_minVolumeGrowth;

    if(d_maxVolumeGrowth.isSet())
        m_deltaMax[0] = maxVolumeGrowth;

    if(d_minVolumeGrowth.isSet())
        m_deltaMin[0] = minVolumeGrowth;

    if(d_maxVolumeGrowthVariation.isSet())
    {
        if(rabs(m_deltaMax[0] - volumeGrowth) >= maxVolumeGrowthVariation || !d_maxVolumeGrowth.isSet())
            m_deltaMax[0] = volumeGrowth + maxVolumeGrowthVariation;
        if(rabs(m_deltaMin[0] + volumeGrowth) <= -maxVolumeGrowthVariation || !d_minVolumeGrowth.isSet())
            m_deltaMin[0] = volumeGrowth - maxVolumeGrowthVariation;
    }
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::storeResults(sofa::type::vector<double> &lambda,
                                                      sofa::type::vector<double> &delta)
{
    d_pressure.setValue(lambda[0]);
    d_volumeGrowth.setValue(delta[0]);

    updateLimits();

    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void SurfacePressureActuator<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                       sofa::core::MultiVecDerivId res,
                                                       const BaseVector* lambda)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(res);
    SOFA_UNUSED(lambda);

    // Do nothing, because the position of the mechanical state is not up to date when storeLambda() is called.
    // So if we compute delta from SurfacePressureModel::storeLambda() we would be one step behind.
    // Instead the actual delta is stored in storeResults(), which is called from QPInverseProblemSolver
}


} // namespace

