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

#include <SoftRobots.Inverse/component/constraint/CableActuator.h>

namespace softrobotsinverse::constraint
{

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;

template<class DataTypes>
CableActuator<DataTypes>::CableActuator(MechanicalState* object)
    : Actuator<DataTypes>(object)
    , softrobots::constraint::CableModel<DataTypes>(object)
    , d_initForce(initData(&d_initForce,Real(0.0), "initForce",
                          "Initial force if any. Default is 0."))

    , d_constrainAtTime(initData(&d_constrainAtTime,Real(0.0), "constrainAtTime",
                          "No constraints will be applied before this time. \n"
                          "Example of use: to allow the cable to reach an \n"
                          "initial configuration before optimizing."))

    , d_displayCableLimit(initData(&d_displayCableLimit,false, "displayCableLimit",
                          "Display cable in red when the limit set by user has been reach."))
{
    d_displayCableLimit.setGroup("Visualization");
    m_color = d_color.getValue();

    // These datas from CableModel have no sense for actuator
    d_eqForce.setDisplayed(false);
    d_eqDisplacement.setDisplayed(false);

    // QP on only one value, we set dimension to one
    m_lambdaInit.resize(1);
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}


template<class DataTypes>
CableActuator<DataTypes>::~CableActuator()
{
}

template<class DataTypes>
void CableActuator<DataTypes>::init()
{
    softrobots::constraint::CableModel<DataTypes>::init();
    initDatas();
    initLimit();
}


template<class DataTypes>
void CableActuator<DataTypes>::reinit()
{
    softrobots::constraint::CableModel<DataTypes>::reinit();
    initDatas();
    initLimit();
}

template<class DataTypes>
void CableActuator<DataTypes>::reset()
{
    initDatas();
    initLimit();
}

template<class DataTypes>
void CableActuator<DataTypes>::initDatas()
{
    d_displacement.setValue(0.0);
    d_force.setValue(d_initForce.getValue());
    if(d_initForce.isSet())
    {
        m_hasLambdaInit = true;
        m_lambdaInit[0] = d_initForce.getValue();
    }
}

template<class DataTypes>
void CableActuator<DataTypes>::initLimit()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    ReadAccessor<sofa::Data<double>> displacement = d_displacement;
    ReadAccessor<sofa::Data<Real>> maxDispVariation = d_maxDispVariation;
    ReadAccessor<sofa::Data<Real>> maxPositiveDisplacement = d_maxPositiveDisplacement;
    ReadAccessor<sofa::Data<Real>> maxNegativeDisplacement = d_maxNegativeDisplacement;
    ReadAccessor<sofa::Data<Real>> maxForce = d_maxForce;
    ReadAccessor<sofa::Data<Real>> minForce = d_minForce;

    if(d_maxPositiveDisplacement.isSet())
    {
        m_hasDeltaMax = true;
        m_deltaMax[0] = maxPositiveDisplacement;
    }

    if(d_maxNegativeDisplacement.isSet())
    {
        m_hasDeltaMin = true;
        m_deltaMin[0] = -maxNegativeDisplacement;
    }

    if(d_maxForce.isSet())
    {
        m_hasLambdaMax = true;
        m_lambdaMax[0] = maxForce;
    }

    if(d_minForce.isSet())
    {
        m_hasLambdaMin = true;
        m_lambdaMin[0] = minForce;
    }

    if(!m_hasLambdaMin || m_lambdaMin[0]<0)
        msg_info(this) << "By not setting minForce=0 you are considering the cable as a stiff rod able to push. ";

    if(d_maxDispVariation.isSet())
    {
        m_hasDeltaMax = true;
        m_hasDeltaMin = true;
        if(rabs(m_deltaMin[0] - displacement) >= maxDispVariation || !d_maxNegativeDisplacement.isSet())
            m_deltaMin[0] = displacement - maxDispVariation;
        if(rabs(m_deltaMax[0] - displacement) >= maxDispVariation || !d_maxPositiveDisplacement.isSet())
            m_deltaMax[0] = displacement + maxDispVariation;
    }
}


template<class DataTypes>
void CableActuator<DataTypes>::updateLimit()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    if(d_constrainAtTime.getValue() == time)
    {
        initLimit();
        return;
    }

    ReadAccessor<sofa::Data<double>> displacement = d_displacement;
    ReadAccessor<sofa::Data<Real>> maxDispVariation = d_maxDispVariation;
    ReadAccessor<sofa::Data<Real>> maxPositiveDisplacement = d_maxPositiveDisplacement;
    ReadAccessor<sofa::Data<Real>> maxNegativeDisplacement = d_maxNegativeDisplacement;

    if(d_maxPositiveDisplacement.isSet())
        m_deltaMax[0] = maxPositiveDisplacement;

    if(d_maxNegativeDisplacement.isSet())
        m_deltaMin[0] = -maxNegativeDisplacement;

    if(d_maxDispVariation.isSet())
    {
        if(rabs(m_deltaMin[0] - displacement) >= maxDispVariation || !d_maxNegativeDisplacement.isSet())
            m_deltaMin[0] = displacement - maxDispVariation;
        if(rabs(m_deltaMax[0] - displacement) >= maxDispVariation || !d_maxPositiveDisplacement.isSet())
            m_deltaMax[0] = displacement + maxDispVariation;
    }
}


template<class DataTypes>
void CableActuator<DataTypes>::updateVisualization()
{
    ReadAccessor<sofa::Data<double>> displacement = d_displacement;
    ReadAccessor<sofa::Data<double>> force = d_force;
    ReadAccessor<sofa::Data<Real>> maxPositiveDisplacement = d_maxPositiveDisplacement;
    ReadAccessor<sofa::Data<Real>> maxForce = d_maxForce;

    if(d_maxPositiveDisplacement.isSet())
    {
        if(rabs(displacement-maxPositiveDisplacement) < 1e-5)
            d_color.setValue(sofa::type::RGBAColor::red());
        else
            d_color.setValue(m_color);
    }

    if(d_maxForce.isSet())
    {
        if(rabs(force-maxForce) < 1e-5)
            d_color.setValue(sofa::type::RGBAColor::red());
        else
            d_color.setValue(m_color);
    }
}

template<class DataTypes>
void CableActuator<DataTypes>::storeResults(sofa::type::vector<double> &lambda,
                                            sofa::type::vector<double> &delta)
{
    d_force.setValue(lambda[0]);
    d_displacement.setValue(delta[0]);

    updateLimit();
    if(d_displayCableLimit.getValue())
        updateVisualization();

    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void CableActuator<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                           sofa::core::MultiVecDerivId res,
                                           const BaseVector* lambda)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(res);
    SOFA_UNUSED(lambda);

    // Do nothing, because the position of the mechanical state is not up to date when storeLambda() is called.
    // So if we compute delta from CableModel::storeLambda() we would be one step behind.
    // Instead the actual delta is stored in storeResults(), which is called from QPInverseProblemSolver
}


} // namespace
