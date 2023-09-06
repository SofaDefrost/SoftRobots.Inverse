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

#include <SoftRobots.Inverse/component/constraint/JointActuator.h>
#include <sofa/core/visual/VisualParams.h>

namespace softrobotsinverse::constraint
{
using sofa::core::objectmodel::ComponentState;
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::linearalgebra::BaseVector;
using sofa::type::Mat;
using sofa::core::VecCoordId ;
using sofa::type::vector;
using sofa::type::Vec3;
using sofa::helper::rabs;

template<class DataTypes>
JointActuator<DataTypes>::JointActuator(MechanicalState* object)
    : Inherit1(object)

    , d_index(initData(&d_index, "index", "Index of the point of the model on which we want to apply the effort"))

    , d_initEffort(initData(&d_initEffort, Real(0.0), "initEffort", "Initial effort value."))

    , d_initAngle(initData(&d_initAngle, Real(0.0), "initAngle", "Initial angle value."))

    , d_maxEffort(initData(&d_maxEffort, "maxEffort", ""))

    , d_minEffort(initData(&d_minEffort, "minEffort", ""))

    , d_maxEffortVariation(initData(&d_maxEffortVariation, "maxEffortVariation", ""))

    , d_maxAngle(initData(&d_maxAngle, "maxAngle", "In radian"))

    , d_minAngle(initData(&d_minAngle, "minAngle", "In radian"))

    , d_maxAngleVariation(initData(&d_maxAngleVariation, "maxAngleVariation", "In radian"))

    , d_effort(initData(&d_effort, "effort", "Warning: to get the actual effort you should divide this value by dt."))

    , d_angle(initData(&d_angle, "angle", ""))
{
    setUpData();

    // QP on only one value, we set dimension to one
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}


template<class DataTypes>
void JointActuator<DataTypes>::setUpData()
{
    d_effort.setReadOnly(true);
    d_angle.setReadOnly(true);
}


template<class DataTypes>
JointActuator<DataTypes>::~JointActuator()
{
}


template<class DataTypes>
void JointActuator<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    Inherit1::init();

    if(m_state==nullptr){
        msg_error() << "There is no mechanical state associated with this node. "
                       "the object is deactivated. "
                       "To remove this error message fix your scene possibly by "
                       "adding a MechanicalObject." ;
        return;
    }

    initDatas();
    initLimit();
    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void JointActuator<DataTypes>::reinit()
{
    initDatas();
    initLimit();
}


template<class DataTypes>
void JointActuator<DataTypes>::initLimit()
{
    if(d_maxEffort.isSet())
    {
        m_hasLambdaMax = true;
        m_lambdaMax[0] = d_maxEffort.getValue();
    }

    if(d_minEffort.isSet())
    {
        m_hasLambdaMin = true;
        m_lambdaMin[0] = d_minEffort.getValue();
    }

    if(d_maxAngle.isSet())
    {
        m_hasDeltaMax = true;
        m_deltaMax[0] = d_maxAngle.getValue();
    }

    if(d_minAngle.isSet())
    {
        m_hasDeltaMin = true;
        m_deltaMin[0] = d_minAngle.getValue();
    }

    if(d_maxEffortVariation.isSet())
    {
        m_hasLambdaMax = true;
        m_hasLambdaMin = true;
        if(rabs(m_lambdaMin[0] - d_effort.getValue()) >= d_maxEffortVariation.getValue() || !d_minEffort.isSet())
            m_lambdaMin[0] = d_effort.getValue() - d_maxEffortVariation.getValue();
        if(rabs(m_lambdaMax[0] - d_effort.getValue()) >= d_maxEffortVariation.getValue() || !d_maxEffort.isSet())
            m_lambdaMax[0] = d_effort.getValue() + d_maxEffortVariation.getValue();
    }

    if(d_maxAngleVariation.isSet())
    {
        m_hasDeltaMax = true;
        m_hasDeltaMin = true;
        if(rabs(m_deltaMin[0] - d_angle.getValue()) >= d_maxAngleVariation.getValue() || !d_minAngle.isSet())
            m_deltaMin[0] = d_angle.getValue() - d_maxAngleVariation.getValue();
        if(rabs(m_deltaMax[0] - d_angle.getValue()) >= d_maxAngleVariation.getValue() || !d_maxAngle.isSet())
            m_deltaMax[0] = d_angle.getValue() + d_maxAngleVariation.getValue();
    }
}


template<class DataTypes>
void JointActuator<DataTypes>::initDatas()
{
    d_effort.setValue(d_initEffort.getValue());
    d_angle.setValue(d_initAngle.getValue());
}


template<class DataTypes>
void JointActuator<DataTypes>::updateLimit()
{
    if(d_maxEffort.isSet())
        m_lambdaMax[0] = d_maxEffort.getValue();

    if(d_minEffort.isSet())
        m_lambdaMin[0] = d_minEffort.getValue();

    if(d_maxAngle.isSet())
        m_deltaMax[0] = d_maxAngle.getValue();

    if(d_minAngle.isSet())
        m_deltaMin[0] = d_minAngle.getValue();

    if(d_maxEffortVariation.isSet())
    {
        if(rabs(m_lambdaMin[0] - d_effort.getValue()) >= d_maxEffortVariation.getValue() || !d_minEffort.isSet())
            m_lambdaMin[0] = d_effort.getValue() - d_maxEffortVariation.getValue();
        if(rabs(m_lambdaMax[0] - d_effort.getValue()) >= d_maxEffortVariation.getValue() || !d_maxEffort.isSet())
            m_lambdaMax[0] = d_effort.getValue() + d_maxEffortVariation.getValue();
    }

    if(d_maxAngleVariation.isSet())
    {
        if(rabs(m_deltaMin[0] - d_angle.getValue()) >= d_maxAngleVariation.getValue() || !d_minAngle.isSet())
            m_deltaMin[0] = d_angle.getValue() - d_maxAngleVariation.getValue();
        if(rabs(m_deltaMax[0] - d_angle.getValue()) >= d_maxAngleVariation.getValue() || !d_maxAngle.isSet())
            m_deltaMax[0] = d_angle.getValue() + d_maxAngleVariation.getValue();
    }
}


template<class DataTypes>
void JointActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                          DataMatrixDeriv &cMatrix,
                                                          unsigned int &cIndex,
                                                          const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    if(!this->isComponentStateValid())
        return ;

    m_constraintId = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_constraintId);
    rowIterator.addCol(d_index.getValue(), Deriv(1.));
    cIndex++;

    cMatrix.endEdit();

    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void JointActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                       BaseVector *resV,
                                                       const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    if(!this->isComponentStateValid())
        return ;

    Real dFree = Jdx->element(0) - d_initAngle.getValue() + d_angle.getValue();
    resV->set(m_constraintId, dFree);
}


template<class DataTypes>
void JointActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    if(!this->isComponentStateValid())
        return ;

    d_effort.setValue(lambda[0]);
    d_angle.setValue(delta[0]);
    updateLimit();

    Actuator<DataTypes>::storeResults(lambda, delta);
}


} // namespace

