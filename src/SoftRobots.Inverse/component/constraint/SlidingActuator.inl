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
#include <sofa/type/Vec.h>

#include <SoftRobots.Inverse/component/constraint/SlidingActuator.h>

namespace softrobotsinverse::constraint
{

using sofa::core::objectmodel::ComponentState ;

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;
using sofa::type::RGBAColor;
using sofa::type::Vec;

template<class DataTypes>
SlidingActuator<DataTypes>::SlidingActuator(MechanicalState* object)
    : Inherit(object)
    , d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement, "maxPositiveDisp",
                                         "Maximum displacement of the actuator in the given direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement, "maxNegativeDisp",
                                         "Maximum displacement of the actuator in the negative direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxDispVariation(initData(&d_maxDispVariation, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_maxForce(initData(&d_maxForce, "maxForce",
                          "Maximum force of the actuator. \n"
                          "If unspecified no maximum value will be considered."))

    , d_minForce(initData(&d_minForce, "minForce",
                          "Minimum force of the actuator. \n"
                          "If unspecified no minimum value will be considered."))

    , d_direction(initData(&d_direction, "direction",
                          "Direction of the actuation."))

    , d_indices(initData(&d_indices, "indices",
                          "Indices of the nodes subjected to the force. \n"
                          "If no indices given, mechanical context considered."))

    , d_initForce(initData(&d_initForce, double(0.0), "initForce",
                           "Initial force. Default is 0."))

    , d_force(initData(&d_force,double(0.0), "force",
                          "Output force. Warning: to get the actual force you should divide this value by dt."))

    , d_initDisplacement(initData(&d_initDisplacement, double(0.0), "initDisplacement",
                          "Initial displacement. Default is 0."))

    , d_displacement(initData(&d_displacement,double(0.0), "displacement",
                          "Output displacement compared to the initial position."))

    , d_accumulateDisp(initData(&d_accumulateDisp, false, "accumulateDisp", "In case of relative displacement, accumulate the displacement."))

    , d_showDirection(initData(&d_showDirection,false, "showDirection",
                          "Draw the direction."))

    , d_showVisuScale(initData(&d_showVisuScale,double(0.001), "showVisuScale",
                          "Visualization scale."))
{
    d_maxNegativeDisplacement.setGroup("Input");
    d_maxPositiveDisplacement.setGroup("Input");
    d_maxDispVariation.setGroup("Input");
    d_maxForce.setGroup("Input");
    d_minForce.setGroup("Input");

    d_direction.setGroup("Input");
    d_indices.setGroup("Input");

    d_initForce.setGroup("Input");
    d_initDisplacement.setGroup("Input");

    d_force.setGroup("Output");
    d_displacement.setGroup("Output");

    d_showDirection.setGroup("Visualization");
    d_showVisuScale.setGroup("Visualization");

    // QP on only one value, we set dimension to one
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}


template<class DataTypes>
SlidingActuator<DataTypes>::~SlidingActuator()
{
}

template<class DataTypes>
void SlidingActuator<DataTypes>::init()
{
    d_componentState = ComponentState::Invalid ;
    SoftRobotsConstraint<DataTypes>::init();

    if(m_state==nullptr)
    {
        msg_error(this) << "There is no mechanical state associated with this node. "
                            "the object is deactivated. "
                            "To remove this error message fix your scene possibly by "
                            "adding a MechanicalObject." ;
        return ;
    }

    if (!d_indices.isSet())
    {
        SetIndexArray &list = (*d_indices.beginEdit());
        msg_warning(this)<<"No index of actuation given, set default (all points of context MechanicalState).";

        for(unsigned int i=0; i<m_state->getSize(); i++)
            list.push_back(i);
        d_indices.endEdit();
    }
    else
        checkIndicesRegardingState();
    
    initData();
    initLimit();

    d_componentState = ComponentState::Valid ;
}

template<class DataTypes>
void SlidingActuator<DataTypes>::bwdInit()
{
}

template<class DataTypes>
void SlidingActuator<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    checkIndicesRegardingState();
    initData();
    initLimit();
}

template<class DataTypes>
void SlidingActuator<DataTypes>::reset()
{
    reinit();
}


// Rigid implementation in SlidingActuator.cpp
template<class DataTypes>
void SlidingActuator<DataTypes>::initData()
{
    if (!d_direction.isSet())
    {
        msg_warning()<<"No direction of actuation provided by user. Default (1. 0. 0.)";
        Deriv x;
        x[0]=1;
        d_direction.setValue(x);
    }
    else
    {
        Deriv direction = d_direction.getValue();
        direction.normalize();
        d_direction.setValue(direction);
    }

    d_displacement.setValue(d_initDisplacement.getValue());
    d_force.setValue(d_initForce.getValue());
}


template<class DataTypes>
void SlidingActuator<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();

    for(unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error(this) << "Indices at index " << i << " is too large regarding mechanicalState [position] size" ;
    }
}

template<class DataTypes>
void SlidingActuator<DataTypes>::initLimit()
{
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
void SlidingActuator<DataTypes>::updateLimit()
{
    ReadAccessor<sofa::Data<double>> displacement = d_displacement;
    ReadAccessor<sofa::Data<Real>> maxDispVariation = d_maxDispVariation;
    ReadAccessor<sofa::Data<Real>> maxPositiveDisplacement = d_maxPositiveDisplacement;
    ReadAccessor<sofa::Data<Real>> maxNegativeDisplacement = d_maxNegativeDisplacement;

    if(d_maxPositiveDisplacement.isSet())
    {
        if (d_accumulateDisp.getValue() && displacement > 0)
            m_deltaMax[0] = maxPositiveDisplacement - displacement;
        else
            m_deltaMax[0] = maxPositiveDisplacement;
    }

    if(d_maxNegativeDisplacement.isSet())
    {
        if (d_accumulateDisp.getValue() && displacement < 0)
            m_deltaMin[0] = -maxNegativeDisplacement - displacement;
        else
            m_deltaMin[0] = -maxNegativeDisplacement;
    }

    if(d_maxDispVariation.isSet())
    {
        if (d_accumulateDisp.getValue())
        {
            if (m_deltaMin[0] < -maxDispVariation)
                m_deltaMin[0] = -maxDispVariation;
            if (m_deltaMax[0] > maxDispVariation)
                m_deltaMax[0] = maxDispVariation;
        }
        else
        {
            if(rabs(m_deltaMin[0] - displacement) >= maxDispVariation || !d_maxNegativeDisplacement.isSet())
                m_deltaMin[0] = displacement - maxDispVariation;
            if(rabs(m_deltaMax[0] - displacement) >= maxDispVariation || !d_maxPositiveDisplacement.isSet())
                m_deltaMax[0] = displacement + maxDispVariation;
        }
    }
}


template<class DataTypes>
void SlidingActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                       DataMatrixDeriv &cMatrix,
                                                       unsigned int &cIndex,
                                                       const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    m_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    
    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

    Deriv direction = d_direction.getValue();

    ReadAccessor<sofa::Data<SetIndexArray>> indices = d_indices;

    auto ratio = direction/indices.size();
    for(unsigned int indice : indices)
    {
        rowIterator.setCol(indice, ratio);
    }

    cIndex++;
    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
}


template<class DataTypes>
void SlidingActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                        BaseVector *resV,
                                                        const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    ReadAccessor<sofa::Data<VecCoord> > restPositions = m_state->readRestPositions();
    const SetIndexArray &indices       = d_indices.getValue();
    const Deriv         &direction     = d_direction.getValue();

    // Projection of the global displacement along the direction (normalized) of the actuation
    Deriv d = DataTypes::coordDifference(positions[indices[0]],restPositions[indices[0]]);
    const int size = Deriv::total_size;

    Real dFree = Jdx->element(0);

    for (unsigned int i=0; i<size; i++){
        dFree+= d[i]*direction[i];
    }

    if(indices.size())
    {
        resV->set(m_constraintIndex.getValue(), dFree);
    }
}

template<class DataTypes>
void SlidingActuator<DataTypes>::storeResults(sofa::type::vector<double> &lambda,
                                              sofa::type::vector<double> &delta)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;
    d_force.setValue(lambda[0]);

    double& displacement = sofa::helper::getWriteAccessor(d_displacement);
    if(d_accumulateDisp.getValue())
    {
        displacement += delta[0];
    }
    else
    {
        displacement = delta[0];
    }

    updateLimit();

    Actuator<DataTypes>::storeResults(lambda, delta);
}

template<class DataTypes>
void SlidingActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    if (!vparams->displayFlags().getShowInteractionForceFields()) return;
    if (!d_showDirection.getValue()) return;

    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    const SetIndexArray &indices = d_indices.getValue();

    Vec<3,SReal> bary(0.,0.,0.);
    for(unsigned int i=0; i<indices.size(); i++)
        for (unsigned int j=0; j<3; j++)
            bary[j]+=positions[indices[i]][j]/indices.size();

    Vec<3,SReal> baryArrow(0.,0.,0.);
    for (unsigned int j=0; j<3; j++)
        baryArrow[j] = bary[j]+d_direction.getValue()[j]*d_showVisuScale.getValue();

    vparams->drawTool()->setLightingEnabled(true);
    vparams->drawTool()->drawArrow(bary,baryArrow,d_showVisuScale.getValue()/20.0f, RGBAColor(1,0,0,1));
    vparams->drawTool()->restoreLastState();
}

} // namespace

