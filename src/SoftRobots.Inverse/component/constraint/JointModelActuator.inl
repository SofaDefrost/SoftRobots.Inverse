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

#include <SoftRobots.Inverse/component/constraint/JointModelActuator.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/type/RGBAColor.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::type::RGBAColor;

template<class DataTypes>
JointModelActuator<DataTypes>::JointModelActuator(MechanicalState* object)
    : Actuator<DataTypes>(object)
    , softrobots::constraint::JointModel<DataTypes>(object)
    , d_initAngle(initData(&d_initAngle, Real(0.0), "initAngle", "Initial joint angle/position for reference"))
    , d_constrainAtTime(initData(&d_constrainAtTime, Real(0.0), "constrainAtTime", "Time at which the constraint becomes active"))
    , d_displayJointLimit(initData(&d_displayJointLimit, false, "displayJointLimit", "Display joint limits in visualization"))
{
    initData();
}

template<class DataTypes>
JointModelActuator<DataTypes>::~JointModelActuator()
{
}

template<class DataTypes>
void JointModelActuator<DataTypes>::init()
{
    Actuator<DataTypes>::init();
    softrobots::constraint::JointModel<DataTypes>::init();
    
    initLimit();
}

template<class DataTypes>
void JointModelActuator<DataTypes>::reinit()
{
    softrobots::constraint::JointModel<DataTypes>::reinit();
    updateLimit();
}

template<class DataTypes>
void JointModelActuator<DataTypes>::reset()
{
    softrobots::constraint::JointModel<DataTypes>::reset();
}

template<class DataTypes>
void JointModelActuator<DataTypes>::initData()
{
    // Set constraint type as ACTUATOR
    this->m_constraintType = softrobots::behavior::SoftRobotsBaseConstraint::ACTUATOR;
}

template<class DataTypes>
void JointModelActuator<DataTypes>::initLimit()
{
    // Initialize limits for the inverse problem solver
    
    // Delta limits (displacement/rotation limits)
    if (d_maxDisplacement.getValue() != Real(1e12))
    {
        m_hasDeltaMax = true;
        m_deltaMax.clear();
        m_deltaMax.push_back(static_cast<double>(d_maxDisplacement.getValue()));
    }
    
    if (d_minDisplacement.getValue() != Real(-1e12))
    {
        m_hasDeltaMin = true;
        m_deltaMin.clear();
        m_deltaMin.push_back(static_cast<double>(d_minDisplacement.getValue()));
    }
    
    // Lambda limits (force/torque limits)
    if (d_maxForce.getValue() != Real(1e12))
    {
        m_hasLambdaMax = true;
        m_lambdaMax.clear();
        m_lambdaMax.push_back(static_cast<double>(d_maxForce.getValue()));
    }
    
    if (d_minForce.getValue() != Real(-1e12))
    {
        m_hasLambdaMin = true;
        m_lambdaMin.clear();
        m_lambdaMin.push_back(static_cast<double>(d_minForce.getValue()));
    }
    
    // Initial lambda value if specified
    if (d_force.getValue() != 0.0)
    {
        m_hasLambdaInit = true;
        m_lambdaInit.clear();
        m_lambdaInit.push_back(d_force.getValue());
    }
}

template<class DataTypes>
void JointModelActuator<DataTypes>::updateLimit()
{
    initLimit(); // Simply reinitialize limits
}

template<class DataTypes>
void JointModelActuator<DataTypes>::updateVisualization()
{
    // Update color based on current force
    double normalizedForce = 0.0;
    double maxForce = static_cast<double>(d_maxForce.getValue());
    
    if (maxForce > 0.0)
    {
        normalizedForce = std::abs(d_force.getValue()) / maxForce;
        normalizedForce = std::min(normalizedForce, 1.0);
    }
    
    // Color interpolation from blue (no force) to red (max force)
    m_color = RGBAColor(static_cast<float>(normalizedForce), 
                       0.35f, 
                       static_cast<float>(1.0 - normalizedForce), 
                       1.0f);
                       
    d_color.setValue(m_color);
}

template<class DataTypes>
void JointModelActuator<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                               sofa::core::MultiVecDerivId res,
                                               const sofa::linearalgebra::BaseVector* lambda)
{
    softrobots::constraint::JointModel<DataTypes>::storeLambda(cParams, res, lambda);
    updateVisualization();
}

template<class DataTypes>
void JointModelActuator<DataTypes>::storeResults(sofa::type::vector<double> &lambda,
                                                sofa::type::vector<double> &delta)
{
    // Store results from the QP solver
    
    if (lambda.size() >= this->m_nbLines && delta.size() >= this->m_nbLines)
    {
        // For single DOF joints (revolute, prismatic), store single value
        if (this->m_nbLines == 1)
        {
            d_force.setValue(lambda[0]);
            d_displacement.setValue(delta[0]);
        }
        else if (this->m_nbLines > 1)
        {
            // For multi-DOF joints (spherical, etc.), store first component
            // More sophisticated handling could be implemented for multi-DOF cases
            d_force.setValue(lambda[0]);
            d_displacement.setValue(delta[0]);
            
            // Log info about multi-DOF case
            msg_info() << "JointModelActuator: Storing results for " << this->m_nbLines 
                       << " DOF joint. Only first component displayed in d_force/d_displacement.";
        }
    }
    else
    {
        msg_warning() << "JointModelActuator: Insufficient data in lambda (" << lambda.size() 
                      << ") or delta (" << delta.size() << ") vectors. Expected at least " 
                      << this->m_nbLines << " elements.";
    }
    
    updateVisualization();
}

} // namespace softrobotsinverse::constraint