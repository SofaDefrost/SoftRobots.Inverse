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

#include <SoftRobots.Inverse/component/constraint/JointModelEffector.h>
#include <sofa/helper/logging/Messaging.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;

template<class DataTypes>
JointModelEffector<DataTypes>::JointModelEffector(MechanicalState* object)
    : softrobotsinverse::behavior::Effector<DataTypes>(object)
    , softrobots::constraint::JointModel<DataTypes>(object)
    , d_desiredAngle(initData(&d_desiredAngle, Real(0.0), "desiredAngle", "Desired joint angle"))
    , d_desiredPosition(initData(&d_desiredPosition, Real(0.0), "desiredPosition", "Desired joint position"))
    , d_desiredAngles3D(initData(&d_desiredAngles3D, sofa::type::Vec3(0.0, 0.0, 0.0), "desiredAngles3D", "Desired joint angles (3D)"))
{
    setUpData();
}

template<class DataTypes>
JointModelEffector<DataTypes>::~JointModelEffector()
{
}

template<class DataTypes>
void JointModelEffector<DataTypes>::init()
{
    softrobotsinverse::behavior::Effector<DataTypes>::init();
    softrobots::constraint::JointModel<DataTypes>::init();
}

template<class DataTypes>
void JointModelEffector<DataTypes>::reinit()
{
    softrobots::constraint::JointModel<DataTypes>::reinit();
}

template<class DataTypes>
void JointModelEffector<DataTypes>::reset()
{
    softrobots::constraint::JointModel<DataTypes>::reset();
}

template<class DataTypes>
void JointModelEffector<DataTypes>::setUpData()
{
    // Set constraint type as EFFECTOR
    this->m_constraintType = softrobots::behavior::SoftRobotsBaseConstraint::EFFECTOR;
}

template<class DataTypes>
void JointModelEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                          sofa::linearalgebra::BaseVector *resV,
                                                          const sofa::linearalgebra::BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(Jdx);
    
    ReadAccessor<typename softrobots::constraint::JointModel<DataTypes>::DataVecCoord> positions = this->m_state->readPositions();
    
    // Get joint type using the base class method
    typename softrobots::constraint::JointModel<DataTypes>::JointType jointType = this->getJointTypeFromData();
    unsigned int cIndex = this->d_constraintIndex.getValue();
    
    switch(jointType)
    {
        case softrobots::constraint::JointModel<DataTypes>::REVOLUTE:
        {
            Real currentAngle = this->computeJointAngle(positions.ref());
            Real targetAngle = getTargetAngle();
            Real violation = currentAngle - targetAngle;
            resV->set(cIndex, violation);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::PRISMATIC:
        {
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real targetPosition = getTargetPosition();
            Real violation = currentPosition - targetPosition;
            resV->set(cIndex, violation);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::SPHERICAL:
        {
            sofa::type::Vec3 currentAngles = this->computeJointAngles3D(positions.ref());
            sofa::type::Vec3 targetAngles = getTargetAngles3D();
            for (unsigned int i = 0; i < 3; i++)
            {
                resV->set(cIndex + i, currentAngles[i] - targetAngles[i]);
            }
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::CYLINDRICAL:
        {
            // For cylindrical joints, we have both rotation and translation
            Real currentAngle = this->computeJointAngle(positions.ref());
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real targetAngle = getTargetAngle();
            Real targetPosition = getTargetPosition();
            
            resV->set(cIndex, currentAngle - targetAngle);
            resV->set(cIndex + 1, currentPosition - targetPosition);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::UNIVERSAL:
        {
            // For universal joints, we have two rotations
            sofa::type::Vec3 currentAngles = this->computeJointAngles3D(positions.ref());
            sofa::type::Vec3 targetAngles = getTargetAngles3D();
            
            resV->set(cIndex, currentAngles[0] - targetAngles[0]);
            resV->set(cIndex + 1, currentAngles[1] - targetAngles[1]);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::PLANAR:
        {
            // For planar joints, we have 2 translations + 1 rotation
            Real currentAngle = this->computeJointAngle(positions.ref());
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real targetAngle = getTargetAngle();
            Real targetPosition = getTargetPosition();
            
            // Simplified implementation - more sophisticated planar joint handling needed
            resV->set(cIndex, currentPosition - targetPosition); // Translation X
            resV->set(cIndex + 1, 0.0);                         // Translation Y (placeholder)
            resV->set(cIndex + 2, currentAngle - targetAngle);  // Rotation
            break;
        }
        default:
            msg_warning() << "JointModelEffector: Unsupported joint type.";
            break;
    }
}

template<class DataTypes>
typename JointModelEffector<DataTypes>::Real JointModelEffector<DataTypes>::getTargetAngle()
{
    // Use Effector's getTarget method to get progressive target
    Real currentAngle = d_desiredAngle.getValue();
    Real currentJointAngle = this->d_jointAngle.getValue();
    return this->getTarget(currentAngle, currentJointAngle);
}

template<class DataTypes>
typename JointModelEffector<DataTypes>::Real JointModelEffector<DataTypes>::getTargetPosition()
{
    // Use Effector's getTarget method to get progressive target
    Real currentPosition = d_desiredPosition.getValue();
    Real currentJointPosition = this->d_jointPosition1D.getValue();
    return this->getTarget(currentPosition, currentJointPosition);
}

template<class DataTypes>
sofa::type::Vec3 JointModelEffector<DataTypes>::getTargetAngles3D()
{
    // For 3D angles, apply getTarget to each component
    sofa::type::Vec3 desired = d_desiredAngles3D.getValue();
    sofa::type::Vec3 current = this->d_jointAngles3D.getValue();
    
    sofa::type::Vec3 target;
    target[0] = this->getTarget(desired[0], current[0]);
    target[1] = this->getTarget(desired[1], current[1]);
    target[2] = this->getTarget(desired[2], current[2]);
    
    return target;
}

} // namespace softrobotsinverse::constraint