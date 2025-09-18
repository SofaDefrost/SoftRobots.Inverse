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

#include <SoftRobots.Inverse/component/constraint/JointModelEquality.h>
#include <sofa/helper/logging/Messaging.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;

template<class DataTypes>
JointModelEquality<DataTypes>::JointModelEquality(MechanicalState* object)
    : softrobots::constraint::JointModel<DataTypes>(object)
    , d_fixedAngle(initData(&d_fixedAngle, Real(0.0), "fixedAngle", "Fixed joint angle"))
    , d_fixedPosition(initData(&d_fixedPosition, Real(0.0), "fixedPosition", "Fixed joint position"))
    , d_fixedAngles3D(initData(&d_fixedAngles3D, sofa::type::Vec3(0.0, 0.0, 0.0), "fixedAngles3D", "Fixed joint angles (3D)"))
{
    setUpData();
}

template<class DataTypes>
JointModelEquality<DataTypes>::~JointModelEquality()
{
}

template<class DataTypes>
void JointModelEquality<DataTypes>::init()
{
    softrobots::constraint::JointModel<DataTypes>::init();
}

template<class DataTypes>
void JointModelEquality<DataTypes>::reinit()
{
    softrobots::constraint::JointModel<DataTypes>::reinit();
}

template<class DataTypes>
void JointModelEquality<DataTypes>::reset()
{
    softrobots::constraint::JointModel<DataTypes>::reset();
}

template<class DataTypes>
void JointModelEquality<DataTypes>::setUpData()
{
    // Set constraint type as EQUALITY
    this->m_constraintType = softrobots::behavior::SoftRobotsBaseConstraint::EQUALITY;
}

template<class DataTypes>
void JointModelEquality<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
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
            Real fixedAngle = d_fixedAngle.getValue();
            Real violation = currentAngle - fixedAngle;
            resV->set(cIndex, violation);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::PRISMATIC:
        {
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real fixedPosition = d_fixedPosition.getValue();
            Real violation = currentPosition - fixedPosition;
            resV->set(cIndex, violation);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::SPHERICAL:
        {
            sofa::type::Vec3 currentAngles = this->computeJointAngles3D(positions.ref());
            sofa::type::Vec3 fixedAngles = d_fixedAngles3D.getValue();
            for (unsigned int i = 0; i < 3; i++)
            {
                resV->set(cIndex + i, currentAngles[i] - fixedAngles[i]);
            }
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::CYLINDRICAL:
        {
            // For cylindrical joints, we have both rotation and translation
            Real currentAngle = this->computeJointAngle(positions.ref());
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real fixedAngle = d_fixedAngle.getValue();
            Real fixedPosition = d_fixedPosition.getValue();
            
            resV->set(cIndex, currentAngle - fixedAngle);
            resV->set(cIndex + 1, currentPosition - fixedPosition);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::UNIVERSAL:
        {
            // For universal joints, we have two rotations
            sofa::type::Vec3 currentAngles = this->computeJointAngles3D(positions.ref());
            sofa::type::Vec3 fixedAngles = d_fixedAngles3D.getValue();
            
            resV->set(cIndex, currentAngles[0] - fixedAngles[0]);
            resV->set(cIndex + 1, currentAngles[1] - fixedAngles[1]);
            break;
        }
        case softrobots::constraint::JointModel<DataTypes>::PLANAR:
        {
            // For planar joints, we have 2 translations + 1 rotation
            Real currentAngle = this->computeJointAngle(positions.ref());
            Real currentPosition = this->computeJointPosition(positions.ref());
            Real fixedAngle = d_fixedAngle.getValue();
            Real fixedPosition = d_fixedPosition.getValue();
            
            // Simplified implementation - more sophisticated planar joint handling needed
            resV->set(cIndex, currentPosition - fixedPosition);     // Translation X
            resV->set(cIndex + 1, 0.0);                           // Translation Y (placeholder)
            resV->set(cIndex + 2, currentAngle - fixedAngle);     // Rotation
            break;
        }
        default:
            msg_warning() << "JointModelEquality: Unsupported joint type.";
            break;
    }
}

} // namespace softrobotsinverse::constraint