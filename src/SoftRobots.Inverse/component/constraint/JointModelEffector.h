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

#include <SoftRobots.Inverse/component/behavior/Effector.h>
#include <SoftRobots/component/constraint/model/JointModel.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;

/**
 * Joint effector for inverse problem resolution.
 * This component defines target joint angles/positions that should be reached by actuators.
 * It supports different types of joints: revolute, prismatic, spherical, planar, cylindrical, and universal.
 * Based on the JointModel class for kinematic computation.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template< class DataTypes >
class JointModelEffector : public softrobotsinverse::behavior::Effector<DataTypes>, public softrobots::constraint::JointModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointModelEffector,DataTypes),
               SOFA_TEMPLATE(softrobotsinverse::behavior::Effector,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

public:
    JointModelEffector(MechanicalState* object = nullptr);
    ~JointModelEffector() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void getConstraintViolation(const ConstraintParams* cParams,
                                sofa::linearalgebra::BaseVector *resV,
                                const sofa::linearalgebra::BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using softrobots::constraint::JointModel<DataTypes>::d_jointAngle;
    using softrobots::constraint::JointModel<DataTypes>::d_jointPosition1D;
    using softrobots::constraint::JointModel<DataTypes>::d_jointAngles3D;
    using softrobots::constraint::JointModel<DataTypes>::d_constraintIndex;
    using softrobots::constraint::JointModel<DataTypes>::m_state;
    using softrobots::constraint::JointModel<DataTypes>::getJointTypeFromData;
    using softrobots::constraint::JointModel<DataTypes>::computeJointAngle;
    using softrobots::constraint::JointModel<DataTypes>::computeJointPosition;
    using softrobots::constraint::JointModel<DataTypes>::computeJointAngles3D;

    using softrobots::constraint::JointModel<DataTypes>::d_maxDisplacement;
    using softrobots::constraint::JointModel<DataTypes>::d_minDisplacement;
    using softrobots::constraint::JointModel<DataTypes>::d_eqDisplacement;
    using softrobots::constraint::JointModel<DataTypes>::d_maxForce;
    using softrobots::constraint::JointModel<DataTypes>::d_minForce;
    using softrobots::constraint::JointModel<DataTypes>::d_eqForce;

    using softrobotsinverse::behavior::Effector<DataTypes>::getTarget;
    ////////////////////////////////////////////////////////////////////////////

protected:

    sofa::Data<Real> d_desiredAngle;        ///< Desired joint angle for revolute/cylindrical joints
    sofa::Data<Real> d_desiredPosition;     ///< Desired joint position for prismatic/cylindrical joints
    sofa::Data<sofa::type::Vec3> d_desiredAngles3D; ///< Desired joint angles for spherical/universal joints

private:

    void setUpData();
    
    Real getTargetAngle();
    Real getTargetPosition();
    sofa::type::Vec3 getTargetAngles3D();

};

#if !defined(SOFTROBOTS_INVERSE_CONSTRAINT_JOINTMODELEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEffector<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEffector<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEffector<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace softrobotsinverse::constraint