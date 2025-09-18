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

#include <SoftRobots/component/constraint/model/JointModel.h>
#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;

/**
 * Joint equality constraint for inverse problem resolution.
 * This component enforces strict equality constraints on joint angles/positions.
 * Unlike effectors which define targets to reach, equality constraints must be satisfied exactly.
 * It supports different types of joints: revolute, prismatic, spherical, planar, cylindrical, and universal.
 * Based on the JointModel class for kinematic computation.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template< class DataTypes >
class JointModelEquality : public softrobots::constraint::JointModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointModelEquality,DataTypes),
               SOFA_TEMPLATE(softrobots::constraint::JointModel,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

public:
    JointModelEquality(MechanicalState* object = nullptr);
    ~JointModelEquality() override;

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

protected:

    sofa::Data<Real> d_fixedAngle;          ///< Fixed joint angle for revolute/cylindrical joints
    sofa::Data<Real> d_fixedPosition;       ///< Fixed joint position for prismatic/cylindrical joints
    sofa::Data<sofa::type::Vec3> d_fixedAngles3D; ///< Fixed joint angles for spherical/universal joints

private:

    void setUpData();

};

#if !defined(SOFTROBOTS_INVERSE_CONSTRAINT_JOINTMODELEQUALITY_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace softrobotsinverse::constraint