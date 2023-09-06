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
#include <SoftRobots/component/constraint/model/CableModel.h>


namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;


/**
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class CableEffector : public softrobotsinverse::behavior::Effector<DataTypes> , public softrobots::constraint::CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableEffector,DataTypes),
               SOFA_TEMPLATE(softrobotsinverse::behavior::Effector,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;


public:
    CableEffector(MechanicalState* object = nullptr);
    ~CableEffector() override;

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
    using softrobots::constraint::CableModel<DataTypes>::d_cableLength ;
    using softrobots::constraint::CableModel<DataTypes>::m_constraintId ;
    using softrobots::constraint::CableModel<DataTypes>::m_state ;
    using softrobots::constraint::CableModel<DataTypes>::getCableLength ;

    using softrobots::constraint::CableModel<DataTypes>::d_maxDispVariation ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_minForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqForce ;

    using softrobotsinverse::behavior::Effector<DataTypes>::getTarget;
    ////////////////////////////////////////////////////////////////////////////

protected:

    sofa::Data<Real> d_desiredLength;

private:

    void setUpData();

};

#if !defined(SOFTROBOTS_INVERSE_CONSTRAINT_CABLEEFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API CableEffector<sofa::defaulttype::Vec3Types>;
#endif

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using CableEffector SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS_INVERSE()
        = softrobotsinverse::constraint::CableEffector<DataTypes>;
}
