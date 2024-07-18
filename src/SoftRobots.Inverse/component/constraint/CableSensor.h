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

#include <SoftRobots.Inverse/component/behavior/Sensor.h>
#include <SoftRobots/component/constraint/model/CableModel.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;


/**
 * This component simulates a cable sensor that measures its length.
 * TODO -> Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class CableSensor : public softrobotsinverse::behavior::Sensor<DataTypes> , public softrobots::constraint::CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableSensor,DataTypes),
               SOFA_TEMPLATE(softrobotsinverse::behavior::Sensor,DataTypes));

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

public:
    CableSensor(MechanicalState* object = nullptr);
    ~CableSensor() override;

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using softrobots::constraint::CableModel<DataTypes>::d_displacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_force ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxDispVariation ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_minForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqForce ;
    ////////////////////////////////////////////////////////////////////////////

private:

    void setUpData();
};

#if !defined(SOFTROBOTS_INVERSE_CABLESENSOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API CableSensor<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
