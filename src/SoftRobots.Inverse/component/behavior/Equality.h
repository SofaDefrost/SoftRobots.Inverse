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

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::behavior
{

/**
 *  \brief This class specifies the type of the constraint as Equality.
 */

template<class DataTypes>
class Equality : virtual public softrobots::behavior::SoftRobotsConstraint<DataTypes>
{
public:

    SOFA_CLASS(SOFA_TEMPLATE(Equality,DataTypes), SOFA_TEMPLATE(softrobots::behavior::SoftRobotsConstraint,DataTypes));

    Equality(sofa::core::behavior::MechanicalState<DataTypes> *mm = nullptr);
    ~Equality() override;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using softrobots::behavior::SoftRobotsBaseConstraint::m_constraintType ;
    using softrobots::behavior::SoftRobotsBaseConstraint::EQUALITY ;
    ////////////////////////////////////////////////////////////////////////////
};

#if !defined(SOFTROBOTS_INVERSE_EQUALITY_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Equality<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace
