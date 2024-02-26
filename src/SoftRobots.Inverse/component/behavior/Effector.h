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
 *  \brief This class specifies the type of the constraint as effector.
 */

template<class DataTypes>
class Effector : virtual public softrobots::behavior::SoftRobotsConstraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(Effector,DataTypes), softrobots::behavior::SoftRobotsConstraint<DataTypes>);
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;

    Effector(sofa::core::behavior::MechanicalState<DataTypes> *mm = nullptr);
    ~Effector() override;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using softrobots::behavior::SoftRobotsBaseConstraint::m_constraintType ;
    using softrobots::behavior::SoftRobotsBaseConstraint::EFFECTOR ;
    ////////////////////////////////////////////////////////////////////////////

protected:

    sofa::Data<bool>   d_limitShiftToTarget;
    sofa::Data<Real>   d_maxShiftToTarget;
    sofa::Data<Real>   d_maxSpeed;

    SReal getTarget(const Real& target, const Real& current);
    Coord getTarget(const Coord& target, const Coord& current);
};

#if !defined(SOFTROBOTS_INVERSE_EFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API Effector<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Effector<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Effector<sofa::defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API Effector<sofa::defaulttype::Rigid3Types>;
#endif
} // namespace

namespace sofa::core::behavior
{
    template <class DataTypes>
    using Effector SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS_INVERSE()
        = softrobotsinverse::behavior::Effector<DataTypes>;
}
