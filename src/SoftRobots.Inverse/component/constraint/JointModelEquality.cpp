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
#define SOFTROBOTS_INVERSE_CONSTRAINT_JOINTMODELEQUALITY_CPP

#include <SoftRobots.Inverse/component/constraint/JointModelEquality.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::core::objectmodel;

/////////////////////////////////////////// FACTORY //////////////////////////////////////////////
///
/// Register the component to the ObjectFactory
///
int JointModelEqualityClass = sofa::core::RegisterObject("Joint equality constraint based on JointModel for inverse problem resolution")
        .add< JointModelEquality<Vec3Types> >(true)
        .add< JointModelEquality<Vec2Types> >()
        .add< JointModelEquality<Rigid3Types> >();

/////////////////////////////////////////// TEMPLATE INSTANTIATION ///////////////////////////////
///
/// Instanciate the templates
///
template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Vec2Types>;
template class SOFA_SOFTROBOTS_INVERSE_API JointModelEquality<sofa::defaulttype::Rigid3Types>;

} // namespace softrobotsinverse::constraint