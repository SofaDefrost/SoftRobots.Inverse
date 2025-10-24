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

#define SOFTROBOTSINVERSE_CONSTRAINT_POSITIONEQUALITY_CPP

#include <SoftRobots.Inverse/component/constraint/PositionEquality.inl>

#include <sofa/defaulttype/VecTypes.h>
#include <sofa/core/ObjectFactory.h>
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

namespace softrobotsinverse::constraint
{

void registerPositionEquality(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("In an inverse problem, constrains the given positions on chosen directions.")
                                 .add< PositionEquality<Vec2Types> >()
                                 .add< PositionEquality<Vec3Types> >(true)
                                 .add< PositionEquality<Rigid3Types> >());
}

template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<Vec2Types>;
template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<Rigid3Types>;

} // namespace
