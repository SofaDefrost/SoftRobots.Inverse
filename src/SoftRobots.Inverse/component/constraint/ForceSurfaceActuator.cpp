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
#define SOFTROBOTS_INVERSE_FORCESURFACEACTUATOR_CPP

#include <SoftRobots.Inverse/component/constraint/ForceSurfaceActuator.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

void registerForceSurfaceActuator(ObjectFactory* factory)
{
    factory->registerObjects(ObjectRegistrationData("This component is used to solve an inverse problem by applying a force on a given surface of a model.")
                                 .add< ForceSurfaceActuator<Vec3Types> >(true));
}

template class SOFA_SOFTROBOTS_INVERSE_API ForceSurfaceActuator<Vec3Types>;


} // namespace

