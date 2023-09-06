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
#define SOFTROBOTS_INVERSE_POSITIONEFFECTOR_CPP
#include <sofa/core/ObjectFactory.h>
#include <SoftRobots.Inverse/component/constraint/PositionEffector.inl>

namespace softrobotsinverse::constraint
{

using namespace sofa::defaulttype;
using sofa::core::ConstraintParams;


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
using namespace sofa::helper;

// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int PositionEffectorClass = sofa::core::RegisterObject("This component is used to describe one or several desired positions "
                                                 "of points of a model, that will be reached by acting on chosen actuator(s).")
                .add< PositionEffector<Vec3Types> >(true)
                .add< PositionEffector<Vec2Types> >()
                .add< PositionEffector<Rigid3Types> >()
        
        ;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_INVERSE_API PositionEffector<sofa::defaulttype::Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API PositionEffector<sofa::defaulttype::Vec2Types>;
template class SOFA_SOFTROBOTS_INVERSE_API PositionEffector<sofa::defaulttype::Rigid3Types>;


} // namespace
