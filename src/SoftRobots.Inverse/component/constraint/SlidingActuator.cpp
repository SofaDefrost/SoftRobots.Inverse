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
#define SOFTROBOTSINVERSE_CONSTRAINT_SLIDINGACTUATOR_CPP

#include <sofa/defaulttype/VecTypes.h>
#include <SoftRobots.Inverse/component/config.h>
#include <sofa/core/ObjectFactory.h>
using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

#include <SoftRobots.Inverse/component/constraint/SlidingActuator.inl>

namespace softrobotsinverse::constraint
{
using sofa::defaulttype::Rigid3Types;

template<>
void SlidingActuator<Rigid3Types>::initDatas()
{
    if (!d_direction.isSet())
    {
        msg_warning()<<"No direction of actuation provided by user. Default (1. 0. 0. 0. 0. 0.)";
        Deriv x;
        x[0]=1;
        d_direction.setValue(x);
    }
    else
    {
        Deriv direction = d_direction.getValue();
        Vec<3,double> center = direction.getVCenter();
        Vec<3,double> orientation = direction.getVOrientation();
        center.normalize();
        orientation.normalize();
        direction.getVCenter() = center;
        direction.getVOrientation()= orientation;

        d_direction.setValue(direction);
    }

    d_displacement.setValue(d_initDisplacement.getValue());
    d_force.setValue(d_initForce.getValue());
}


int SlidingActuatorClass = RegisterObject("This component simulates a force exerted along a given direction to solve an inverse problem. \n"
                                          "In case of Rigid template, it additionally simulates a force in rotation: the size of 'direction' is equal to 6,\n"
                                          "the three first component give the direction in translation and the three last the axe of rotation.")
.add< SlidingActuator<Vec3Types> >(true)
.add< SlidingActuator<Rigid3Types> >()

;

template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<Rigid3Types>;


} // namespace
