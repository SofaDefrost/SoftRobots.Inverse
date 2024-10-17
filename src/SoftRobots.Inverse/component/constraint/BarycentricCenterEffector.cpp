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
#define SOFTROBOTS_INVERSE_BARYCENTRICCENTEREFFECTOR_CPP

#include <SoftRobots.Inverse/component/constraint/BarycentricCenterEffector.inl>

#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{
using sofa::defaulttype::Vec3Types;
using sofa::defaulttype::Rigid3Types;
using sofa::core::RegisterObject ;

template<>
void BarycentricCenterEffector<Rigid3Types>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    if (!d_drawBarycenter.getValue())
        return;

    computeBarycenter();
    Coord barycenter = d_barycenter.getValue();
    vparams->drawTool()->drawFrame(barycenter.getCenter(), barycenter.getOrientation(), sofa::type::Vec3f(5., 5., 5.));
}

template<>
void BarycentricCenterEffector<Rigid3Types>::setBarycenter(const Coord& _barycenter)
{
    d_barycenter.setValue(_barycenter);
    auto barycenter = sofa::helper::getWriteAccessor(d_barycenter);
    barycenter->getOrientation().normalize();
}


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
int BarycentricCenterEffectorClass = RegisterObject("This component is used to describe one or several desired trajectories "
                                                    "for the barycenter of a model, that will be reached by acting on chosen actuator(s).")
        .add< BarycentricCenterEffector<Vec3Types> >(true)
        .add< BarycentricCenterEffector<Rigid3Types> >()

        ;

template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<Rigid3Types>;

} // namespace
