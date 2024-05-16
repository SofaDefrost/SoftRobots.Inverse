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

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>

#include <SoftRobots.Inverse/component/constraint/PositionEffector.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor ;
using sofa::helper::WriteAccessor ;
using sofa::core::objectmodel::ComponentState;

template<class DataTypes>
PositionEffector<DataTypes>::PositionEffector(MechanicalState* object)
    : Effector<DataTypes>(object)
    , softrobots::constraint::PositionModel<DataTypes>(object)
    , d_effectorGoal(initData(&d_effectorGoal,"effectorGoal",
                    "Desired positions. \n"
                    "If the size does not match with the size of indices, \n"
                    "one will resize considerering the smallest one."))
{
}

template<class DataTypes>
PositionEffector<DataTypes>::~PositionEffector()
{
}

template<class DataTypes>
void PositionEffector<DataTypes>::init()
{
    softrobots::constraint::PositionModel<DataTypes>::init();

    if(!d_effectorGoal.isSet())
    {
        msg_warning(this) <<"TargetPosition not defined. Default value assigned  ("<<Coord()<<").";
        setTargetDefaultValue();
    }

    if(d_indices.getValue().size() != d_effectorGoal.getValue().size())
        resizeData();
}

template<class DataTypes>
void PositionEffector<DataTypes>::setTargetDefaultValue()
{
    WriteAccessor<sofa::Data<VecCoord> > defaultTarget = d_effectorGoal;
    defaultTarget.resize(1);
    defaultTarget[0] = Coord();
}

template<class DataTypes>
void PositionEffector<DataTypes>::resizeData()
{
    if(d_indices.getValue().size() < d_effectorGoal.getValue().size())
    {
        msg_warning(this)<<"Indices size is lower than target size, some targets will not be considered.";
    }
    else
    {
        msg_warning(this) <<"Indices size is larger than target size. Launch resize process.";
        WriteAccessor<sofa::Data<sofa::type::vector<unsigned int> > > indices = d_indices;
        indices.resize(d_effectorGoal.getValue().size());
    }
}

template<class DataTypes>
void PositionEffector<DataTypes>::getConstraintViolation(const sofa::core::ConstraintParams* cParams,
                                                         sofa::linearalgebra::BaseVector *resV,
                                                         const sofa::linearalgebra::BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);

    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();
    ReadAccessor<sofa::Data<VecCoord> > effectorGoal = d_effectorGoal;

    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);
    const auto& directions = sofa::helper::getReadAccessor(d_directions);
    const auto& weight = sofa::helper::getReadAccessor(d_weight);
    const auto& indices = sofa::helper::getReadAccessor(d_indices);
    sofa::Index sizeIndices = indices.size();


    int index = 0;
    for (unsigned int i=0; i<sizeIndices; i++)
    {
        Coord pos = x[indices[i]];
        Coord goalPos = getTarget(effectorGoal[i],pos);

        Deriv d = DataTypes::coordDifference(pos,goalPos);

        for(sofa::Size j=0; j<DataTypes::Deriv::total_size; j++)
            if(useDirections[j])
            {
                Real dfree = Jdx->element(index) + d*directions[j]*weight[j];
                resV->set(m_constraintId+index, dfree);
                index++;
            }
    }
}


} // namespace
