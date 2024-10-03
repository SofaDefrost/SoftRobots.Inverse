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

#include <SoftRobots.Inverse/component/constraint/BarycentricCenterEffector.h>


namespace softrobotsinverse::constraint
{

using sofa::core::ConstVecCoordId;
using sofa::type::RGBAColor;
using sofa::helper::ReadAccessor;

using sofa::core::objectmodel::BaseContext;
using sofa::core::collision::ContactManager;
using sofa::type::vector;
using sofa::type::vector;
using sofa::type::Vec3;

using sofa::core::objectmodel::ComponentState;

template<class DataTypes>
BarycentricCenterEffector<DataTypes>::BarycentricCenterEffector(MechanicalState* object)
    : Inherit1(object)
    , d_axis(initData(&d_axis, sofa::type::Vec<3,bool>(true,true,true), "axis",
                      "The parameter axis is of type Vec3<bool> and allows to specify the directions in \n"
                      "which you want to solve the effector. If unspecified, the default    \n"
                      "values are {true, true, true})."))

    , d_drawBarycenter(initData(&d_drawBarycenter,false,"drawBarycenter",
                                "If true, draw the barycenter" ))

    , d_barycenter(initData(&d_barycenter,"barycenter",
                            "Position of barycenter." ))
{
    d_axis.setDisplayed(false);
    d_directions.setDisplayed(false); // inherited from PositionModel but not used here
    d_indices.setDisplayed(false); // inherited from PositionModel but not used here
    d_drawBarycenter.setGroup("Visualization");
    d_barycenter.setReadOnly(true);
}

template<class DataTypes>
BarycentricCenterEffector<DataTypes>::~BarycentricCenterEffector()
{
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::init()
{
    // Inherited from PositionEffector
    // Even though we do not use this data in the component, we initilize the data to avoid the warnings about the wrong usage
    auto indices = sofa::helper::getWriteAccessor(d_indices);
    indices.resize(1);

    Inherit1::init();

    if (d_axis.isSet())
    {
        msg_deprecated() << "The data axis is deprecated. To fix your scene please use useDirections instead. It will be remove in v25.06.";
        auto useDirections = sofa::helper::getWriteAccessor(d_useDirections);
        const auto& axis = sofa::helper::getReadAccessor(d_axis);
        useDirections[0] = axis[0];
        useDirections[1] = axis[1];
        useDirections[2] = axis[2];
    }
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::reset()
{
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::computeBarycenter()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    const unsigned int nbp = m_state->getSize();
    Coord barycenter = Coord();
    for (unsigned int i=0; i<nbp; i++)
        for(sofa::Size j=0; j<DataTypes::Coord::total_size; j++)
            barycenter[j] += positions[i][j]/Real(nbp);

    d_barycenter.setValue(barycenter);
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                 DataMatrixDeriv &cMatrix,
                                                                 unsigned int &cIndex,
                                                                 const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    d_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(d_constraintIndex);
    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);
    const auto& directions = sofa::helper::getReadAccessor(d_directions);
    const auto& weight = sofa::helper::getReadAccessor(d_weight);

    const unsigned int nbp = m_state->getSize();

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    unsigned int index = 0;
    for(sofa::Size j=0; j<Deriv::total_size; j++)
    {
        if(useDirections[j])
        {
            MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
            for (unsigned int i=0; i<nbp; i++)
            {
                rowIterator.setCol(i, directions[j] * weight[j] * 1.0/Real(nbp));
            }
            index++;
        }
    }

    cIndex+=index;

    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
    computeBarycenter();
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                  BaseVector *resV,
                                                                  const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);

    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();
    ReadAccessor<sofa::Data<VecCoord> > effectorGoal = d_effectorGoal;

    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);
    const auto& directions = sofa::helper::getReadAccessor(d_directions);
    const auto& weight = sofa::helper::getReadAccessor(d_weight);
    const auto& constraintIndex = sofa::helper::getReadAccessor(d_constraintIndex);

    computeBarycenter();
    const auto& barycenter = sofa::helper::getReadAccessor(d_barycenter);

    Coord target = getTarget(effectorGoal[0], barycenter);
    Deriv d = DataTypes::coordDifference(barycenter, target);

    int index = 0;
    for(sofa::Size j=0; j<DataTypes::Deriv::total_size; j++)
    {
        if(useDirections[j])
        {
            Real dfree = Jdx->element(index) + d * directions[j] * weight[j];
            resV->set(constraintIndex+index, dfree);
            index++;
        }
    }
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    computeBarycenter();
    Coord b = d_barycenter.getValue();
    std::vector<Vec3> positions;
    positions.push_back(b);
    vparams->drawTool()->drawPoints(positions, float(5.), RGBAColor(0.0f,0.0f,1.0f,1.0f));
}

} // namespace
