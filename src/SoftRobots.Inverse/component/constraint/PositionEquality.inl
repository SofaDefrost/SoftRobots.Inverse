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
#include <sofa/type/Vec.h>
#include <SoftRobots.Inverse/component/constraint/PositionEquality.h>

namespace softrobotsinverse::constraint
{

using sofa::core::objectmodel::ComponentState ;

using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;
using sofa::type::Vec;

template<class DataTypes>
PositionEquality<DataTypes>::PositionEquality(MechanicalState* object)
    : Equality<DataTypes>(object)
    , softrobots::constraint::PositionModel<DataTypes>(object)
    , d_eqDelta(initData(&d_eqDelta, Real(0.0), "eqDelta",
                         ""))
    , d_constrainAtTime(initData(&d_constrainAtTime,Real(0.0), "constrainAtTime",
                                 "No constraints will be applied before this time. \n"
                                 "Example of use: to allow the cable to reach an \n"
                                 "initial configuration before optimizing."))
{
    // PositionEquality of dimension 1
    m_deltaEqual.resize(1);
}


template<class DataTypes>
PositionEquality<DataTypes>::~PositionEquality()
{
}


template<class DataTypes>
void PositionEquality<DataTypes>::init()
{
    softrobots::constraint::PositionModel<DataTypes>::init();
    updateConstraint();
}


template<class DataTypes>
void PositionEquality<DataTypes>::updateConstraint()
{
    Real time = this->getContext()->getTime();
    if(time < d_constrainAtTime.getValue())
        return;

    m_hasDeltaEqual = true;
    m_deltaEqual[0] = d_eqDelta.getValue();
}


template<class DataTypes>
void PositionEquality<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                        DataMatrixDeriv &cMatrix,
                                                        unsigned int &cIndex,
                                                        const DataVecCoord &x)
{
    SOFA_UNUSED(x);

    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);
    MatrixDeriv& matrix = *cMatrix.beginEdit();
    const auto& indices = sofa::helper::getReadAccessor(d_indices);
    const auto& directions = sofa::helper::getReadAccessor(d_directions);
    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);
    m_constraintId = cIndex;

    for (sofa::Index i: indices)
    {
        for (sofa::Index j=0; j<Deriv::total_size; j++)
        {
            if (useDirections[j])
            {
                MatrixDerivRowIterator c_it = matrix.writeLine(cIndex++);
                c_it.addCol(i, directions[j]);
            }
        }
    }

    cMatrix.endEdit();
    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void PositionEquality<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                         BaseVector *resV,
                                                         const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    SOFA_UNUSED(cParams);
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();
    const auto& indices = sofa::helper::getReadAccessor(d_indices);
    const auto& useDirections = sofa::helper::getReadAccessor(d_useDirections);

    bool withJdx = (Jdx->size()!=0);

    sofa::Index index = 0;
    for (sofa::Index i: indices)
    {
        for (sofa::Index j=0; j<Coord::total_size; j++)
        {
            if (useDirections[j])
            {
                Real dfree = positions[i][j];
                if (withJdx)
                {
                    dfree += Jdx->element(index);
                }
                resV->set(m_constraintId + index++, dfree);
            }
        }
    }
}

template<class DataTypes>
void PositionEquality<DataTypes>::storeResults(sofa::type::vector<double> &lambda,
                                               sofa::type::vector<double> &delta)
{
    SOFA_UNUSED(lambda);
    d_delta.setValue(delta);
    updateConstraint();
}


} // namespace
