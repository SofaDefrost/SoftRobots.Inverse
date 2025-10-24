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

#include <SoftRobots.Inverse/component/behavior/Equality.h>
#include <SoftRobots/component/constraint/model/PositionModel.h>
#include <SoftRobots.Inverse/component/config.h>


namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;
using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;

using softrobots::behavior::SoftRobotsBaseConstraint;
using softrobots::behavior::SoftRobotsConstraint;
using softrobots::constraint::PositionModel;
using softrobotsinverse::behavior::Equality;

template< class DataTypes >
class PositionEquality : public Equality<DataTypes>, public PositionModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PositionEquality,DataTypes), SOFA_TEMPLATE(Equality,DataTypes));

    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::core::objectmodel::Data<VecCoord>		DataVecCoord;
    typedef sofa::core::objectmodel::Data<VecDeriv>		DataVecDeriv;
    typedef sofa::core::objectmodel::Data<MatrixDeriv>    DataMatrixDeriv;

    typedef sofa::type::vector<unsigned int> SetIndexArray;

public:
    PositionEquality(MechanicalState* object = nullptr);
    ~PositionEquality() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    /////////////////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector * Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from SoftRobotsBaseConstraint /////////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    using PositionModel<DataTypes>::d_indices;
    using PositionModel<DataTypes>::d_directions;
    using PositionModel<DataTypes>::d_useDirections;
    using PositionModel<DataTypes>::d_delta;

    using Equality<DataTypes>::m_hasDeltaEqual ;
    using Equality<DataTypes>::m_deltaEqual ;

    using Equality<DataTypes>::m_hasLambdaEqual;
    using Equality<DataTypes>::m_lambdaEqual ;
    
    using SoftRobotsBaseConstraint::d_constraintIndex ;
    using SoftRobotsBaseConstraint::m_nbLines ;
    using Equality<DataTypes>::d_componentState ;

    using SoftRobotsConstraint<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////


protected:
    sofa::Data<Real>    d_eqDelta;
    sofa::Data<Real>    d_constrainAtTime;

    void updateConstraint();
};

#ifndef SOFTROBOTSINVERSE_CONSTRAINT_POSITIONEQUALITY_CPP
extern template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<sofa::defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API PositionEquality<sofa::defaulttype::Rigid3Types>;
#endif // SOFTROBOTSINVERSE_CONSTRAINT_POSITIONEQUALITY_CPP

} // namespace

