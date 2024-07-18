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

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{

using softrobotsinverse::behavior::Actuator;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;
using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using softrobots::behavior::SoftRobotsBaseConstraint;
using softrobots::behavior::SoftRobotsConstraint;


/**
 * This component simulates a force exerted along a given direction to solve an inverse problem.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SlidingActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SlidingActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

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
    typedef Actuator<DataTypes> Inherit;

public:
    SlidingActuator(MechanicalState* object = nullptr);
    ~SlidingActuator() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void bwdInit() override;
    void reinit() override;
    void reset() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    //////////////// Inherited from SoftRobotsConstraint ///////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector * Jdx) override;
    /////////////////////////////////////////////////////////////////////////


    /////////////// Inherited from BaseSoftRobotsConstraint /////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Inherit::m_hasDeltaMax ;
    using Inherit::m_hasDeltaMin ;
    using Inherit::m_deltaMax ;
    using Inherit::m_deltaMin ;

    using Inherit::m_hasLambdaMax ;
    using Inherit::m_hasLambdaMin ;
    using Inherit::m_lambdaMax ;
    using Inherit::m_lambdaMin ;

    using Inherit::m_nbLines ;
    using Inherit::m_constraintIndex ;
    using Inherit::d_componentState ;

    using Inherit::m_state ;
    ////////////////////////////////////////////////////////////////////////////


    sofa::Data<Real>                  d_maxPositiveDisplacement;
    sofa::Data<Real>                  d_maxNegativeDisplacement;
    sofa::Data<Real>                  d_maxDispVariation;
    sofa::Data<Real>                  d_maxForce;
    sofa::Data<Real>                  d_minForce;

    sofa::Data<Deriv>                 d_direction;
    sofa::Data<SetIndexArray>         d_indices;

    sofa::Data<double>                d_initForce;
    sofa::Data<double>                d_force;
    sofa::Data<double>                d_initDisplacement;
    sofa::Data<double>                d_displacement;
    sofa::Data<bool>                  d_accumulateDisp;

    sofa::Data<bool>                  d_showDirection;
    sofa::Data<double>                d_showVisuScale;
    
    void initData();
    void initLimit();
    void updateLimit();
    void checkIndicesRegardingState();
};

#ifndef SOFTROBOTSINVERSE_CONSTRAINT_SLIDINGACTUATOR_CPP
extern template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API SlidingActuator<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace
