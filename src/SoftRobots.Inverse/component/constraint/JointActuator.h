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
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{
    using softrobotsinverse::behavior::Actuator;
    using sofa::core::topology::BaseMeshTopology;
    using sofa::core::visual::VisualParams;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::helper::ReadAccessor;
    using sofa::core::ConstVecCoordId;
    using softrobots::behavior::SoftRobotsBaseConstraint;

/**
 * This component is used to solve an inverse problem by applying force on a joint (Vec1)
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class JointActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(JointActuator,DataTypes), SOFA_TEMPLATE(softrobotsinverse::behavior::Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                                  DataVecCoord;
    typedef sofa::Data<VecDeriv>                                  DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                               DataMatrixDeriv;


public:
    JointActuator(MechanicalState* = nullptr);
    ~JointActuator() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    /////////////////////////////////////////////////////////////

    ///////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////

protected:

    sofa::Data<unsigned int>  d_index;
    sofa::Data<Real>          d_initEffort;
    sofa::Data<Real>          d_initAngle;
    sofa::Data<Real>          d_maxEffort;
    sofa::Data<Real>          d_minEffort;
    sofa::Data<Real>          d_maxEffortVariation;
    sofa::Data<Real>          d_maxAngle;
    sofa::Data<Real>          d_minAngle;
    sofa::Data<Real>          d_maxAngleVariation;
    sofa::Data<Real>          d_effort;
    sofa::Data<Real>          d_angle;


    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::d_constraintIndex ;
    ////////////////////////////////////////////////////////////////////////////


    void initLimit();
    void initData();
    void updateLimit();

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_deltaMin ;
    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasDeltaMin ;
    using Actuator<DataTypes>::m_nbLines ;
    using Actuator<DataTypes>::d_componentState ;
    ////////////////////////////////////////////////////////////////////////////


private:

    void setUpData();

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_JOINTACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API JointActuator<sofa::defaulttype::Vec1Types>;
#endif

} // namespace
