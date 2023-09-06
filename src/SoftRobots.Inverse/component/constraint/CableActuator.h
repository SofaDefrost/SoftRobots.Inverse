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
#include <SoftRobots/component/constraint/model/CableModel.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{

using sofa::core::behavior::Actuator;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;


/**
 * This component simulates a force exerted by a cable to solve an effector constraint.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class CableActuator : public Actuator<DataTypes> , public softrobots::constraint::CableModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CableActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::Coord Coord;
    typedef typename Coord::value_type Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;


public:
    CableActuator(MechanicalState* object = nullptr);
    ~CableActuator() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using softrobots::constraint::CableModel<DataTypes>::d_force ;
    using softrobots::constraint::CableModel<DataTypes>::d_displacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxDispVariation ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxNegativeDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxPositiveDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqDisplacement ;
    using softrobots::constraint::CableModel<DataTypes>::d_maxForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_minForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_eqForce ;
    using softrobots::constraint::CableModel<DataTypes>::d_color ;
    using softrobots::constraint::CableModel<DataTypes>::m_state ;

    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasDeltaMin ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_deltaMin ;

    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaInit ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_lambdaInit ;
    ////////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const sofa::linearalgebra::BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    ////////////////////////////////////////////////////////////////////////

protected:
    sofa::Data<Real>                  d_initForce;
    sofa::Data<Real>                  d_constrainAtTime;
    sofa::Data<bool>                  d_displayCableLimit;

    sofa::type::RGBAColor    m_color;

    void initDatas();
    void initLimit();
    void updateLimit();
    void updateVisualization();
};

#if !defined(SOFTROBOTS_INVERSE_CABLEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API CableActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace


namespace sofa::component::constraintset
{
    template <class DataTypes>
    using CableActuator SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS_INVERSE()
        = softrobotsinverse::constraint::CableActuator<DataTypes>;
}
