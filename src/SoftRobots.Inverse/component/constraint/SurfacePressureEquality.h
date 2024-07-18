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

#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <SoftRobots.Inverse/component/behavior/Equality.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{
    using softrobotsinverse::behavior::Equality;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::core::ConstVecCoordId;

/**
 * This component is used to solve an inverse problem by applying an imposed pressure on surfaces (for exemple cavities).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class SurfacePressureEquality : public Equality<DataTypes> , public softrobots::constraint::SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureEquality,DataTypes), SOFA_TEMPLATE(Equality,DataTypes));

    typedef typename DataTypes::Coord                       Coord;
    typedef typename Coord::value_type                      Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

public:
    SurfacePressureEquality(MechanicalState* object = nullptr);
    ~SurfacePressureEquality() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void reset() override;
    /////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from BaseConstraint ////////////////
    void storeLambda(const ConstraintParams* cParams,
                     sofa::core::MultiVecDerivId res,
                     const BaseVector* lambda) override;
    /////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    using Equality<DataTypes>::m_state ;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_cavityVolume ;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_initialCavityVolume;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_pressure ;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_volumeGrowth ;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqVolumeGrowth ;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void updateConstraint();

    ////////////////////////// Inherited attributes ////////////////////////////
    using Equality<DataTypes>::m_hasDeltaEqual ;
    using Equality<DataTypes>::m_hasLambdaEqual ;
    using Equality<DataTypes>::m_deltaEqual ;
    using Equality<DataTypes>::m_lambdaEqual ;
    using Equality<DataTypes>::m_nbLines ;
    ////////////////////////////////////////////////////////////////////////////

};

#if !defined(SOFTROBOTS_INVERSE_SURFACEPRESSUREEQUALITY_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SurfacePressureEquality<sofa::defaulttype::Vec3Types>;
#endif

} // namespace
