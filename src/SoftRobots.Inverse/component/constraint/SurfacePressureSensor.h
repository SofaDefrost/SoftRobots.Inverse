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

#include <SoftRobots.Inverse/component/behavior/Sensor.h>
#include <SoftRobots/component/constraint/model/SurfacePressureModel.h>
#include <sofa/core/behavior/ConstraintResolution.h>

namespace softrobotsinverse::constraint
{

using softrobotsinverse::behavior::Sensor;
using sofa::helper::ReadAccessor;
using sofa::core::VecCoordId;
using sofa::core::ConstraintParams;

using sofa::core::behavior::ConstraintResolution ;

/**
 * This component allows the user to get the (change of) volume of a cavity
 * TODO: In the future we should also aim for giving the (change of) pressure, which
 * TODO -> Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/

class DoNothingConstraintResolution : public ConstraintResolution
{
public:
    DoNothingConstraintResolution();

//////////////////// Inherited from ConstraintResolution ////////////////////
void init(int line, double** w, double *lambda);
void resolution(int line, double** w, double* d, double* lambda, double* dfree);
/////////////////////////////////////////////////////////////////////////////

};

template< class DataTypes >
class SurfacePressureSensor : public Sensor<DataTypes> , public softrobots::constraint::SurfacePressureModel<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SurfacePressureSensor,DataTypes), SOFA_TEMPLATE(Sensor,DataTypes));

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

public:    
    SurfacePressureSensor(MechanicalState* object = nullptr);
    ~SurfacePressureSensor() override;

    void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;

    ////////////////////////// Inherited attributes ////////////////////////////
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_eqVolumeGrowth;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxPressure;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxPressureVariation;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_minVolumeGrowth;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowth;
    using softrobots::constraint::SurfacePressureModel<DataTypes>::d_maxVolumeGrowthVariation;
    ////////////////////////////////////////////////////////////////////////////

};

#if !defined(SOFTROBOTS_INVERSE_SURFACEPRESSURESENSOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SurfacePressureSensor<sofa::defaulttype::Vec3Types>;
#endif

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using SurfacePressureSensor SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS_INVERSE()
        = softrobotsinverse::constraint::SurfacePressureSensor<DataTypes>;
}
