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

#include <sofa/component/solidmechanics/spring/SpringForceField.h>
#include <SoftRobots.Inverse/component/behavior/Actuator.h>

#include <SoftRobots.Inverse/component/config.h>


namespace softrobotsinverse::constraint
{

template< class DataTypes >
class SpringStiffnessActuator : public behavior::Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(SpringStiffnessActuator,DataTypes), SOFA_TEMPLATE(behavior::Actuator,DataTypes));

    typedef typename DataTypes::VecCoord        VecCoord;
    typedef typename DataTypes::VecDeriv        VecDeriv;
    typedef typename DataTypes::VecReal         VecReal;
    typedef typename DataTypes::Coord           Coord;
    typedef typename DataTypes::Deriv           Deriv;
    typedef typename DataTypes::MatrixDeriv     MatrixDeriv;
    typedef typename Coord::value_type          Real;
    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>		DataVecCoord;
    typedef sofa::Data<VecDeriv>		DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>    DataMatrixDeriv;

    SpringStiffnessActuator(MechanicalState* object = nullptr);
    ~SpringStiffnessActuator() override = default;

    void init() override;
    void reinit() override;
    void bwdInit() override;
    void reset() override;

    void buildConstraintMatrix(const sofa::core::ConstraintParams * cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;


    void getConstraintViolation(const sofa::core::ConstraintParams* cParams ,
                                sofa::linearalgebra::BaseVector *resV,
                                const sofa::linearalgebra::BaseVector *Jdx) override;

    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;


    sofa::Data<Real> d_minValue;
    sofa::Data<Real> d_maxValue;


    /// Link to be set to the forcefield in the component graph.
    sofa::SingleLink<SpringStiffnessActuator<DataTypes>,
                     sofa::component::solidmechanics::spring::SpringForceField<DataTypes>,
                     sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK> l_forceField;


private:
    void initLimit();
    void updateLimit();

    VecReal m_initialValues;
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_SPRINGSTIFFNESSACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API SpringStiffnessActuator<sofa::defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_INVERSE_API SpringStiffnessActuator<sofa::defaulttype::Rigid3Types>;
#endif

} // namespace





