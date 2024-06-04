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

#include <sofa/component/solidmechanics/fem/elastic/TetrahedronFEMForceField.h>
#include <SoftRobots.Inverse/component/behavior/Actuator.h>

#include <SoftRobots.Inverse/component/config.h>


namespace sofa::component::constraintset
{
    using sofa::core::visual::VisualParams ;
    using sofa::linearalgebra::BaseVector ;
    using sofa::core::ConstraintParams ;
    using sofa::core::behavior::Actuator ;
    using sofa::component::solidmechanics::fem::elastic::TetrahedronFEMForceField ;

/**
 * This component is used to solve effector constraint by changing young moduli.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class YoungModulusActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(YoungModulusActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::VecCoord        VecCoord;
    typedef typename DataTypes::VecDeriv        VecDeriv;
    typedef typename DataTypes::VecReal         VecReal;
    typedef typename DataTypes::Coord           Coord;
    typedef typename DataTypes::Deriv           Deriv;
    typedef typename DataTypes::MatrixDeriv     MatrixDeriv;
    typedef typename Coord::value_type          Real;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>    DataMatrixDeriv;

public:
    YoungModulusActuator(MechanicalState* object = nullptr);
    ~YoungModulusActuator() override;

    ///////////////// Inherited from BaseObject ///////////////////////////
    void init() override;
    void reinit() override;
    void bwdInit() override;
    void reset() override;
    ///////////////////////////////////////////////////////////////////////


    ///////////////// Inherited from SoftRobotsConstraint /////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;


    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////


    ///////////////// Inherited from BaseSoftRobotsConstraint ///////////////////////////
    void storeResults(type::vector<double> &lambda,
                      type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////////////

protected:

    Data<Real> d_minYoung;
    Data<Real> d_maxYoung;
    Data<Real> d_maxYoungVariationRatio;
    Data<bool> d_hasVolumeOptimization;

    double m_previousYoungValue;
    Real   m_previousVolumeValue;

    double m_deltaYoungModulus;
    Real   m_deltaVolume;

    bool m_initError;
    Real m_youngModulus;
    Real m_initialYoungModulus;

    TetrahedronFEMForceField< DataTypes > * m_tetraForceField;


private:
    void initLimit();
    void updateLimit();
    void getForce(VecDeriv& force, const DataVecCoord &x);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::m_nbLines ;
    using Actuator<DataTypes>::m_constraintIndex ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::getContext ;
    using Actuator<DataTypes>::d_componentState ;
    ///////////////////////////////////////////////////////////////////////////
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_YOUNGMODULUSACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API YoungModulusActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace sofa::component::constraintset





