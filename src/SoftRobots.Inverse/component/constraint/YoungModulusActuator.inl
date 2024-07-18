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

#include "YoungModulusActuator.h"


namespace sofa::component::constraintset
{

using type::vector;
using core::objectmodel::BaseContext;
using sofa::core::objectmodel::ComponentState;

template<class DataTypes>
YoungModulusActuator<DataTypes>::YoungModulusActuator(MechanicalState* object)
    : Inherit1(object)

    , d_minYoung(initData(&d_minYoung,(Real)1.0e1, "minYoung",
                          "Minimum value for Young Modulus. \n"
                          "If unspecified default value 1.0e1."))

    , d_maxYoung(initData(&d_maxYoung,(Real)1.0e5, "maxYoung",
                          "Maximum value for Young Modulus. \n"
                          "If unspecified default value 1.0e5."))

    , d_maxYoungVariationRatio(initData(&d_maxYoungVariationRatio, (Real)1.0e1, "maxYoungVariationRatio",
                                        "Maximum variation of young / its actual value. \n"
                                        "If unspecified default value 1.0e1."))
    , m_previousYoungValue(0.0)
    , m_previousVolumeValue(0.0)
    , m_deltaYoungModulus(0.0)
    , m_deltaVolume(0.0)
    , m_initError(false)
    , m_youngModulus(0.0)
{
    // QP on only one value, we set dimension to one
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}


template<class DataTypes>
YoungModulusActuator<DataTypes>::~YoungModulusActuator()
{
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::init()
{
    d_componentState = ComponentState::Valid;
    Inherit1::init();
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::reinit()
{
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::initLimit()
{
    m_hasLambdaMin=true;
    m_hasLambdaMax=true;

    updateLimit();
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::updateLimit()
{
    m_lambdaMin[0]=-(m_youngModulus-d_minYoung.getValue());
    m_lambdaMax[0]=d_maxYoung.getValue()-m_youngModulus;

    double youngMin=m_youngModulus-m_youngModulus*d_maxYoungVariationRatio.getValue();
    if(youngMin>=d_minYoung.getValue())
        m_lambdaMin[0]=-m_youngModulus*d_maxYoungVariationRatio.getValue();

    double youngMax=m_youngModulus+m_youngModulus*d_maxYoungVariationRatio.getValue();
    if(youngMax<=d_maxYoung.getValue())
        m_lambdaMax[0]=m_youngModulus*d_maxYoungVariationRatio.getValue();
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::bwdInit()
{
    BaseContext * context = getContext();
    m_tetraForceField = context->get< TetrahedronFEMForceField< DataTypes >  >();

    if(m_tetraForceField != nullptr)
    {
        msg_info()<<"Found tetrahedronFEMForceField named "<<m_tetraForceField->getName();
        const VecReal& youngModulus = m_tetraForceField->d_youngModulus.getValue();
        m_youngModulus = youngModulus[0];
        m_initialYoungModulus = m_youngModulus;
        initLimit();
    }
    else
    {
        msg_error()<<"No TetrahedronFEMForceField found";
        d_componentState = ComponentState::Invalid;
    }
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::reset()
{
    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    m_youngModulus = m_initialYoungModulus;
    initLimit();
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                            DataMatrixDeriv &cMatrix,
                                                            unsigned int &cIndex,
                                                            const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    d_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(d_constraintIndex);

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    const VecReal& youngModulus = m_tetraForceField->d_youngModulus.getValue();
    m_youngModulus = youngModulus[0];

    // TODO(damien): this seems a bit hacky :) what are the other possibilities.
    VecDeriv force;
    getForce(force, x);
    
    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

    for (unsigned int j=0; j<force.size(); j++)
    {
        force[j] = force[j]*(1.0/m_youngModulus);

        if(force[j].norm() != 0.0)
            rowIterator.setCol(j, force[j]);
    }
    cIndex++;

    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::getForce(VecDeriv& force,
                                               const DataVecCoord &x)
{
    DataVecDeriv f;
    VecDeriv f0;
    f0.resize(x.getValue().size());
    f.setValue(f0);

    // AddForce(): computes internal forces with respect to given positions and known rest positions.
    //             The velocities v are not used in the computation.
    // void TetrahedronFEMForceField<DataTypes>::addForce (const core::MechanicalParams* /*mparams*/,
    //                                                      DataVecDeriv& d_f,
    //                                                      const DataVecCoord& d_x,
    //                                                      const DataVecDeriv& /*d_v*/)
    DataVecDeriv v;
    m_tetraForceField->addForce(nullptr, f, x, v);
    force = f.getValue();
}



template<class DataTypes>
void YoungModulusActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                             BaseVector *resV,
                                                             const BaseVector *Jdx)
{
    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    SOFA_UNUSED(Jdx);
    SOFA_UNUSED(cParams);
    
    resV->set(d_constraintIndex.getValue(), 0.0);
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    SOFA_UNUSED(delta);

    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    VecReal &youngModulus = *m_tetraForceField->d_youngModulus.beginEdit();

    m_previousYoungValue = youngModulus[0];
    youngModulus[0] += Real(lambda[0]);
    m_youngModulus = youngModulus[0];

    m_tetraForceField->d_youngModulus.endEdit();
    m_tetraForceField->reinit();

    updateLimit();

    Actuator<DataTypes>::storeResults(lambda, delta);
}


} // namespace sofa::component::constraintset





