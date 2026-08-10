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

#include <SoftRobots.Inverse/component/constraint/YoungModulusActuator.h>


namespace softrobotsinverse::constraint
{

using sofa::type::vector;
using sofa::core::objectmodel::BaseContext;
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

    , l_forceField(initLink("forceField", "link to the force field"))

    , m_initialYoungModulus(0.0)
    , m_initError(false)
    , m_deltaYoungModulus(0.0)
{
    this->d_lambda.setHelp("Optimized Young modulus");
    this->d_lambda.setName("youngModulus");
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
    const auto& youngModulus = sofa::helper::getReadAccessor(this->d_lambda);
    const auto& minYoung = sofa::helper::getReadAccessor(d_minYoung);
    const auto& maxYoung = sofa::helper::getReadAccessor(d_maxYoung);
    const auto& maxYoungVariationRatio = sofa::helper::getReadAccessor(d_maxYoungVariationRatio);

    m_lambdaMin[0] =- (youngModulus[0] - minYoung);
    m_lambdaMax[0] = maxYoung - youngModulus[0];

    double youngMin = youngModulus[0] - youngModulus[0] * maxYoungVariationRatio;
    if(youngMin >= minYoung)
        m_lambdaMin[0] =- youngModulus[0] * maxYoungVariationRatio;

    double youngMax = youngModulus[0] + youngModulus[0] * maxYoungVariationRatio;
    if(youngMax <= maxYoung)
        m_lambdaMax[0] = youngModulus[0] * maxYoungVariationRatio;
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::bwdInit()
{
    BaseContext * context = getContext();

    if (l_forceField.empty() || l_forceField.get()==nullptr)
    {
        l_forceField.set(context->get< BaseLinearElasticityFEMForceField< DataTypes >>());
        if (!l_forceField.empty() && l_forceField.get()!=nullptr)
        {
            msg_info() << "Found force field named " << l_forceField->getName();
            m_initialYoungModulus = l_forceField->d_youngModulus.getValue()[0];

            auto youngModulus = sofa::helper::getWriteAccessor(this->d_lambda);
            youngModulus[0] = m_initialYoungModulus;
            d_youngModulus.setValue(m_initialYoungModulus);
            initLimit();
        }
        else
        {
            msg_error() << "No force field found";
            d_componentState = ComponentState::Invalid;
        }
    }
}


template<class DataTypes>
void YoungModulusActuator<DataTypes>::reset()
{
    if(d_componentState.getValue() == ComponentState::Invalid)
        return;

    auto youngModulus = sofa::helper::getWriteAccessor(this->d_lambda);
    youngModulus[0] = m_initialYoungModulus;
    d_youngModulus.setValue(m_initialYoungModulus);
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

    // TODO(damien): this seems a bit hacky :) what are the other possibilities.
    VecDeriv force;
    getForce(force, x);
    
    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);
    const auto& youngModulus = l_forceField->d_youngModulus.getValue()[0];
    auto lambda = sofa::helper::getWriteAccessor(this->d_lambda);
    lambda[0] = youngModulus;
    d_youngModulus.setValue(youngModulus);

    for (unsigned int j=0; j<force.size(); j++)
    {
        force[j] = force[j]*(1.0/youngModulus);

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
    l_forceField->addForce(nullptr, f, x, v);
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

    auto l = sofa::helper::getWriteAccessor(this->d_lambda);
    l[0] += Real(lambda[0]);
    Real youngModulus = sofa::helper::getWriteAccessor(d_youngModulus);
    youngModulus += Real(lambda[0]);

    if (d_applyForce.getValue())
    {
        l_forceField->setYoungModulus(youngModulus);
        l_forceField->reinit();
    }

    updateLimit();

    Actuator<DataTypes>::storeResults(lambda, delta);
}


} // namespace sofa::component::constraintset





