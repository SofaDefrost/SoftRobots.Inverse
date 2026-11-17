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

#include <SoftRobots.Inverse/component/constraint/SpringStiffnessActuator.h>


namespace softrobotsinverse::constraint
{

using sofa::type::vector;
using sofa::core::objectmodel::BaseContext;
using sofa::core::objectmodel::ComponentState;

template<class DataTypes>
SpringStiffnessActuator<DataTypes>::SpringStiffnessActuator(MechanicalState* object)
    : Inherit1(object)

    , d_minValue(initData(&d_minValue, (Real)1.0e1, "minValue",
                   "Lower bound on the spring stiffness. The optimizer will "
                   "never drive a spring's stiffness below this value."))

    , d_maxValue(initData(&d_maxValue, (Real)1.0e5, "maxValue",
                   "Upper bound on the spring stiffness. The optimizer will "
                   "never drive a spring's stiffness above this value."))

    , l_forceField(initLink("forceField",
                   "Path to the SpringForceField whose springs are actuated. "
                   "If unset, the component falls back to the first "
                   "SpringForceField<DataTypes> found in the same context — an "
                   "explicit link is recommended when several exist."))

    , m_initialValues(0.0)
{
    this->setLambdaHelp("The current optimized stiffness of each spring, one "
                        "per spring of the linked force field.");
}

template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::init()
{
    this->d_componentState = ComponentState::Valid;
    Inherit1::init();

    if (l_forceField.empty() || l_forceField.get()==nullptr)
    {
        l_forceField.set(this->getContext()->get< sofa::component::solidmechanics::spring::SpringForceField< DataTypes >>());
    }
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::reinit()
{
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::initLimit()
{
    this->m_hasLambdaMin = true;
    this->m_hasLambdaMax = true;

    updateLimit();
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::updateLimit()
{
    const auto& values = sofa::helper::getReadAccessor(this->d_lambda);
    const auto& minValue = sofa::helper::getReadAccessor(d_minValue);
    const auto& maxValue = sofa::helper::getReadAccessor(d_maxValue);

    this->m_lambdaMax.resize(values.size());
    this->m_lambdaMin.resize(values.size());

    for (std::size_t i = 0; i < values.size(); ++i)
    {
        this->m_lambdaMin[i] = -(values[i] - minValue);
        this->m_lambdaMax[i] = maxValue - values[i];
    }
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::bwdInit()
{
    if (!l_forceField.empty() && l_forceField.get()!=nullptr)
    {
        const auto springs = sofa::helper::getReadAccessor(l_forceField->d_springs);
        m_initialValues.clear();
        m_initialValues.reserve(springs.size());
        for (const auto& spring : springs)
        {
            m_initialValues.push_back(spring.ks);
        }

        this->d_lambda.setValue(m_initialValues);
        initLimit();
    }
    else
    {
        msg_error() << "No force field found";
        this->d_componentState = ComponentState::Invalid;
    }
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::reset()
{
    if(this->d_componentState.getValue() == ComponentState::Invalid)
        return;

    this->d_lambda.setValue(m_initialValues);
    initLimit();
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::buildConstraintMatrix(const sofa::core::ConstraintParams* cParams,
                                                            DataMatrixDeriv &cMatrix,
                                                            unsigned int &cIndex,
                                                            const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);

    if(this->d_componentState.getValue() == ComponentState::Invalid)
        return;

    this->d_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(this->d_constraintIndex);

    auto matrix = sofa::helper::getWriteAccessor(cMatrix);

    std::size_t stateId = 0;
    if (l_forceField->getMState1() == this->m_state)
    {
        stateId = 0;
    }
    else if (l_forceField->getMState2() == this->m_state)
    {
        stateId = 1;
    }
    else
    {
        msg_error() << "invalid x parameter";
        return;
    }

    const auto springs = sofa::helper::getReadAccessor(l_forceField->d_springs);
    auto values = sofa::helper::getWriteOnlyAccessor(this->d_lambda);
    if (values.size() < springs.size())
    {
        values.resize(springs.size());
    }

    const auto& p1 = l_forceField->getMState1()->readPositions().ref();
    const auto& p2 = l_forceField->getMState2()->readPositions().ref();

    for (std::size_t i = 0; i < springs.size(); ++i)
    {
        MatrixDerivRowIterator rowIterator = matrix->writeLine(constraintIndex + i);

        const auto spring = springs[i];

        const auto stiffness = spring.ks;
        values[i] = spring.ks;

        const sofa::Index a = spring.m1;
        const sofa::Index b = spring.m2;

        typename DataTypes::CPos u = DataTypes::getCPos(p2[b]) - DataTypes::getCPos(p1[a]);
        const Real d = u.norm();

        u /= d; //potential division by zero

        const Real elongation = d - spring.initpos;
        Deriv dfdk;
        DataTypes::setDPos(dfdk, elongation * u);

        if (stateId == 0)
        {
            rowIterator.setCol(a, dfdk);
        }
        else
        {
            rowIterator.setCol(b, -dfdk);
        }
        cIndex++;
    }

    cMatrix.endEdit();
    this->m_nbLines = cIndex - constraintIndex;
}

template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::getConstraintViolation(const sofa::core::ConstraintParams* cParams,
                                                             sofa::linearalgebra::BaseVector *resV,
                                                             const sofa::linearalgebra::BaseVector *Jdx)
{
    if(this->d_componentState.getValue() == ComponentState::Invalid)
        return;

    SOFA_UNUSED(Jdx);
    SOFA_UNUSED(cParams);

    const auto springs = sofa::helper::getReadAccessor(l_forceField->d_springs);
    const auto& constraintIndex = sofa::helper::getReadAccessor(this->d_constraintIndex);
    for (std::size_t i = 0; i < springs.size(); i++)
    {
        resV->set(constraintIndex + i, 0.0);
    }
}


template<class DataTypes>
void SpringStiffnessActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    SOFA_UNUSED(delta);

    if(this->d_componentState.getValue() == ComponentState::Invalid)
        return;

    if (this->d_applyForce.getValue())
    {
        auto springs = sofa::helper::getWriteOnlyAccessor(l_forceField->d_springs);
        const auto values = sofa::helper::getReadAccessor(this->d_lambda);

        if (springs.size() < values.size())
        {
            springs.resize(values.size());
        }

        for (std::size_t i = 0; i < std::min(springs.size(), lambda.size()); i++)
        {
            springs[i].ks = values[i] + static_cast<Real>(lambda[i]);
        }

        l_forceField->reinit();
    }

    updateLimit();

    behavior::Actuator<DataTypes>::storeResults(lambda, delta);
}


}





