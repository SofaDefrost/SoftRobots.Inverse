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

#include <sofa/core/visual/VisualParams.h>

#include <SoftRobots.Inverse/component/constraint/BarycentricCenterEffector.h>


namespace softrobotsinverse::constraint
{

using sofa::core::ConstVecCoordId;
using sofa::type::RGBAColor;
using sofa::helper::ReadAccessor;

using sofa::core::objectmodel::BaseContext;
using sofa::core::collision::ContactManager;
using sofa::type::vector;
using sofa::type::vector;
using sofa::type::Vec3;

template<class DataTypes>
BarycentricCenterEffector<DataTypes>::BarycentricCenterEffector(MechanicalState* object)
    : Inherit1(object)
    , d_axis(initData(&d_axis, sofa::type::Vec<3,bool>(true,true,true), "axis",
                      "The parameter axis is of type Vec3<bool> and allows to specify the directions in \n"
                      "which you want to solve the effector. If unspecified, the default    \n"
                      "values are {true, true, true})."))

    , d_effectorGoalPosition(initData(&d_effectorGoalPosition,"effectorGoal",
                                      "The parameter effectorGoal allows to specifiy the desired position of the\n"
                                      "constraint. If unspecified the default values are {0.0,0.0,0.0}" ))

    , d_drawBarycenter(initData(&d_drawBarycenter,false,"drawBarycenter",
                                "If true, draw the barycenter" ))

    , d_barycenter(initData(&d_barycenter,"barycenter",
                            "Position of barycenter." ))

    , d_delta(initData(&d_delta,"delta",
                            "Distance to target" ))
{
}

template<class DataTypes>
BarycentricCenterEffector<DataTypes>::~BarycentricCenterEffector()
{
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::init()
{
    Inherit1::init();

    if(m_state==nullptr)
        msg_error() << "There is no mechanical state associated with this node. "
                       "the object is deactivated. "
                       "To remove this error message fix your scene possibly by "
                       "adding a MechanicalObject." ;
    initData();

    d_drawBarycenter.setGroup("Visualization");
    d_barycenter.setReadOnly(true);

    computeBarycenter();
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::reinit()
{
    initData();
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::reset()
{
    computeBarycenter();
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::computeBarycenter()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    const unsigned int nbp = m_state->getSize();
    Coord barycenter = Coord();
    for (unsigned int i=0; i<nbp; i++)
    {
        barycenter[0] += positions[i][0]/Real(nbp);
        barycenter[1] += positions[i][1]/Real(nbp);
        barycenter[2] += positions[i][2]/Real(nbp);
    }
    d_barycenter.setValue(barycenter);
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::initData()
{
    if(!d_effectorGoalPosition.isSet())
    {
        d_effectorGoalPosition.setValue(Coord());
        msg_warning() << "EffectorGoal is not defined. No target is given. Set default to (0.0,0.0,0.0). ";
    }

    if(!d_axis.getValue()[0] && !d_axis.getValue()[1] && !d_axis.getValue()[2])
    {
        d_axis.setValue(sofa::type::Vec<3,bool>(true,true,true));
        msg_warning() << "Axis = (0, 0, 0). No direction given. Set default to (1, 1, 1).";
    }
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                 DataMatrixDeriv &cMatrix,
                                                                 unsigned int &cIndex,
                                                                 const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    const unsigned int nbp = m_state->getSize();

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    unsigned int index = 0;

    if(d_axis.getValue()[0])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(1.0/Real(nbp), 0, 0));
        index++;
    }

    if(d_axis.getValue()[1])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(0, 1.0/Real(nbp), 0));
        index++;
    }

    if(d_axis.getValue()[2])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(0, 0, 1.0/Real(nbp)));
        index++;
    }

    cIndex+=index;

    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;

    computeBarycenter();
}

template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                  BaseVector *resV,
                                                                  const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);

    const unsigned int nbp = m_state->getSize();
    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();

    Coord barycenter = Coord();
    for (unsigned int i=0; i<nbp; i++)
    {
        barycenter[0] += x[i][0]/Real(nbp);
        barycenter[1] += x[i][1]/Real(nbp);
        barycenter[2] += x[i][2]/Real(nbp);
    }

    Coord effectorGoal = getTarget(d_effectorGoalPosition.getValue(), barycenter);
    Coord dFree = barycenter - effectorGoal;
    for(unsigned int i=0; i<m_nbLines; i++)
        dFree[i] += Jdx->element(i);

    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    int index = 0;
    if(d_axis.getValue()[0])
    {
        resV->set(constraintIndex, dFree[0]);
        index++;
    }

    if(d_axis.getValue()[1])
    {
        resV->set(constraintIndex+index, dFree[1]);
        index++;
    }

    if(d_axis.getValue()[2])
    {
        resV->set(constraintIndex+index, dFree[2]);
        index++;
    }
}


template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::storeResults(vector<double> &delta)
{
    d_delta.setValue(delta);
}


template<class DataTypes>
void BarycentricCenterEffector<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields()) return;
    if(!d_drawBarycenter.getValue()) return;

    Coord barycenter = d_barycenter.getValue();
    vector<Vec3> points;
    points.push_back(Vec3(barycenter[0], barycenter[1], barycenter[2]));
    vparams->drawTool()->drawPoints(points, float(5.), RGBAColor(0.0f,0.0f,1.0f,1.0f));
}

} // namespace
