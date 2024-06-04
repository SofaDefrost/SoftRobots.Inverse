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
#define SOFTROBOTS_INVERSE_BARYCENTRICCENTEREFFECTOR_CPP

#include <SoftRobots.Inverse/component/constraint/BarycentricCenterEffector.inl>

#include <sofa/core/ObjectFactory.h>

namespace softrobotsinverse::constraint
{
using sofa::defaulttype::Vec3Types;
using sofa::defaulttype::Rigid3Types;
using sofa::core::RegisterObject ;


template<>
void BarycentricCenterEffector<Rigid3Types>::buildConstraintMatrix(const ConstraintParams* cParams,
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
    const Vec<3, Real> cx(1.0/Real(nbp),0,0), cy(0,1.0/Real(nbp),0), cz(0,0,1.0/Real(nbp));
    const Vec<3, Real> vZero(0,0,0);

    if(d_axis.getValue()[0])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(cx,vZero));
        index++;
    }

    if(d_axis.getValue()[1])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(cy,vZero));
        index++;
    }

    if(d_axis.getValue()[2])
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+index);
        for (unsigned int i=0; i<nbp; i++)
            rowIterator.setCol(i, Deriv(cz,vZero));
        index++;
    }

    cIndex+=index;

    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
}


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
int BarycentricCenterEffectorClass = RegisterObject("This component is used to describe one or several desired trajectories "
                                                    "for the barycenter of a model, that will be reached by acting on chosen actuator(s).")
        .add< BarycentricCenterEffector<Vec3Types> >(true)
        .add< BarycentricCenterEffector<Rigid3Types> >()

        ;

template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<Vec3Types>;
template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<Rigid3Types>;

} // namespace
