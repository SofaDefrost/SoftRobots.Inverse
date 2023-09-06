/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: Eulalie Coevoet                                                    *
*                                                                             *
* Contact information: eulalie.coevoet@inria.fr                               *
******************************************************************************/
#ifndef SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_INL
#define SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_INL

#include "BeamRestPositionActuator.h"

#include <sofa/helper/gl/template.h>
#include <sofa/helper/rmath.h>


namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::helper::gl::glVertexT;
using sofa::defaulttype::Quat;
using sofa::defaulttype::Vec;
using sofa::defaulttype::Mat;
using std::cout;
using std::endl;
using defaulttype::StdRigidTypes;
using sofa::helper::rsin;
using sofa::helper::rcos;
using sofa::helper::rsqrt;
using sofa::helper::SQR;
using std::acos;
using core::VecCoordId;



template<class DataTypes>
BeamRestPositionActuator<DataTypes>::BeamRestPositionActuator(MechanicalState* object)
    : Inherit(object)

    , d_actuationDirection(initData(&d_actuationDirection, Vec<3,bool>(1,1,1), "direction","Direction of actuation. Default y and z i.e (1,1).\n"
                                    "If no direction given i.e (0,0,0): (1,1,1) will be considered."))
{
}

template<class DataTypes>
BeamRestPositionActuator<DataTypes>::BeamRestPositionActuator()
    : d_actuationDirection(initData(&d_actuationDirection, Vec<3,bool>(1,1,1), "direction","Direction of actuation. Default y and z i.e (1,1).\n"
                                    "If no direction given i.e (0,0,0): (1,1,1) will be considered."))
{
}


template<class DataTypes>
BeamRestPositionActuator<DataTypes>::~BeamRestPositionActuator()
{
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::init()
{
    Inherit::init();
    initLimit();
    initBeamLenght();
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::reinit()
{
    initLimit();
    initBeamLenght();
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::initLimit()
{
    //TODO: consider limits
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::initBeamLenght()
{
    ReadAccessor<Data<VecCoord> > restPosition = m_state->read(core::VecCoordId::restPosition());

    for (int i=1; i<m_state->getSize(); i++)
    {
        double length = getBeamLength(restPosition[i-1], restPosition[i]);
        m_beamLengthList.push_back(length);
    }
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::bwdInit()
{
    throw_when(m_state == nullptr)  << "BeamRestPositionActuator needs a context with a mechanical object" ;

    core::objectmodel::BaseContext * context = getContext();

    m_beamForceField = context->get< sofa::component::forcefield::AdaptiveBeamForceFieldAndMass< DataTypes >  >();

    if(m_beamForceField != nullptr)
    {
        sout<<"AdaptiveBeamForceFieldAndMass named : "<<m_beamForceField->getName()<<" found " <<sendl;
    }
    else
    {
        serr<<"No AdaptiveBeamForceFieldAndMass found"<<sendl;
        throw_runtime << "BeamRestPositionActuator needs a context with a AdaptiveBeamForceFieldAndMass" ;
    }

    initBeamLenght();
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                DataMatrixDeriv &column_d,
                                                                unsigned int &columnIndex,
                                                                const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);

    m_columnId = columnIndex;

    MatrixDeriv& column = *column_d.beginEdit();
    column.begin();

    int index = 0;
    double eps = 0.000001;

    if(d_actuationDirection.getValue()[1]) // Actuation around y axis
    {
        for (unsigned int i=1; i<m_state->getSize(); i++)
        {
            // Get the Jacobian for each point
            Deriv dy = Deriv(Vec3(0.,0.,0.),Vec3(0.,eps,0.));
            Deriv Jy;
            getLocalJacobian(Jy, dy, i, eps, x);

            // Fill the constraint matrix
            MatrixDerivRowIterator rowIterator_dy = column.writeLine(m_columnId+index++);
            rowIterator_dy.setCol(i, Jy);

            columnIndex++;
        }
    }

    if(d_actuationDirection.getValue()[2]) // Actuation around z axis
    {
        for (unsigned int i=1; i<m_state->getSize(); i++)
        {
            // Get the Jacobian for each point
            Deriv dz = Deriv(Vec3(0.,0.,0.),Vec3(0.,0.,eps));
            Deriv Jz;
            getLocalJacobian(Jz, dz, i, eps, x);

            // Fill the constraint matrix
            MatrixDerivRowIterator rowIterator_dz = column.writeLine(m_columnId+index++);
            rowIterator_dz.setCol(i, Jz);

            columnIndex++;
        }
    }

    if(d_actuationDirection.getValue()[0]) // Actuation around x axis
    {
        for (unsigned int i=1; i<m_state->getSize(); i++)
        {
            // Get the Jacobian for each point
            Deriv dx = Deriv(Vec3(0.,0.,0.),Vec3(eps,0.,0.));
            Deriv Jx;
            getLocalJacobian(Jx, dx, i, eps, x);

            // Fill the constraint matrix
            MatrixDerivRowIterator rowIterator_dx = column.writeLine(m_columnId+index++);
            rowIterator_dx.setCol(i, Jx);

            columnIndex++;
        }
    }

    column_d.endEdit();
    m_nbLines = columnIndex - m_columnId;
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::getLocalJacobian(Deriv &J,
                                                           Deriv &dr,
                                                           int index,
                                                           double eps,
                                                           const DataVecCoord &x)
{
    // Call to force field
    // (eulalie)TODO:
    // Find the analytical formulation of the force derivative.
    // For now we compute it numerically

    WriteAccessor<Data<VecCoord> > restPosition = m_state->write(VecCoordId::restPosition());
    Coord restPositionCopy = restPosition[index];
    VecDeriv force;

    // F(x,x0+t*dx0)
    applyCurvingRotationToBeam(restPosition[index-1], restPosition[index], m_beamLengthList[index-1], dr);
    //m_beamForceField->applyRotationToBeamRestPosition(index,dr);

    addForce(force, x);
    J = force[index];

    // F(x,x0+t*dx0) - F(x,x0)
    restPosition[index] = restPositionCopy;
    addForce(force, x);
    J -= force[index];

    // (F(x,x0+t*dx0) - F(x,x0))/t
    J /= eps;
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::applyStraightRotationToBeam(const Coord &p0,
                                                                      Coord &p1,
                                                                      const Deriv &dr)
{
    // Apply rotation to the quaternion
    p1 += dr;

    // Apply straight rotation to the beam
    Quat q = StdRigidTypes<3,Real>::rotationEuler(dr[3],dr[4],dr[5]);
    Mat<3,3,Real> M;
    q.toMatrix(M);
    p1.getCenter() = p0.getCenter() + M*(p1.getCenter() - p0.getCenter());
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::applyCurvingRotationToBeam(const Coord &p0,
                                                                     Coord &p1,
                                                                     const double length,
                                                                     const Deriv &dr)
{
    // Apply rotation to the quaternion
    p1 += dr;

    // Get local p0 and p1 rotation matrices
    Quat q0 = StdRigidTypes<3,Real>::rotationEuler(p0[3],p0[4],p0[5]);
    Mat<3,3,Real> M0;
    q0.toMatrix(M0);

    Quat q1 = StdRigidTypes<3,Real>::rotationEuler(p1[3],p1[4],p1[5]);
    Mat<3,3,Real> M1;
    q1.toMatrix(M1);

    Vec3 v  = Vec3(0.,1.,0.);
    //Vec3 v  = Vec3(0,abs(dr[5]/dr.norm()),abs(dr[4]/dr.norm()));
    Vec3 v0 = M0*v;
    v0.normalize();
    Vec3 v1 = M1*v;
    v1.normalize();

    // Curving rotation of the beam that leaves the beam length invariant
    // Assumptions:
    // A beam is always along local x axis
    // The beam curve is always around local z axis
    if (v0*v1<=1e-14)
    {
        // Straight case
        p1.getCenter() = p0.getCenter() + Vec3(length,0.,0.);
    }
    else
    {
        double theta  = acos(v0*v1);
        if (theta<=1e-14)
        {
            serr<<"Attempt to divide by zero in applyCurvingRotationToBeam"<<sendl;
            return;
        }

        double radius = length/theta;
        double coeff  = rsqrt(SQR(radius-radius*rcos(theta))+SQR(radius*rsin(theta)));

        double alpha  = (M_PI - theta)/2.;

        Quat q = StdRigidTypes<3,Real>::rotationEuler(0.,0.,alpha);
        //Quat q = StdRigidTypes<3,Real>::rotationEuler(0,alpha*abs(dr[4]/dr.norm()),alpha*abs(dr[5]/dr.norm()));
        Mat<3,3,Real> M;
        q.toMatrix(M);

        p1.getCenter() = p0.getCenter() - (M*v0)*coeff;
    }
}


template<class DataTypes>
double BeamRestPositionActuator<DataTypes>::getBeamLength(const Coord &p0,
                                                          const Coord &p1)
{
    // Assumptions:
    // A beam is always along local x axis
    // The beam curve is always around local z axis
    double length;

    Vec3 v = Vec3(0.,1.,0.);

    Quat q = StdRigidTypes<3,Real>::rotationEuler(p0[3],p0[4],p0[5]);
    Mat<3,3,Real> M;
    q.toMatrix(M);

    Vec3 v1 = M*v;
    Vec3 v2 = p1.getCenter()-p0.getCenter();
    v2.normalize();

    double alpha  = acos(v1*v2);
    double theta  = M_PI - 2.*alpha;
    double radius = (p1-p0).getCenter().norm()*rsin(alpha)/rsin(theta);
    length = theta*radius;

    return length;
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::addForce(VecDeriv& force,
                                                   const DataVecCoord &x)
{
    DataVecDeriv f;
    VecDeriv f0;
    f0.resize(x.getValue().size());
    f.setValue(f0);

    // AddForce(): computes internal forces with respect to given positions and known rest positions.
    //             The velocities v are not used in the computation.
    // void BeamFEMForceField<DataTypes>::addForce (const core::MechanicalParams* /*mparams*/,
    //                                              DataVecDeriv& d_f,
    //                                              const DataVecCoord& d_x,
    //                                              const DataVecDeriv& /*d_v*/)
    DataVecDeriv v;
    m_beamForceField->addForce(nullptr, f, x, v);
    force = f.getValue();
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                 BaseVector *resV,
                                                                 const DataVecCoord &x,
                                                                 const DataVecDeriv &v)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);
    SOFA_UNUSED(v);

    resV->set(m_columnId, 0.0);
}


template<class DataTypes>
void BeamRestPositionActuator<DataTypes>::storeResults(vector<double>& lambda,
                                                       vector<double>& delta)
{
    SOFA_UNUSED(delta);

    // Update the rest position by applying lambda
    WriteAccessor<Data<VecCoord> > restPosition = m_state->write(core::VecCoordId::restPosition());

    int index = 0;

    int nbBeams = m_state->getSize()-1;
    for (unsigned int beamId=0; beamId<nbBeams; beamId++)
    {
        int pointId = beamId+1;
        if(d_actuationDirection.getValue()[1]) // Actuation around y axis
        {
            Deriv dy = Deriv(Vec3(0.,0.,0.),Vec3(0.,lambda[index],0.));
            applyCurvingRotationToBeam(restPosition[pointId-1], restPosition[pointId], m_beamLengthList[beamId], dy);
            //m_beamForceField->applyRotationToBeamRestPosition(pointId,dy);
            index++;
        }

        if(d_actuationDirection.getValue()[2]) // Actuation around z axis
        {
            Deriv dz = Deriv(Vec3(0.,0.,0.),Vec3(0.,0.,lambda[index]));
            applyCurvingRotationToBeam(restPosition[pointId-1], restPosition[pointId], m_beamLengthList[beamId], dz);
            //m_beamForceField->applyRotationToBeamRestPosition(pointId,dz);
            index++;
        }

        if(d_actuationDirection.getValue()[0]) // Actuation around x axis
        {
            Deriv dx = Deriv(Vec3(0.,0.,0.),Vec3(lambda[index],0.,0.));
            applyCurvingRotationToBeam(restPosition[pointId-1], restPosition[pointId], m_beamLengthList[beamId], dx);
            //m_beamForceField->applyRotationToBeamRestPosition(pointId,dx);
            index++;
        }
    }
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_INL
