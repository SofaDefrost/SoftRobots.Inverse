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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_H
#define SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_H

#include "../behavior/Actuator.h"
#include "AdaptiveBeamForceFieldAndMass.h"

namespace sofa
{

namespace component
{

namespace constraintset
{
    using sofa::core::behavior::Actuator;
    using sofa::core::topology::BaseMeshTopology;
    using sofa::core::visual::VisualParams;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::helper::ReadAccessor;
    using sofa::helper::WriteAccessor;
    using sofa::core::ConstVecCoordId;
    using sofa::component::forcefield::AdaptiveBeamForceFieldAndMass;

/**
 * This component is used to solve effector constraint by acting on model rest position.
 * Only works with adaptive beam model.
 * Description can be found at:
 * https://partage.inria.fr/share/page/site/defrost-project-documentation/wiki-page?title=BeamRestPositionActuator#Contents
*/
template< class DataTypes >
class BeamRestPositionActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BeamRestPositionActuator,DataTypes),
               SOFA_TEMPLATE(core::behavior::Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef defaulttype::Vec<3,Real>                        Vec3;

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename core::behavior::Actuator<DataTypes> Inherit;

    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef Data<VecCoord>                                  DataVecCoord;
    typedef Data<VecDeriv>                                  DataVecDeriv;
    typedef Data<MatrixDeriv>                               DataMatrixDeriv;

    typedef typename defaulttype::SolidTypes<Real>::Transform Transform;


public:
    BeamRestPositionActuator();
    BeamRestPositionActuator(MechanicalState*);
    virtual ~BeamRestPositionActuator() ;

    /////////////// Inherited from BaseObject ////////////////////
    virtual void init();
    virtual void reinit();
    virtual void bwdInit();
    /////////////////////////////////////////////////////////////

    /////////////// Inherited from InverseProblemConstraint ///////
    virtual void buildConstraintMatrix(const sofa::core::ConstraintParams* cParams ,
                                       DataMatrixDeriv &c_d,
                                       unsigned int &cIndex,
                                       const DataVecCoord &x);

    virtual void getConstraintViolation(const sofa::core::ConstraintParams* cParams ,
                                        sofa::defaulttype::BaseVector *resV,
                                        const DataVecCoord &x,
                                        const DataVecDeriv &v);
    /////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseInverseProblemConstraint ////////////////////
    void storeResults(vector<double>& lambda, vector<double>& delta) override;
    /////////////////////////////////////////////////////////////

protected:
    unsigned int                        m_columnId;

    Data<defaulttype::Vec<3,bool>>      d_actuationDirection;

    AdaptiveBeamForceFieldAndMass< DataTypes > * m_beamForceField;
    helper::vector<Real> m_beamLengthList;

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_state ;
    ////////////////////////////////////////////////////////////////////////////

private:
    void initLimit();
    void initBeamLenght();

    void getLocalJacobian(Deriv &J,
                          Deriv &dr,
                          int index,
                          double eps,
                          const DataVecCoord &x);

    void applyStraightRotationToBeam(const Coord &p0,
                             Coord &p1,
                             const Deriv &dr);

    void applyCurvingRotationToBeam(const Coord &p0,
                                    Coord &p1,
                                    const double length,
                                    const Deriv &dr);

    double getBeamLength(const Coord &p0,
                         const Coord &p1);

    void addForce(VecDeriv& force,
                  const DataVecCoord &x);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_nbLines ;
    using Actuator<DataTypes>::getContext ;
    ////////////////////////////////////////////////////////////////////////////
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class BeamRestPositionActuator<sofa::defaulttype::Rigid3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_BEAMRESTPOSITIONACTUATOR_H
