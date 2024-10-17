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

#include <SoftRobots.Inverse/component/constraint/PositionEffector.h>

namespace softrobotsinverse::constraint
{
    using softrobotsinverse::constraint::PositionEffector ;
    using sofa::core::ConstraintParams ;
    using sofa::linearalgebra::BaseVector ;
    using sofa::core::visual::VisualParams ;
    using sofa::type::Vec ;

/**
 * The "BarycentricCenterEffector" component is used to constrain the barycenter of a model
 * to reach a desired position, by acting on chosen actuator(s).
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template<class DataTypes>
class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector : public PositionEffector<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(BarycentricCenterEffector,DataTypes),
               SOFA_TEMPLATE(PositionEffector,DataTypes));

    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord       Coord;
    typedef typename DataTypes::Deriv       Deriv;
    typedef typename Coord::value_type      Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef sofa::Data<VecCoord>		 DataVecCoord;
    typedef sofa::Data<VecDeriv>		 DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>    DataMatrixDeriv;

public:
    BarycentricCenterEffector(MechanicalState* object = nullptr);
    virtual ~BarycentricCenterEffector() override;

    ////////////////////////// Inherited from BaseObject ////////////////////
    /// According to BaseObject::init
    /// this method should be used to initialize the object during the top-down
    /// traversal of graph creation and modification,
    void init() override;

    /// According to BaseObject::draw
    /// this method should be used to render internal data of this object,
    /// for debugging purposes.
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord & x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

protected:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring m_state in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// using the "this->" approach.
    using Effector<DataTypes>::m_state ;
    using Effector<DataTypes>::getContext ;
    using Effector<DataTypes>::m_nbLines ;
    using Effector<DataTypes>::d_constraintIndex ;
    using Effector<DataTypes>::f_listening ;
    using Effector<DataTypes>::getTarget ;
    using PositionEffector<DataTypes>::d_useDirections ;
    using PositionEffector<DataTypes>::d_directions ;
    using PositionEffector<DataTypes>::d_effectorGoal ;
    using PositionEffector<DataTypes>::d_delta ;
    using PositionEffector<DataTypes>::d_weight ;
    using PositionEffector<DataTypes>::d_indices ;
    using PositionEffector<DataTypes>::d_componentState ;

    SOFA_ATTRIBUTE_DEPRECATED("v24.12", "v25.06", "Use d_useDirections instead.")
    sofa::Data<Vec<3,bool> > d_axis;

    SOFA_ATTRIBUTE_DEPRECATED("v24.12", "v25.06", "Use d_effectorGoal instead.")
    sofa::Data<Coord >       d_effectorGoalPosition;

    sofa::Data<bool>         d_drawBarycenter;
    sofa::Data<Coord >       d_barycenter;

    void computeBarycenter();
    void setBarycenter(const Coord& barycenter){d_barycenter.setValue(barycenter);}
};

template<> SOFA_SOFTROBOTS_INVERSE_API
void BarycentricCenterEffector<sofa::defaulttype::Rigid3Types>::draw(const VisualParams* vparams);

template<> SOFA_SOFTROBOTS_INVERSE_API
void BarycentricCenterEffector<sofa::defaulttype::Rigid3Types>::setBarycenter(const Coord& barycenter);

#if !defined(SOFTROBOTS_INVERSE_BARYCENTRICCENTEREFFECTOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<sofa::defaulttype::Vec3Types >;
extern template class SOFA_SOFTROBOTS_INVERSE_API BarycentricCenterEffector<sofa::defaulttype::Rigid3Types >;
#endif


} // namespace

