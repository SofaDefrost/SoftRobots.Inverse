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

#include <SoftRobots.Inverse/component/behavior/Actuator.h>
#include <sofa/core/topology/BaseMeshTopology.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::constraint
{
    using sofa::core::behavior::Actuator;
    using sofa::core::topology::BaseMeshTopology;
    using sofa::core::visual::VisualParams;
    using sofa::core::ConstraintParams;
    using sofa::linearalgebra::BaseVector;
    using sofa::helper::ReadAccessor;
    using sofa::core::ConstVecCoordId;

/**
 * This component is used to solve an inverse problem by applying a force on a given surface of a model.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
*/
template< class DataTypes >
class ForceSurfaceActuator : public Actuator<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(ForceSurfaceActuator,DataTypes), SOFA_TEMPLATE(Actuator,DataTypes));

    typedef typename DataTypes::VecCoord                    VecCoord;
    typedef typename DataTypes::VecDeriv                    VecDeriv;
    typedef typename DataTypes::Coord                       Coord;
    typedef typename DataTypes::Deriv                       Deriv;
    typedef typename DataTypes::MatrixDeriv                 MatrixDeriv;
    typedef typename Coord::value_type                      Real;

    typedef typename sofa::core::topology::BaseMeshTopology::Triangle      Triangle;
    typedef typename sofa::core::topology::BaseMeshTopology::Quad          Quad;
    typedef typename sofa::core::topology::BaseMeshTopology::Edge          Edge;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename DataTypes::MatrixDeriv::RowIterator    MatrixDerivRowIterator;
    typedef sofa::Data<VecCoord>                                  DataVecCoord;
    typedef sofa::Data<VecDeriv>                                  DataVecDeriv;
    typedef sofa::Data<MatrixDeriv>                               DataMatrixDeriv;


public:
    ForceSurfaceActuator(MechanicalState* = nullptr);
    ~ForceSurfaceActuator() override;

    /////////////// Inherited from BaseObject ////////////////////
    void init() override;
    void reinit() override;
    void draw(const VisualParams* vparams) override;
    /////////////////////////////////////////////////////////////

    ///////// Inherited from SoftRobotsConstraint ////////////
    void buildConstraintMatrix(const ConstraintParams* cParams ,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams ,
                                BaseVector *resV,
                                const BaseVector *Jdx) override;
    /////////////////////////////////////////////////////////////////////////

    /////////////// Inherited from BaseSoftRobotsConstraint ////////////////
    void storeResults(sofa::type::vector<double> &lambda,
                      sofa::type::vector<double> &delta) override;
    /////////////////////////////////////////////////////////////


protected:

    sofa::Data<VecCoord>                      d_centers;
    VecCoord                                  m_initialCenters;
    sofa::Data<sofa::type::vector<Real>>      d_radii;
    sofa::Data<VecDeriv>                      d_directions;
    sofa::Data<bool>                          d_updateNormals;

    sofa::Data<sofa::type::vector<Triangle>>      d_triangles;
    sofa::Data<sofa::type::vector<Quad>>          d_quads;
    sofa::Data<sofa::type::vector<Coord>>         d_positions;
    sofa::type::vector<Edge>                m_edges;

    sofa::Data<Real>                          d_maxForce;
    sofa::Data<Real>                          d_minForce;
    sofa::Data<Real>                          d_maxForceVariation;

    sofa::Data<Real>                          d_maxDisplacement;
    sofa::Data<Real>                          d_minDisplacement;

    sofa::Data<sofa::type::vector<Real>>          d_force;
    sofa::Data<sofa::type::vector<Real>>          d_displacement;

    sofa::Data<bool>                          d_drawForce;
    sofa::Data<bool>                          d_drawSphere;
    sofa::Data<bool>                          d_drawSurface;
    sofa::Data<Real>                          d_visuScale;

    bool                                m_useNormals{false};

    sofa::type::vector<sofa::type::vector<unsigned int>>        m_pointsInSphereId;
    sofa::type::vector<sofa::type::vector<unsigned int>>        m_trianglesInSpheresId;
    sofa::type::vector<sofa::type::vector<unsigned int>>        m_quadsInSpheresId;
    sofa::type::vector<sofa::type::vector<unsigned int>>        m_edgesInSpheresId;
    sofa::type::vector<sofa::type::vector<Real>>                m_ratios;




    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_state ;
    using Actuator<DataTypes>::m_constraintIndex ;
    ////////////////////////////////////////////////////////////////////////////


    void initLimit();
    void initData();
    void updateLimit();
    void updateCenter();

    bool isPointInSphere(unsigned int pointId, unsigned int sphereId);
    bool isTriangleInSphere(unsigned int triangleId, unsigned int sphereId);
    bool isQuadInSphere(unsigned int quadId, unsigned int sphereId);
    bool isEdgeInSphere(unsigned int edgeId, unsigned int sphereId);

    bool isIndexInPointsList(unsigned int index, unsigned int sphereId);

    void computeSurfaces();
    void computePointsInSpheres();
    void computeNormals();
    void computeEdges();

    void drawForces(const VisualParams* vparams);
    void drawSpheres(const VisualParams* vparams);
    void drawSurfaces(const VisualParams* vparams);
    void drawTriangles(const VisualParams* vparams);
    void drawLines(const VisualParams* vparams);

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Actuator<DataTypes>::m_lambdaMax ;
    using Actuator<DataTypes>::m_lambdaMin ;
    using Actuator<DataTypes>::m_hasLambdaMax ;
    using Actuator<DataTypes>::m_hasLambdaMin ;
    using Actuator<DataTypes>::m_deltaMax ;
    using Actuator<DataTypes>::m_deltaMin ;
    using Actuator<DataTypes>::m_hasDeltaMax ;
    using Actuator<DataTypes>::m_hasDeltaMin ;
    using Actuator<DataTypes>::m_nbLines ;
    ////////////////////////////////////////////////////////////////////////////

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#if !defined(SOFTROBOTS_INVERSE_FORCESURFACEACTUATOR_CPP)
extern template class SOFA_SOFTROBOTS_INVERSE_API ForceSurfaceActuator<sofa::defaulttype::Vec3Types>;
#endif

} // namespace

namespace sofa::component::constraintset
{
    template <class DataTypes>
    using ForceSurfaceActuator SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS_INVERSE()
        = softrobotsinverse::constraint::ForceSurfaceActuator<DataTypes>;
}
