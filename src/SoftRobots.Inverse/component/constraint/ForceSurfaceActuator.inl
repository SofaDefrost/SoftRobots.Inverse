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

#include <SoftRobots.Inverse/component/constraint/ForceSurfaceActuator.h>
#include <sofa/core/visual/VisualParams.h>

namespace softrobotsinverse::constraint
{

using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::linearalgebra::BaseVector;
using sofa::core::VecCoordId ;
using sofa::type::vector;
using sofa::type::Vec;
using sofa::type::Vec3;
using sofa::type::Mat;
using sofa::type::RGBAColor;
using sofa::helper::rabs;

template<class DataTypes>
ForceSurfaceActuator<DataTypes>::ForceSurfaceActuator(MechanicalState* object)
    : Inherit1(object)

    , d_centers(initData(&d_centers, "centers",
                         "List of centers describing the ROI spheres."))

    , d_radii(initData(&d_radii, "radii",
                       "List of radii describing the ROI spheres."))

    , d_directions(initData(&d_directions, "directions",
                            "If not set, will computes and uses the normal of the ROIs described on the surface by the sphere ROI."))

    , d_updateNormals(initData(&d_updateNormals, false, "updateNormals",
                               "When considering normals as direction for actuation."))

    , d_triangles(initData(&d_triangles, "triangles",
                           "List of triangles describing the surface.\n"
                           "If no list is given, the component will \n"
                           "fill the list with the context topology."))

    , d_quads(initData (&d_quads, "quads",
                        "List of quads describing the surface. \n"
                        "If no list is given, the component will \n"
                        "fill the list with the context topology."))

    , d_positions(initData (&d_positions, "position",
                            "List of positions describing the surface. \n"
                            "If no list is given, the component will \n"
                            "fill the list with the context topology."))

    , d_maxForce(initData(&d_maxForce, "maxForce",
                          ""))

    , d_minForce(initData(&d_minForce, "minForce",
                          ""))

    , d_maxForceVariation(initData(&d_maxForceVariation, "maxForceVariation",
                                   "Only available if the direction is set."))

    , d_maxDisplacement(initData(&d_maxDisplacement, "maxDisplacement",
                                 ""))

    , d_minDisplacement(initData(&d_minDisplacement, "minDisplacement",
                                 ""))

    , d_force(initData(&d_force, vector<Real>(0), "force",
                       "Warning: to get the actual force you should divide this value by dt."))

    , d_displacement(initData(&d_displacement, vector<Real>(0), "displacement",
                              ""))

    , d_drawForce(initData(&d_drawForce, false, "drawForces",
                           ""))

    , d_drawSphere(initData(&d_drawSphere, false, "drawSpheres",
                            ""))

    , d_drawSurface(initData(&d_drawSurface, false, "drawSurfaces",
                             ""))

    , d_visuScale(initData(&d_visuScale, Real(0.1), "visuScale",
                           ""))

{
    d_force.setReadOnly(true);
    WriteAccessor<sofa::Data<vector<Real>>> force = d_force;
    force.resize(1);
    d_displacement.setReadOnly(true);

    d_drawForce.setGroup("Visualization");
    d_drawSphere.setGroup("Visualization");
    d_drawSurface.setGroup("Visualization");
    d_visuScale.setGroup("Visualization");

    // QP on only one value, we set dimension to one
    m_deltaMax.resize(1);
    m_deltaMin.resize(1);
    m_lambdaMax.resize(1);
    m_lambdaMin.resize(1);
}


template<class DataTypes>
ForceSurfaceActuator<DataTypes>::~ForceSurfaceActuator()
{
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::init()
{
    Inherit1::init();

    if(m_state==nullptr)
        msg_error(this) << "There is no mechanical state associated with this node. "
                           "the object is deactivated. "
                           "To remove this error message fix your scene possibly by "
                           "adding a MechanicalObject." ;

    initData();
    initLimit();
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::reinit()
{
    initData();
    initLimit();
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::initData()
{
    /// Check that the triangles and quads datafield contain something, otherwise get context
    /// topology
    if(d_triangles.getValue().size() == 0 && d_quads.getValue().size() == 0)
    {
        msg_info(this) <<"No triangles and quads given. Get context topology.";
        BaseMeshTopology* topology = this->getContext()->getMeshTopology();

        if(topology==nullptr)
            msg_error(this) << "There is no topology state associated with this node. "
                               "To remove this error message, fix your scene possibly by "
                               "adding a Topology in the parent node or by giving a list of triangles"
                               "indices or a list of quads indices as nodes parameters ." ;

        d_triangles.setValue(topology->getTriangles());
        d_quads.setValue(topology->getQuads());
        m_edges = topology->getEdges();
    }
    else
    {
        computeEdges();
    }


    /// Check that the position datafield contain something, otherwise get context
    /// topology
    WriteAccessor<sofa::Data<VecCoord> > positions = d_positions;
    if(d_positions.getValue().size() == 0)
    {
        msg_info(this) <<"No positions given. Get context mechanical rest position.";
        ReadAccessor<sofa::Data<VecCoord> > restPositions = m_state->readRestPositions();
        positions.resize(restPositions.size());
        for(unsigned int i=0; i<restPositions.size(); i++)
            positions[i] = restPositions[i];
    }


    /// Check that the triangles datafield does not contains indices that would crash the
    /// component.
    int numTris = d_triangles.getValue().size() ;
    auto triangles = d_triangles.getValue() ;
    for(int i=0;i<numTris;i++){
        for(int j=0;j<3;j++){
            if( triangles[i][j] >= positions.size() )
                msg_error(this) << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                                   <<". is too large regarding mechanicalState size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the quads datafield does not contains indices that would crash the
    /// component.
    int numQuads = d_quads.getValue().size() ;
    auto quads = d_quads.getValue() ;
    for(int i=0;i<numQuads;i++){
        for(int j=0;j<4;j++){
            if( quads[i][j] >= positions.size() )
                msg_error(this) << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                                   << " is too large regarding mechanicalState size of("
                                   << positions.size() << ")" ;
        }
    }

    int nbCenters = d_centers.getValue().size();
    int nbRadii = d_radii.getValue().size();
    int nbDirections = d_directions.getValue().size();

    m_initialCenters = d_centers.getValue();

    WriteAccessor<sofa::Data<vector<Real>>> radii = d_radii;

    if(nbRadii == 0)
    {
        radii.resize(1);
        radii[0] = 1;
        nbRadii = 1;
        msg_warning(this) << "No radius given. Set default radius = 1.";
    }

    if(nbCenters != nbRadii)
    {
        double radius = radii[0];
        radii.resize(nbCenters);
        for(int i=0; i<nbCenters; i++)
            radii[i] = radius;
        msg_warning(this) << "Size of centers and radii list not the same. Will apply first raidus to all the spheres.";
    }

    computeSurfaces();

    if(nbCenters != nbDirections)
    {
        m_useNormals = true;
        computeNormals();
        msg_warning(this) << "Size of centers and directions list not the same. Will use discs normal as direction for actuation.";
    }
    else
    {
        WriteAccessor<sofa::Data<VecDeriv>> directions = d_directions;
        for(int i=0; i<nbCenters; i++)
            directions[i].normalize();
    }

}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::initLimit()
{
    if(d_maxForce.isSet())
    {
        m_hasLambdaMax = true;
        m_lambdaMax[0] = d_maxForce.getValue();
    }

    if(d_minForce.isSet())
    {
        m_hasLambdaMin = true;
        m_lambdaMin[0] = d_minForce.getValue();
    }

    if(d_maxForceVariation.isSet())
    {
        m_hasLambdaMax = true;
        m_hasLambdaMin = true;
        if(rabs(m_lambdaMin[0] - d_force.getValue()[0]) >= d_maxForceVariation.getValue() || !d_minForce.isSet())
            m_lambdaMin[0] = d_force.getValue()[0] - d_maxForceVariation.getValue();
        if(rabs(m_lambdaMax[0] - d_force.getValue()[0]) >= d_maxForceVariation.getValue() || !d_maxForce.isSet())
            m_lambdaMax[0] = d_force.getValue()[0] + d_maxForceVariation.getValue();
    }

    if(d_maxDisplacement.isSet())
    {
        m_hasDeltaMax = true;
        m_deltaMax[0] = d_maxDisplacement.getValue();
    }

    if(d_minDisplacement.isSet())
    {
        m_hasDeltaMin = true;
        m_deltaMin[0] = d_minDisplacement.getValue();
    }
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::updateLimit()
{
    if(d_maxForce.isSet())
        m_lambdaMax[0] = d_maxForce.getValue();

    if(d_minForce.isSet())
        m_lambdaMin[0] = d_minForce.getValue();

    if(d_maxForceVariation.isSet())
    {
        if(rabs(m_lambdaMin[0] - d_force.getValue()[0]) >= d_maxForceVariation.getValue() || !d_minForce.isSet())
            m_lambdaMin[0] = d_force.getValue()[0] - d_maxForceVariation.getValue();
        if(rabs(m_lambdaMax[0] - d_force.getValue()[0]) >= d_maxForceVariation.getValue() || !d_maxForce.isSet())
            m_lambdaMax[0] = d_force.getValue()[0] + d_maxForceVariation.getValue();
    }

    if(d_maxDisplacement.isSet())
        m_deltaMax[0] = d_maxDisplacement.getValue();

    if(d_minDisplacement.isSet())
        m_deltaMin[0] = d_minDisplacement.getValue();
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::computeSurfaces()
{
    ReadAccessor<sofa::Data<vector<Triangle> > >  triangles = d_triangles;
    ReadAccessor<sofa::Data<vector<Quad> > >      quads     = d_quads;
    ReadAccessor<sofa::Data<VecCoord> >      centers     = d_centers;

    computePointsInSpheres();

    m_trianglesInSpheresId.clear();
    m_trianglesInSpheresId.resize(centers.size());
    for(unsigned int i=0; i<centers.size(); i++)
        for(unsigned int t=0; t<triangles.size(); t++)
            if(isTriangleInSphere(t,i))
                m_trianglesInSpheresId[i].push_back(t);

    m_quadsInSpheresId.clear();
    m_quadsInSpheresId.resize(centers.size());
    for(unsigned int i=0; i<centers.size(); i++)
        for(unsigned int q=0; q<quads.size(); q++)
            if(isQuadInSphere(q,i))
                m_quadsInSpheresId[i].push_back(q);

    m_edgesInSpheresId.clear();
    m_edgesInSpheresId.resize(centers.size());
    for(unsigned int i=0; i<centers.size(); i++)
        for(unsigned int e=0; e<m_edges.size(); e++)
            if(isEdgeInSphere(e,i))
                m_edgesInSpheresId[i].push_back(e);
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::computePointsInSpheres()
{
    ReadAccessor<sofa::Data<vector<Coord> > >     positions = d_positions;
    ReadAccessor<sofa::Data<VecCoord> >           centers   = d_centers;
    ReadAccessor<sofa::Data<vector<Real>>>        radii     = d_radii;

    m_pointsInSphereId.clear();
    m_pointsInSphereId.resize(centers.size());
    m_ratios.clear();
    m_ratios.resize(centers.size());
    for(unsigned int i=0; i<centers.size(); i++)
        for(unsigned int p=0; p<positions.size(); p++)
            if(isPointInSphere(p,i))
            {
                double ratio = rabs((positions[p]-centers[i]).norm() - radii[i])/radii[i];
                m_ratios[i].push_back(ratio);
                m_pointsInSphereId[i].push_back(p);
            }
}


template<class DataTypes>
bool ForceSurfaceActuator<DataTypes>::isPointInSphere(unsigned int pointId, unsigned int sphereId)
{
    ReadAccessor<sofa::Data<VecCoord>>  positions = d_positions;
    ReadAccessor<sofa::Data<VecCoord>>  centers = d_centers;
    ReadAccessor<sofa::Data<vector<Real>>>  radii = d_radii;

    Coord position = positions[pointId];
    Coord center = centers[sphereId];
    Deriv direction = position - center;

    double norm = direction.norm();
    if(norm<radii[sphereId])
        return true;

    return false;
}


template<class DataTypes>
bool ForceSurfaceActuator<DataTypes>::isTriangleInSphere(unsigned int triangleId, unsigned int sphereId)
{
    ReadAccessor<sofa::Data<vector<Triangle> > >  triangles = d_triangles;

    if(m_pointsInSphereId.size()==0)
        computePointsInSpheres();

    unsigned int id1 = triangles[triangleId][0];
    unsigned int id2 = triangles[triangleId][1];
    unsigned int id3 = triangles[triangleId][2];

    if(isIndexInPointsList(id1, sphereId) || isIndexInPointsList(id2, sphereId) || isIndexInPointsList(id3, sphereId))
        return true;

    return false;
}


template<class DataTypes>
bool ForceSurfaceActuator<DataTypes>::isQuadInSphere(unsigned int quadId, unsigned int sphereId)
{
    ReadAccessor<sofa::Data<vector<Quad> > > quads = d_quads;

    if(m_pointsInSphereId.size()==0)
        computePointsInSpheres();

    unsigned int id1 = quads[quadId][0];
    unsigned int id2 = quads[quadId][1];
    unsigned int id3 = quads[quadId][2];
    unsigned int id4 = quads[quadId][2];

    if(isIndexInPointsList(id1, sphereId) || isIndexInPointsList(id2, sphereId) || isIndexInPointsList(id3, sphereId) || isIndexInPointsList(id4, sphereId))
        return true;

    return false;
}


template<class DataTypes>
bool ForceSurfaceActuator<DataTypes>::isEdgeInSphere(unsigned int edgeId, unsigned int sphereId)
{
    if(m_pointsInSphereId.size()==0)
        computePointsInSpheres();

    int id1 = m_edges[edgeId][0];
    int id2 = m_edges[edgeId][1];

    if(isIndexInPointsList(id1, sphereId) || isIndexInPointsList(id2, sphereId))
        return true;

    return false;
}


template<class DataTypes>
bool ForceSurfaceActuator<DataTypes>::isIndexInPointsList(unsigned int index, unsigned int sphereId)
{
    vector<unsigned int> list = m_pointsInSphereId[sphereId];

    for(unsigned int i=0; i<list.size(); i++)
        if(list[i]==index)
            return true;

    return false;
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::computeNormals()
{
    ReadAccessor<sofa::Data<vector<Triangle> > >  triangles = d_triangles;
    ReadAccessor<sofa::Data<vector<Quad> > >      quads     = d_quads;
    ReadAccessor<sofa::Data<VecCoord>>            centers   = d_centers;
    WriteAccessor<sofa::Data<VecDeriv>>           directions= d_directions;
    ReadAccessor<sofa::Data<VecCoord> >           positions = *m_state->read(ConstVecCoordId::position());

    directions.resize(centers.size());

    for(unsigned int i=0; i<centers.size(); i++)
    {
        int nbTrianglesInSphere = m_trianglesInSpheresId[i].size();
        int nbQuadsInSphere = m_quadsInSpheresId[i].size();

        Deriv normal(0,0,0);
        for(int t=0; t<nbTrianglesInSphere; t++)
        {
            unsigned int triangleId = m_trianglesInSpheresId[i][t];
            Triangle triangle = triangles[triangleId];

            Coord p0 = positions[triangle[0]];
            Coord p1 = positions[triangle[1]];
            Coord p2 = positions[triangle[2]];

            Deriv triangleNormal = cross(p1-p0, p2-p0);

            normal += triangleNormal;
        }
        for(int q=0; q<nbQuadsInSphere; q++)
        {
            unsigned int quadId = m_quadsInSpheresId[i][q];
            Quad quad = quads[quadId];

            Coord p0 = positions[quad[0]];
            Coord p1 = positions[quad[1]];
            Coord p2 = positions[quad[2]];

            Deriv quadNormal = cross(p1-p0, p2-p0);

            normal += quadNormal;
        }
        normal.normalize();
        directions[i] = normal;
    }
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                            DataMatrixDeriv &cMatrix,
                                                            unsigned int &cIndex,
                                                            const DataVecCoord &x)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    if(d_triangles.getValue().size() == 0 && d_quads.getValue().size() == 0)
    {
        msg_info(this) <<"No triangles and quads given. Get context topology.";
        BaseMeshTopology* topology = this->getContext()->getMeshTopology();

        if(topology==nullptr)
            msg_error(this) << "There is no topology state associated with this node. "
                               "To remove this error message, fix your scene possibly by "
                               "adding a Topology in the parent node or by giving a list of triangles"
                               "indices or a list of quads indices as nodes parameters ." ;

        d_triangles.setValue(topology->getTriangles());
        d_quads.setValue(topology->getQuads());
        m_edges = topology->getEdges();
    }
    else
    {
        computeEdges();
    }


    /// Check that the position datafield contain something, otherwise get context
    /// topology
    WriteAccessor<sofa::Data<VecCoord> > positions = d_positions;
    if(d_positions.getValue().size() == 0)
    {
        msg_info(this) <<"No positions given. Get context mechanical rest position.";
        ReadAccessor<sofa::Data<VecCoord> > restPositions = m_state->readRestPositions();
        positions.resize(restPositions.size());
        for(unsigned int i=0; i<restPositions.size(); i++)
            positions[i] = restPositions[i];
    }


    /// Check that the triangles datafield does not contains indices that would crash the
    /// component.
    int numTris = d_triangles.getValue().size() ;
    auto triangles = d_triangles.getValue() ;
    for(int i=0;i<numTris;i++){
        for(int j=0;j<3;j++){
            if( triangles[i][j] >= positions.size() )
                msg_error(this) << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                                   <<". is too large regarding mechanicalState size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the quads datafield does not contains indices that would crash the
    /// component.
    int numQuads = d_quads.getValue().size() ;
    auto quads = d_quads.getValue() ;
    for(int i=0;i<numQuads;i++){
        for(int j=0;j<4;j++){
            if( quads[i][j] >= positions.size() )
                msg_error(this) << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                                   << " is too large regarding mechanicalState size of("
                                   << positions.size() << ")" ;
        }
    }

    int nbCenters = d_centers.getValue().size();
    int nbRadii = d_radii.getValue().size();

    m_initialCenters = d_centers.getValue();

    WriteAccessor<sofa::Data<vector<Real>>> radii = d_radii;

    if(nbRadii == 0)
    {
        radii.resize(1);
        radii[0] = 1;
        nbRadii = 1;
        msg_warning(this) << "No radius given. Set default radius = 1.";
    }

    if(nbCenters != nbRadii)
    {
        double radius = radii[0];
        radii.resize(nbCenters);
        for(int i=0; i<nbCenters; i++)
            radii[i] = radius;
        msg_warning(this) << "Size of centers and radii list not the same. Will apply first raidus to all the spheres.";
    }

    computeSurfaces();

    d_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(d_constraintIndex);

    if(d_updateNormals.getValue() && m_useNormals)
        computeNormals();

    VecDeriv directions = d_directions.getValue();
    ReadAccessor<sofa::Data<VecCoord>> centers = d_centers;

    MatrixDeriv& matrix = *cMatrix.beginEdit();
    for(unsigned int i=0; i<centers.size(); i++)
    {
        MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex+i);
        for(unsigned int j=0; j<m_pointsInSphereId[i].size(); j++)
            rowIterator.addCol(m_pointsInSphereId[i][j], directions[i]*m_ratios[i][j]);

        cIndex++;
    }
    cMatrix.endEdit();
    
    m_nbLines = cIndex - constraintIndex;
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                             BaseVector *resV,
                                                             const BaseVector *Jdx)
{
    SOFA_UNUSED(cParams);
    SOFA_UNUSED(Jdx);

    const auto& constraintIndex = sofa::helper::getReadAccessor(d_constraintIndex);
    for(unsigned int i=0; i<d_centers.getValue().size(); i++)
        resV->set(constraintIndex+i, 0);
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::storeResults(vector<double> &lambda, vector<double> &delta)
{
    WriteAccessor<sofa::Data<vector<Real>>> force = d_force;
    WriteAccessor<sofa::Data<vector<Real>>> displacement = d_displacement;

    unsigned int nbForces = d_centers.getValue().size();
    force.resize(nbForces);
    displacement.resize(nbForces);
    for(unsigned int i=0; i<nbForces; i++)
    {
        force[i] = lambda[i];
        displacement[i] = delta[i];
    }

    updateCenter();
    updateLimit();

    Actuator<DataTypes>::storeResults(lambda, delta);
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::updateCenter()
{
    WriteAccessor<sofa::Data<vector<Coord>>> centers = d_centers;
    ReadAccessor<sofa::Data<vector<Real>>> displacement = d_displacement;
    ReadAccessor<sofa::Data<vector<Deriv>>> directions = d_directions;

    int nbCenters = centers.size();

    for(int i=0; i<nbCenters; i++)
        centers[i] = m_initialCenters[i] + directions[i]*displacement[i];
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::draw(const VisualParams* vparams)
{
    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    if(d_drawForce.getValue())
        drawForces(vparams);

    if(d_drawSphere.getValue())
        drawSpheres(vparams);

    if(d_drawSurface.getValue())
        drawSurfaces(vparams);
}



template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::drawForces(const VisualParams* vparams)
{
    if(m_state == nullptr || d_force.getValue().size()==0)
        return;

    VecDeriv directions = d_directions.getValue();

    ReadAccessor<sofa::Data<VecCoord> > positions = m_state->readPositions();
    ReadAccessor<sofa::Data<vector<Real>>> force = d_force;

    RGBAColor color(0,1,0,1);
    for(unsigned int i=0; i<directions.size(); i++)
        for(unsigned int j=0; j<m_pointsInSphereId[i].size(); j++)
            if(m_pointsInSphereId[i][j]<m_state->getSize())
                vparams->drawTool()->drawArrow(positions[m_pointsInSphereId[i][j]] - directions[i]*force[i]*m_ratios[i][j], positions[m_pointsInSphereId[i][j]], d_visuScale.getValue(), color, 4);
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::drawSpheres(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord>> centers = d_centers;
    ReadAccessor<sofa::Data<vector<Real>>> radii = d_radii;

    for(unsigned int i=0; i<centers.size(); i++)
        vparams->drawTool()->drawSphere(centers[i], float(radii[i]));
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::drawSurfaces(const VisualParams* vparams)
{

    if(m_state == nullptr)
        return;

    drawTriangles(vparams);
    drawLines(vparams);
}

template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::drawTriangles(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();
    ReadAccessor<sofa::Data<vector<Triangle>>> triangles  = d_triangles;
    vector<Vec3> points;

    for (unsigned int i=0; i<m_trianglesInSpheresId.size(); i++)
        for (unsigned int t=0; t<m_trianglesInSpheresId[i].size(); t++)
        {
            Triangle tri = triangles[m_trianglesInSpheresId[i][t]];
            points.push_back(x[tri[0]]);
            points.push_back(x[tri[1]]);
            points.push_back(x[tri[2]]);
        }

    vparams->drawTool()->drawTriangles(points, sofa::type::RGBAColor(0.6f,0.0f,0.0f,1.0f));
}

template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::drawLines(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord> > x = m_state->readPositions();
    vector<Vec3> points;

    for (unsigned int i=0; i<m_edgesInSpheresId.size(); i++)
        for (unsigned int e=0; e<m_edgesInSpheresId[i].size(); e++)
        {
            Edge edge = m_edges[m_edgesInSpheresId[i][e]];

            points.push_back(x[edge[0]]);
            points.push_back(x[edge[1]]);
        }

    vparams->drawTool()->drawLines(points, 1.0f, sofa::type::RGBAColor(0.2f,0.0f,0.0f,1.0f));
}


template<class DataTypes>
void ForceSurfaceActuator<DataTypes>::computeEdges()
{
    ReadAccessor<sofa::Data<vector<Triangle>>> triList  = d_triangles;
    ReadAccessor<sofa::Data<vector<Quad>>>   quadList = d_quads;

    std::map<Edge,unsigned int> edgeMap;
    unsigned int edgeIndex;
    m_edges.clear();

    for (Triangle t : triList)
    {
        std::map<Edge,unsigned int>::iterator ite;
        Edge e;
        for (unsigned int j=0; j<3; ++j)
        {
            unsigned int v1=t[(j+1)%3];
            unsigned int v2=t[(j+2)%3];
            // sort vertices in lexicographics order
            if (v1<v2)
                e=Edge(v1,v2);
            else
                e=Edge(v2,v1);
            ite=edgeMap.find(e);
            if (ite==edgeMap.end())
            {
                // edge not in edgeMap so create a new one
                edgeIndex=m_edges.size();
                edgeMap[e]=edgeIndex;
                m_edges.push_back(e);
            }
        }
    }

    for (Quad q : quadList)
    {
        std::map<Edge,unsigned int>::iterator ite;
        Edge e;
        for (unsigned int j=0; j<4; ++j)
        {
            unsigned int v1=q[(j+1)%4];
            unsigned int v2=q[(j+2)%4];
            // sort vertices in lexicographics order
            if (v1<v2)
                e=Edge(v1,v2);
            else
                e=Edge(v2,v1);
            ite=edgeMap.find(e);
            if (ite==edgeMap.end())
            {
                // edge not in edgeMap so create a new one
                edgeIndex=m_edges.size();
                edgeMap[e]=edgeIndex;
                m_edges.push_back(e);
            }
        }
    }
}

} // namespace

