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
*                           Plugin SoftRobots                                 *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
******************************************************************************/
#pragma once

#include <sofa/core/visual/VisualParams.h>
#include <sofa/geometry/proximity/PointTriangle.h>
#include <sofa/type/Vec.h>
#include <SoftRobots/component/constraint/model/CableModel.h>

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;

using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;

using sofa::linearalgebra::BaseVector;

using sofa::type::RGBAColor;
using sofa::type::Vec3;
using sofa::type::vector;

using std::pair;
using std::set;

template<class DataTypes>
CableModel<DataTypes>::CableModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    , d_indices(initData(&d_indices, "indices",
                         "List of points connected by the cable (from extremity to actuated point). \n"
                         "If no indices are given, default value is 0. \n"
                         "In case of multiple indices, one point will be actuated \n"
                         "and the others will represent sliding points for the cable."))

    , d_pullPoint(initData(&d_pullPoint, Coord(), "pullPoint",
                                          "Fixed point from which the cable is pulled. \n"
                                          "If unspecified, the default value is {0.0,0.0,0.0}"))

    , d_hasPullPoint(initData(&d_hasPullPoint, true ,"hasPullPoint",
                              "If false, the pull point is not considered and the cable is entirely mapped \n"
                              " In that case, needs at least 2 different point in indices."))

    , d_cableInitialLength(initData(&d_cableInitialLength, Real(0.0), "cableInitialLength","This value can be defined by the user. \n"
                                    "If not defined, it will correspond to the length of the cable at the start of the simulation"))

    , d_cableLength(initData(&d_cableLength, Real(0.0), "cableLength","Computation done at the end of the time step"))

    , d_method(initData(&d_method, {"point","sphere","geodesic"}, "method", "Default is point method. \n"
                                    "In point method, cable force is applied on a single point. \n"
                                    "Both methods sphere and geodesic are compatible with passing point on a surface only. \n "
                                    "In sphere method, cable force is dispatched in the intersection between a 3D sphere and a surface. \n"
                                    "In geodesic method, cable force is dispatched in a circle projected on a surface. \n"))

    , d_centers(initData(&d_centers, "centers",
                       "List of positions describing attachment of cables on the surface, used only with sphere and geodesic methods. \n"
                       "Points are centers of cable pulling application areas. \n"
                       "If not defined, centers are computed from provided indices instead."))

    , d_radii(initData(&d_radii, "radii", "List of radius used to compute pulling application areas from centers. \n"
                                            "Used only with sphere and geodesic methods."))

    , l_surfaceTopology(initLink("surfaceTopology", "Link to the topology container of the surface on which the cable is attached. \n"
                                                    "Used only with sphere and geodesic methods."))

    , d_force(initData(&d_force,double(0.0), "force",
                                         "Output force. Warning: to get the actual force you should divide this value by dt."))

    , d_displacement(initData(&d_displacement,double(0.0), "displacement",
                          "Output displacement compared to the initial cable length."))

    , d_maxForce(initData(&d_maxForce, "maxForce",
                          "Maximum force of the actuator. \n"
                          "If unspecified no maximum value will be considered."))

    , d_minForce(initData(&d_minForce, "minForce",
                          "Minimum force of the actuator. \n"
                          "If unspecified no minimum value will be considered \n"
                          "and the cable will then be seen as a stiff rod able to push."))

    , d_eqForce(initData(&d_eqForce,Real(0.0), "eqForce",
                          "Equality force of the actuator. \n"
                          "Solver will try to maintain the cable force at this value\n"
                          "If unspecified, no value will be considered \n"
                          ))

    , d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement,"maxPositiveDisp",
                                         "Maximum displacement of the actuator in the positive direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement,"maxNegativeDisp",
                                         "Maximum displacement of the actuator in the negative direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_eqDisplacement(initData(&d_eqDisplacement,Real(0.0), "eqDisp",
                                "Equality displacement of the actuator. \n"
                                "Solver will try to maintain the cable displacement at this value\n"
                                "If unspecified, no value will be considered \n" ))

    , d_maxDispVariation(initData(&d_maxDispVariation, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_drawPullPoint(initData(&d_drawPullPoint,true, "drawPullPoint",
                          ""))

    , d_drawPoints(initData(&d_drawPoints,true, "drawPoints",
                          ""))

    , d_drawPulledAreas(initData(&d_drawPulledAreas, false, "drawPulledAreas","Whether to draw pulled area points or not."))

    , d_color(initData(&d_color,RGBAColor(0.4f,0.4f,0.4f,1.f), "color",
                          "Color of the string."))

{
    setUpData();
}

template<class DataTypes>
CableModel<DataTypes>::~CableModel()
{
}

template<class DataTypes>
void CableModel<DataTypes>::setUpData()
{
    d_cableLength.setReadOnly(true);

    d_force.setGroup("Vector");
    d_displacement.setGroup("Vector");
    d_force.setReadOnly(true);
    d_displacement.setReadOnly(true);

    d_drawPullPoint.setGroup("Visualization");
    d_drawPoints.setGroup("Visualization");
    d_drawPulledAreas.setGroup("Visualization");
    d_color.setGroup("Visualization");
}

template<class DataTypes>
void CableModel<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    SoftRobotsConstraint<DataTypes>::init();

    if(m_state==nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. "
                           "The object is then deactivated. "
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    const unsigned int id_method = d_method.getValue().getSelectedId();
    if (id_method == 1 || id_method == 2)
    {
        if (l_surfaceTopology.empty())
        {
            msg_info() << "Link to topology container of the surface should be set to ensure right behavior. First topology found in current context will be used.";
            l_surfaceTopology.set(this->getContext()->getMeshTopologyLink());
        }

        if (l_surfaceTopology.get() == nullptr)
        {
            msg_error(this) << "There is no topology state associated with this node. "
                                "To remove this error message, fix your scene possibly by "
                                "adding a topology in this node";
            d_componentState.setValue(ComponentState::Invalid);
        }
            
    }
    internalInit();
    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void CableModel<DataTypes>::bwdInit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
        return;
    }
            

    // The initial length of the cable is set or computed in bwdInit so the mapping (if there is any)
    // will be considered

    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    ReadAccessor<Data<VecCoord>> restPositions = m_state->readRestPositions();

    Real cableLength = getCableLength(positions.ref());

    if (!d_cableInitialLength.isSet())
    {
        Real initialCableLength = getCableLength(restPositions.ref());
        d_cableInitialLength.setValue(initialCableLength);
    }
    d_cableLength.setValue(cableLength);
        
}


template<class DataTypes>
void CableModel<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
            return ;
    }
    internalInit();
}


template<class DataTypes>
void CableModel<DataTypes>::reset()
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
        return ;
    }
    d_cableLength.setValue(d_cableInitialLength.getValue());
}



template<class DataTypes>
void CableModel<DataTypes>::internalInit()
{
    initActuatedPoints();
    checkIndicesRegardingState();

    const unsigned int id_method = d_method.getValue().getSelectedId();
    if (id_method == 1 || id_method == 2)
    {
        initCableActionAreas();
    }
}


template<class DataTypes>
void CableModel<DataTypes>::initActuatedPoints()
{   
    auto indices = sofa::helper::getWriteOnlyAccessor(d_indices);
    std::size_t nbIndices = indices.size();
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    ReadAccessor<Data<VecCoord>> centers = d_centers;
    const std::size_t nbCenters = centers.size();
    m_hasCenters = false;
    if (nbCenters != 0)
    {
        m_hasCenters = true;
        msg_warning_when(nbIndices != 0) <<"Both centers and indices are provided. Centers are used by default";
        indices.clear();
        indices.reserve(nbCenters);
        std::transform(centers.begin(), centers.end(), std::back_inserter(indices),
            [this](const Coord& center){ return computeClosestIndice(center);});
        nbIndices = indices.size();
    }

    if ( !d_hasPullPoint.getValue())
    {
        if (nbIndices<=1)
        {
            msg_error() <<"If no pull point, at least two indices of actuation should be given";
        }
        else
        {
            nbIndices-=1;  // the first point of the list considered as the pull point
            d_pullPoint.setValue(positions[indices[0]]);
        }
    }

    if (nbIndices == 0)
    {
        msg_warning() <<"No index of actuation given, set default 0";
        indices.push_back(0);

        m_hasSlidingPoint=false;
    }
    else if (nbIndices == 1)
    {
        m_hasSlidingPoint=false;
    }
    else
    {   
        m_hasSlidingPoint=true;
    }    
}


template<class DataTypes>
void CableModel<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

    const auto& indices = d_indices.getValue();

    for(unsigned int i=0; i<indices.size(); i++)
    {
        if (positions.size() <= indices[i])
        {
            msg_error() << "Indices at index " << i << " is too large regarding mechanicalState [position] size" ;
        }
    }
}


template<class DataTypes>
void CableModel<DataTypes>::initCableActionAreas()
{    
    std::size_t nbCenters = d_indices.getValue().size();
    WriteAccessor<Data<vector<Real>>> radii = d_radii;
    std::size_t nbRadii = radii.size();
    if(nbRadii == 0)
    {
        radii.resize(nbCenters);
        for(std::size_t i=0; i<nbCenters; i++)
            radii[i] = 1;
        nbRadii = nbCenters;
        msg_warning() << "No radius given. Set default radius = 1.";
    }

    if(nbCenters != nbRadii)
    {
        const Real radius = radii[0];
        radii.resize(nbCenters);
        for(std::size_t i=0; i<nbCenters; i++)
            radii[i] = radius;
        msg_warning(this) << "Size of centers and radii list not the same. Will apply first radius to all the effect areas.";
    }
    
    computePointsActionArea();
}


template<class DataTypes>
void CableModel<DataTypes>::computePointsActionArea()
{
    // Implementing continuous Dijkstra as a geodesic distance measure
    const unsigned int id_method = d_method.getValue().getSelectedId();
    std::size_t nbCenters = d_indices.getValue().size();      
    const SetIndexArray &indices = d_indices.getValue();

    ReadAccessor<Data<VecCoord>> cablePositions = m_state->readPositions();
    ReadAccessor<Data<VecCoord>> centers = d_centers;    
    if (m_hasCenters)
    {
        m_alphaBarycentric.clear();
        m_alphaBarycentric.resize(nbCenters);
        m_betaBarycentric.clear();
        m_betaBarycentric.resize(nbCenters);
        m_closestTriangle.clear();
        m_closestTriangle.resize(nbCenters);
    } 
    ReadAccessor<Data<vector<Real>>> maxDistance = d_radii;
    vector<Real> m_totalRatio;
    m_areaIndices.clear();
    m_areaIndices.resize(nbCenters);
    m_ratios.clear();
    m_ratios.resize(nbCenters);
    m_totalRatio.clear();
    m_totalRatio.resize(nbCenters);
    

    // Iterate for each cable attachment
    typedef pair<Real,BaseMeshTopology::PointID> DistanceToPoint;
    for (unsigned int i=0; i<nbCenters; i++)
    {   
        // Init
        set<DistanceToPoint> queue; 
        typename set<DistanceToPoint>::iterator qit;
        vector<Real> distances(l_surfaceTopology.get()->getNbPoints(),maxDistance[i]);

        // If cable attachment does not match with a mesh vertice
        // Project on closest triangle and distribute distance to its vertices 
        if (m_hasCenters)
        {
            const TrianglesAroundVertex& trianglesAroundClosestVertex =l_surfaceTopology->getTrianglesAroundVertex(indices[i]); // Triangles connected to closest vertice
            Coord projectionOnTriangle;
            Real minDistanceToTriangle = std::numeric_limits<Real>::max();
            unsigned int closestTriangleId = 0;
            Coord closestProjectionOnTriangle;
            for(unsigned int j=0; j<trianglesAroundClosestVertex.size(); j++)
            {
                const Triangle triangle = l_surfaceTopology->getTriangle(trianglesAroundClosestVertex[j]);
                const Real distanceToTriangle = getDistanceToTriangle(centers[i], triangle, projectionOnTriangle);

                if(distanceToTriangle < minDistanceToTriangle)
                {
                    minDistanceToTriangle = distanceToTriangle;
                    closestTriangleId = j;
                    closestProjectionOnTriangle = projectionOnTriangle;
                }
            }

            const Triangle closestTriangle =l_surfaceTopology->getTriangle(trianglesAroundClosestVertex[closestTriangleId]);

            for (const auto& t : closestTriangle)
            {
                Coord p;
                getPositionFromTopology(p, t);

                const Real d = (closestProjectionOnTriangle - p).norm();
                if (id_method == 2)
                {
                    distances[t] = minDistanceToTriangle + d;
                } 
                else if (id_method == 1)
                {
                    distances[t] = (p - cablePositions[indices[i]]).norm();
                }
                queue.insert(DistanceToPoint(d,indices[i]));
            }
            
            // Compute barycentric coordinates for visualization purposes
            computeBarycentric(closestTriangle, closestProjectionOnTriangle, m_alphaBarycentric[i], m_betaBarycentric[i]);
            m_closestTriangle[i] = closestTriangle;


        }
        else
        {
            distances[indices[i]] = 0.0;
            queue.insert(DistanceToPoint(0.0,indices[i]));
        }

        // Dijkstra
        while(!queue.empty() )
        {
            DistanceToPoint top = *queue.begin();
            queue.erase(queue.begin());
            BaseMeshTopology::PointID v = top.second; 
            const vector<BaseMeshTopology::PointID> neighboors =l_surfaceTopology->getVerticesAroundVertex(v);
            for (const auto& vn : neighboors)
            {
                Coord pv; 
                getPositionFromTopology(pv, v);
                Coord pvn;
                getPositionFromTopology(pvn, vn);
                Real d = 0.0;
                if (id_method == 2)
                {
                    d = distances[v] + (pv - pvn).norm();
                }
                else if (id_method == 1)
                {
                    d = (pvn - cablePositions[indices[i]]).norm();
                }
                if(distances[vn] > d)
                {
                    qit=queue.find(DistanceToPoint(distances[vn],vn));
                    if(qit != queue.end()) {queue.erase(qit);}
                    distances[vn] = d;
                    queue.insert(DistanceToPoint(d,vn) );
                }
            }
        }

        // Distribute forces
        m_totalRatio[i] = 0.0;
        for(unsigned int p=0; p<distances.size(); p++)
        {
            if(distances[p] < maxDistance[i])
            {
                const auto ratio = rabs(distances[p] - maxDistance[i]);

                m_totalRatio[i] += ratio;
                m_ratios[i].push_back(ratio);
                m_areaIndices[i].push_back(p);
            } 
        }

        // Normalize force ratios
        for(unsigned int j=0; j<m_ratios[i].size(); j++)
        {
            if (m_totalRatio[i] == 0.0)
            {
                m_ratios[i][j] = 0.0;
            }
            else 
            {
                m_ratios[i][j] = m_ratios[i][j] / m_totalRatio[i];
            }
        }
            
    }
}


template<class DataTypes>
unsigned int CableModel<DataTypes>::computeClosestIndice(const Coord& position)
{
    Coord p0;
    getPositionFromTopology(p0, 0);
    auto closest_distance = (position - p0).norm();
    unsigned int closest_vertice = 0;
    for(unsigned int j=1; j<l_surfaceTopology->getNbPoints(); j++)
    {
        Coord pj;
        getPositionFromTopology(pj, j);
        const auto distance = (position - pj).norm();
        if(distance < closest_distance) 
        {
            closest_distance = distance;
            closest_vertice = j;
        }
    } 
   return closest_vertice;
}

template<class DataTypes>
void CableModel<DataTypes>::getPositionFromTopology(Coord& position, sofa::Index index)
{
    position.x() = l_surfaceTopology.get()->getPX(index);
    if constexpr (Coord::spatial_dimensions > 1)
    {
        position.y() = l_surfaceTopology.get()->getPY(index);
        if constexpr (Coord::spatial_dimensions > 2)
        {
            position.z() = l_surfaceTopology.get()->getPZ(index);
        }
    }
}

template<class DataTypes>
SReal CableModel<DataTypes>::getDistanceToTriangle(const Coord& position, const Triangle& triangle, Coord& projectionOnTriangle)
{
    Coord p0 { sofa::type::NOINIT };
    Coord p1 { sofa::type::NOINIT };
    Coord p2 { sofa::type::NOINIT };
    getPositionFromTopology(p0, triangle[0]);
    getPositionFromTopology(p1, triangle[1]);
    getPositionFromTopology(p2, triangle[2]);
    Vec3 projection { sofa::type::NOINIT };
    const bool r = sofa::geometry::proximity::computeClosestPointOnTriangleToPoint(Vec3(p0), Vec3(p1), Vec3(p2), Vec3(position), projection);
    assert(r);
    SOFA_UNUSED(r);
    projectionOnTriangle = projection;
    const Real distanceToTriangle = (projectionOnTriangle - position).norm();
    return distanceToTriangle;
}


template<class DataTypes>
void CableModel<DataTypes>::computeBarycentric(const Triangle& triangle, const Coord& p, Real& alpha, Real& beta)
{
    Coord v0 { sofa::type::NOINIT };
    Coord v1 { sofa::type::NOINIT };
    Coord v2 { sofa::type::NOINIT };
    getPositionFromTopology(v0, triangle[0]);
    getPositionFromTopology(v1, triangle[1]);
    getPositionFromTopology(v2, triangle[2]); 

   // Solving with Cramer's rule
    Coord e0 = v1 - v0; 
    Coord e1 = v2 - v0;
    Coord e2 = p - v0;
    Real d00 = dot(e0, e0);
    Real d01 = dot(e0, e1);
    Real d11 = dot(e1, e1);
    Real d20 = dot(e2, e0);
    Real d21 = dot(e2, e1);
    Real denom = d00 * d11 - d01 * d01;
    if (denom == 0.0)
    {
        alpha = 0.0;
        beta = 0.0;
    } 
    else
    {
        alpha = (d11 * d20 - d01 * d21) / denom;
        beta = (d00 * d21 - d01 * d20) / denom;
    } 
}


template<class DataTypes>
SReal CableModel<DataTypes>::getCableLength(const VecCoord &positions)
{
    const SetIndexArray &indices = d_indices.getValue();
    Coord previousPosition = d_pullPoint.getValue();

    // if no fixed pull point, we take the first point of the list
    if (!d_hasPullPoint.getValue())
    {
        previousPosition = positions[ indices[0] ];
    }
    Real cableLength = 0.0;
    for (unsigned int i=0; i<indices.size(); i++)
    {
        Coord currentPosition  = positions[indices[i]];
        Deriv direction = currentPosition - previousPosition;
        previousPosition = currentPosition;
        cableLength += direction.norm();
    }

    return cableLength;
}


template<class DataTypes>
void CableModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
            return ;
    }

    SOFA_UNUSED(cParams);

    m_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(constraintIndex);

    VecCoord positions = x.getValue();

    const unsigned int id_method = d_method.getValue().getSelectedId();

    if(!m_hasSlidingPoint)
    {

        if ( d_hasPullPoint.getValue())
        {
            Deriv direction = DataTypes::coordDifference(d_pullPoint.getValue(),positions[d_indices.getValue()[0]]);
            direction.normalize();
            if (id_method == 0)
            {
                rowIterator.setCol(d_indices.getValue()[0],  direction);
            }
            else 
            {
                for(unsigned int j=0; j<m_areaIndices[0].size(); j++)
                    rowIterator.setCol(m_areaIndices[0][j], direction*m_ratios[0][j]);
            }
        }
        else
        {
            Deriv direction = DataTypes::coordDifference(positions[d_indices.getValue()[1]],positions[d_indices.getValue()[0]]);
            direction.normalize();
            if (id_method == 0)
            {
                rowIterator.setCol(d_indices.getValue()[1],  -direction);
                rowIterator.setCol(d_indices.getValue()[0],  direction);
            } 
            else 
            {
                for(unsigned int j=0; j<m_areaIndices[0].size(); j++)
                    rowIterator.setCol(m_areaIndices[0][j], direction*m_ratios[0][j]);

                for(unsigned int j=0; j<m_areaIndices[0].size(); j++)
                    rowIterator.setCol(m_areaIndices[1][j], - direction*m_ratios[0][j]);
            }
            
        }
    }
    else
    {
        const SetIndexArray &indices = d_indices.getValue();

        for (unsigned int i=0; i<indices.size(); i++)
        {
            Coord previousPosition;
            Coord currentPosition;
            Coord nextPosition;

            if (i == 0)  // start point of the cable //
            {
                int currentIndex = indices[i];
                int nextIndex    = indices[i+1];

                previousPosition = (d_hasPullPoint.getValue()) ? d_pullPoint.getValue(): positions[d_indices.getValue()[0]];
                currentPosition  = positions[currentIndex];
                nextPosition     = positions[nextIndex];

                Deriv directionBeyond = previousPosition - currentPosition;
                directionBeyond.normalize();

                Deriv directionAhead  = currentPosition - nextPosition;
                directionAhead.normalize();

                Deriv slidingDirection = directionBeyond - directionAhead;
                if (d_hasPullPoint.getValue())
                {
                    if (id_method == 0)
                    {
                        rowIterator.setCol(currentIndex, slidingDirection);
                    }
                    else 
                    {
                        for(unsigned int j=0; j<m_areaIndices[i].size(); j++)
                            rowIterator.setCol(m_areaIndices[i][j], slidingDirection*m_ratios[i][j]);
                    }
                }                 
                else
                {
                    if (id_method == 0)
                    {
                        rowIterator.setCol(currentIndex, -directionAhead);
                    }
                    else 
                    {
                        for(unsigned int j=0; j<m_areaIndices[i].size(); j++)
                            rowIterator.setCol(m_areaIndices[i][j], -directionAhead*m_ratios[i][j]);
                    }
                }             
            }
            else if (i != indices.size()-1)  // all points except extremities
            {
                int previousIndex = indices[i-1];
                int currentIndex  = indices[i];
                int nextIndex     = indices[i+1];

                previousPosition = positions[previousIndex];
                currentPosition  = positions[currentIndex];
                nextPosition     = positions[nextIndex];

                Deriv directionBeyond = previousPosition - currentPosition;
                directionBeyond.normalize();

                Deriv directionAhead  = currentPosition - nextPosition;
                directionAhead.normalize();

                Deriv slidingDirection = directionBeyond - directionAhead;

                if (id_method == 0)
                    rowIterator.setCol(currentIndex, slidingDirection);
                else 
                {
                    for(unsigned int j=0; j<m_areaIndices[i].size(); j++)
                        rowIterator.setCol(m_areaIndices[i][j], slidingDirection*m_ratios[i][j]);
                }

                
            }
            else // end point of the cable
            {
                int previousIndex = indices[i-1];
                int currentIndex  = indices[i];

                previousPosition = positions[previousIndex];
                currentPosition  = positions[currentIndex];

                Deriv directionOfActuation = previousPosition - currentPosition;
                directionOfActuation.normalize();

                if (id_method == 0)
                    rowIterator.setCol(currentIndex, directionOfActuation);
                else 
                {
                    for(unsigned int j=0; j<m_areaIndices[i].size(); j++)
                        rowIterator.setCol(m_areaIndices[i][j], directionOfActuation*m_ratios[i][j]);
                }
            }
        }
    }
    cIndex++;
    cMatrix.endEdit();
    m_nbLines = cIndex - constraintIndex;
}


template<class DataTypes>
void CableModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                   BaseVector *resV,
                                                   const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
            return ;
    }

    SOFA_UNUSED(cParams);

    d_cableLength.setValue(getCableLength(m_state->readPositions().ref()));
    Real dfree = Jdx->element(0) + d_cableInitialLength.getValue() - d_cableLength.getValue();
    resV->set(m_constraintIndex.getValue(), dfree);
}


template<class DataTypes>
void CableModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                        sofa::core::MultiVecDerivId res,
                                        const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
    {
            return ;
    }

    d_force.setValue(lambda->element(m_constraintIndex.getValue()));

    // Compute actual cable length and displacement from updated positions of mechanical
    // Eulalie.C: For now the position of the mechanical state is not up to date when storeLambda() is called
    //            so the value of delta is one step behind...
    //ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    //d_cableLength.setValue(getCableLength(positions.ref()));
    d_displacement.setValue(d_cableInitialLength.getValue()-d_cableLength.getValue());
}

template<class DataTypes>
void CableModel<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
    {
            return ;
    }

    if (!vparams->displayFlags().getShowInteractionForceFields()) 
    {
        return;
    }

    drawLinesBetweenPoints(vparams);
    
    if(d_drawPoints.getValue())
    {
        drawPoints(vparams);
    }

    if(d_drawPullPoint.getValue())
    {
        drawPullPoint(vparams);
    }

    const unsigned int id_method = d_method.getValue().getSelectedId();
    if (id_method == 1 || id_method == 2)
        {
        if(d_drawPulledAreas.getValue())
        {
            drawPulledAreas(vparams);
        }
    }
}


template<class DataTypes>
void CableModel<DataTypes>::drawPullPoint(const VisualParams* vparams)
{
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vec3> points(1);
    points[0] = d_pullPoint.getValue();
    if(!d_hasPullPoint.getValue() && indices.size()>=1)
    {
        ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
        points[0] = positions[indices[0]];
    }

    vparams->drawTool()->drawPoints(points, 15, RGBAColor::yellow());
}


template<class DataTypes>
void CableModel<DataTypes>::drawPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vec3> points(indices.size());
    for (unsigned int i=0; i<indices.size(); i++)
    {
        if (!m_hasCenters)
        {
            points[i] = positions[indices[i]];
        }
        else
        { 
            points[i] = positions[m_closestTriangle[i][0]] 
            + m_alphaBarycentric[i] * (positions[m_closestTriangle[i][1]] - positions[m_closestTriangle[i][0]])
            + m_betaBarycentric[i] * (positions[m_closestTriangle[i][2]] - positions[m_closestTriangle[i][0]]);
        }
    }
        

    vparams->drawTool()->drawPoints(points, 15, RGBAColor::red());
}


template<class DataTypes>
void CableModel<DataTypes>::drawLinesBetweenPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    
    const SetIndexArray &indices = d_indices.getValue();

    RGBAColor color = d_color.getValue();
    vector<Vec3> points(indices.size()*2);
    Coord previousPosition = d_pullPoint.getValue();
    unsigned int first = 0;

    if(!d_hasPullPoint.getValue() && indices.size()>=1)
    {
        if (!m_hasCenters)
        {
            previousPosition = positions[indices[0]];            
        }
        else 
        {
            previousPosition = positions[m_closestTriangle[0][0]] 
            + m_alphaBarycentric[0] * (positions[m_closestTriangle[0][1]] - positions[m_closestTriangle[0][0]])
            + m_betaBarycentric[0] * (positions[m_closestTriangle[0][2]] - positions[m_closestTriangle[0][0]]);
        }
        first = 1;
    }
    
    for (unsigned int i=first; i<indices.size(); i++)
    {
        Coord currentPosition;
        if (!m_hasCenters)
        {
            currentPosition = positions[indices[i]];
        }
        else
        { 
            currentPosition = positions[m_closestTriangle[i][0]] 
            + m_alphaBarycentric[i] * (positions[m_closestTriangle[i][1]] - positions[m_closestTriangle[i][0]])
            + m_betaBarycentric[i] * (positions[m_closestTriangle[i][2]] - positions[m_closestTriangle[i][0]]);
        }
        points[i*2] = currentPosition;
        points[i*2+1] = previousPosition;
        previousPosition = currentPosition;
    }    

    vparams->drawTool()->drawLines(points, 1.5, color);
}


template<class DataTypes>
void CableModel<DataTypes>::drawPulledAreas(const VisualParams* vparams)
{
    ReadAccessor<Data<vector<Coord>>> positions = m_state->readPositions();

    for(unsigned int i=0; i<m_areaIndices.size(); i++)
        for(unsigned int j=0; j<m_areaIndices[i].size(); j++)
        {
            vector<Vec3> point(1);
            point[0] = positions[m_areaIndices[i][j]];
            vparams->drawTool()->drawPoints(point, 40.f * m_ratios[i][j], RGBAColor::yellow());
        }
}



} // namespace

