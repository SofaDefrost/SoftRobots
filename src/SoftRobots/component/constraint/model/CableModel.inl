/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/

#ifndef SOFA_COMPONENT_CONSTRAINTSET_CABLEMODEL_INL
#define SOFA_COMPONENT_CONSTRAINTSET_CABLEMODEL_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/type/Vec.h>
#include "CableModel.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;

using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;

using std::pair;
using std::set;

using sofa::linearalgebra::BaseVector;
using sofa::type::vector;
using sofa::type::Vector3;

using sofa::core::visual::VisualParams;
using sofa::type::RGBAColor;



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

    , d_method(initData(&d_method, sofa::helper::OptionsGroup(3,"point","sphere","geodesic"), "method", "Default is point method."))

    , d_centers(initData(&d_centers, "centers",
                       "List of positions describing attachment of cables"))

    , d_radii(initData(&d_radii, "radii", "List of radii describing the ROI spheres"))

    , l_surfaceTopology(initLink("surfaceTopology", "Link to the topology container of the surface on which the cable is attached."))

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

    , d_color(initData(&d_color,RGBAColor(0.4,0.4,0.4,1), "color",
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
            msg_info() << "link to topology container of the surface should be set to ensure right behavior. First topology found in current context will be used.";
            l_surfaceTopology.set(this->getContext()->getMeshTopologyLink());
        }
        m_topology = l_surfaceTopology.get();

        if (m_topology == nullptr)
            msg_error(this) << "There is no topology state associated with this node. "
                                "To remove this error message, fix your scene possibly by "
                                "adding a topology in the parent node";
    }
    
    internalInit();
    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void CableModel<DataTypes>::bwdInit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    // The initial length of the cable is set or computed in bwdInit so the mapping (if there is any)
    // will be considered

    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    ReadAccessor<Data<VecCoord>> restPositions = m_state->readRestPositions();

    Real cableLength = getCableLength(positions.ref());

    if (!d_cableInitialLength.isSet()){
        Real initialCableLength = getCableLength(restPositions.ref());
        d_cableInitialLength.setValue(initialCableLength);
    }
    d_cableLength.setValue(cableLength);
        
}


template<class DataTypes>
void CableModel<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    internalInit();
}


template<class DataTypes>
void CableModel<DataTypes>::reset()
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    d_cableLength.setValue(d_cableInitialLength.getValue());
}



template<class DataTypes>
void CableModel<DataTypes>::internalInit()
{
    initActuatedPoints();
    checkIndicesRegardingState();

    const unsigned int id_method = d_method.getValue().getSelectedId();
    if (id_method == 1 || id_method == 2)
        initCableActionAreas();
}


template<class DataTypes>
void CableModel<DataTypes>::initActuatedPoints()
{   
    int nbIndices = d_indices.getValue().size();
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    const SetIndexArray &indices = d_indices.getValue();
    
    ReadAccessor<Data<VecCoord>> centers = d_centers;
    unsigned int nbCenters = centers.size();
    if (nbCenters != 0)
    {
        if (nbIndices != 0)
            msg_warning() <<"Both centers and indices are provided. Centers are used by default";

        auto list = sofa::helper::getWriteOnlyAccessor(d_indices);
        list.clear();
        list.resize(nbCenters);
        for (unsigned int i=0; i<nbCenters; i++)
            list[i] = computeClosestIndice(centers[i]);    
        nbIndices = d_indices.getValue().size();
    }

    if ( !d_hasPullPoint.getValue()){
        if (nbIndices<=1)
            msg_error() <<"If no pull point, at least two indices of actuation should be given";
        else
        {
            nbIndices-=1;  // the first point of the list considered as the pull point
            d_pullPoint.setValue(positions[indices[0]]);
        }
    }

    if (nbIndices == 0)
    {
        auto list = sofa::helper::getWriteOnlyAccessor(d_indices);
        msg_warning() <<"No index of actuation given, set default 0";
        list.push_back(0);

        m_hasSlidingPoint=false;
    }
    else if (nbIndices == 1)
        m_hasSlidingPoint=false;
    else
        m_hasSlidingPoint=true;
}


template<class DataTypes>
void CableModel<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

    for(unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error() << "Indices at index " << i << " is too large regarding mechanicalState [position] size" ;
    }
}


template<class DataTypes>
void CableModel<DataTypes>::initCableActionAreas()
{    
    int nbCenters = d_indices.getValue().size();
    WriteAccessor<Data<vector<Real>>> radii = d_radii;
    int nbRadii = radii.size();
    if(nbRadii == 0)
    {
        radii.resize(nbCenters);
        for(int i=0; i<nbCenters; i++)
            radii[i] = 1;
        nbRadii = nbCenters;
        msg_warning(this) << "No radius given. Set default radius = 1.";
    }

    if(nbCenters != nbRadii)
    {
        Real radius = radii[0];
        radii.resize(nbCenters);
        for(int i=0; i<nbCenters; i++)
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
    ReadAccessor<Data<VecCoord>> cablePositions = m_state->readPositions();
    unsigned int nbCenters = d_indices.getValue().size();
    const SetIndexArray &indices = d_indices.getValue();
    ReadAccessor<Data<vector<Real>>> maxDistance = d_radii;
    vector<Real> m_totalRatio;
    m_areaIndices.clear();
    m_areaIndices.resize(nbCenters);
    m_ratios.clear();
    m_ratios.resize(nbCenters);
    m_totalRatio.clear();
    m_totalRatio.resize(nbCenters);

    // Find closest indice on surface topology
    SetIndexArray m_closestCenterIndices;
    if (nbCenters != 0)
    {
        m_closestCenterIndices.resize(nbCenters);
        for(unsigned int i=0; i<nbCenters; i++)
            m_closestCenterIndices[i] = computeClosestIndice(cablePositions[indices[i]]);
    }

    // Iterate for each cable attachment
    typedef pair<Real,BaseMeshTopology::PointID> DistanceToPoint;
    for (unsigned int i=0; i<nbCenters; i++)
    {   
        // Init
        set<DistanceToPoint> queue; 
        typename set<DistanceToPoint>::iterator qit;
        vector<Real> distances(m_topology->getNbPoints(),maxDistance[i]);

        //////////////////// If cable attachment does not match with a mesh vertice /////////////////
        //////////// Project on closest triangle and distribute distance to its vertices ////////////
        const TrianglesAroundVertex& trianglesAroundClosestVertex = m_topology->getTrianglesAroundVertex(m_closestCenterIndices[i]); // Triangles connected to closest vertice
        static sofa::helper::DistancePointTri proximitySolver;
        Vector3 projectionOnTriangle;
        Real minDistanceToTriangle = std::numeric_limits<Real>::max(); unsigned int closestTriangleId = 0; Vector3 closestProjectionOnTriangle;
        for(unsigned int j=0; j<trianglesAroundClosestVertex.size(); j++)
        {
            const Triangle triangle =  m_topology->getTriangle(trianglesAroundClosestVertex[j]);   
            const Coord p0 = Coord(m_topology->getPX(triangle[0]), m_topology->getPY(triangle[0]), m_topology->getPZ(triangle[0]));
            const Coord p1 = Coord(m_topology->getPX(triangle[1]), m_topology->getPY(triangle[1]), m_topology->getPZ(triangle[1]));
            const Coord p2 = Coord(m_topology->getPX(triangle[2]), m_topology->getPY(triangle[2]), m_topology->getPZ(triangle[2]));           
            proximitySolver.NewComputation(p0, p1, p2, cablePositions[indices[i]],projectionOnTriangle);
            Real distanceToTriangle = (projectionOnTriangle - cablePositions[indices[i]]).norm();  
            if(distanceToTriangle < minDistanceToTriangle)
            {
                minDistanceToTriangle = distanceToTriangle;
                closestTriangleId = j;
                closestProjectionOnTriangle = projectionOnTriangle;
            }
        }

        const Triangle closestTriangle =  m_topology->getTriangle(trianglesAroundClosestVertex[closestTriangleId]);

        
        for(unsigned int j=0; j<closestTriangle.size(); j++)
        {
        Coord p = Coord(m_topology->getPX(closestTriangle[j]), m_topology->getPY(closestTriangle[j]), m_topology->getPZ(closestTriangle[j]));
        Real d = (closestProjectionOnTriangle - p).norm();
        if (id_method == 2)
            distances[closestTriangle[j]] = minDistanceToTriangle + d;
        else if (id_method == 1)
            distances[closestTriangle[j]] = (p - cablePositions[indices[i]]).norm();
        queue.insert(DistanceToPoint(d,m_closestCenterIndices[i]));
        }


        // Dijkstra
        while(!queue.empty() )
        {
        DistanceToPoint top = *queue.begin();
        queue.erase(queue.begin());
        BaseMeshTopology::PointID v = top.second; 
        const vector<BaseMeshTopology::PointID> neighboors = m_topology->getVerticesAroundVertex(v);
        for (unsigned int j=0 ; j<neighboors.size(); ++j)
        {
            sofa::core::topology::BaseMeshTopology::PointID vn = neighboors[j];

            Coord pv = Coord(m_topology->getPX(v), m_topology->getPY(v), m_topology->getPZ(v));
            Coord pvn = Coord(m_topology->getPX(vn), m_topology->getPY(vn), m_topology->getPZ(vn));
            Real d = 0.0;
            if (id_method == 2)
                d = distances[v] + (pv - pvn).norm();
            else if (id_method == 1)
                d = (pvn - cablePositions[indices[i]]).norm();

            if((distances[vn]) > d)
            {
                qit=queue.find(DistanceToPoint(distances[vn],vn));
                if(qit != queue.end()) queue.erase(qit);
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
                auto ratio = rabs(distances[p] - maxDistance[i]);

                m_totalRatio[i] += ratio;
                m_ratios[i].push_back(ratio);
                m_areaIndices[i].push_back(p);
            } 
        }

        // Normalize force ratios
        for(unsigned int j=0; j<m_ratios[i].size(); j++)
            m_ratios[i][j] = m_ratios[i][j] / m_totalRatio[i];
    }
}


template<class DataTypes>
unsigned int CableModel<DataTypes>::computeClosestIndice(Coord position)
{
    auto closest_distance = (position - Coord(m_topology->getPX(0), m_topology->getPY(0), m_topology->getPZ(0))).norm();
    unsigned int closest_vertice = 0;
    for(unsigned int j=1; j<m_topology->getNbPoints(); j++)
    {
        const auto distance = (position - Coord(m_topology->getPX(j), m_topology->getPY(j), m_topology->getPZ(j))).norm();
        if(distance < closest_distance) 
        {
            closest_distance = distance;
            closest_vertice = j;
        }
    } 
   return closest_vertice;
}

template<class DataTypes>
SReal CableModel<DataTypes>::getCableLength(const VecCoord &positions)
{
    const SetIndexArray &indices = d_indices.getValue();
    Coord previousPosition = d_pullPoint.getValue();

    // if no fixed pull point, we take the first point of the list
    if (!d_hasPullPoint.getValue())
        previousPosition = positions[ indices[0] ];

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
            return ;

    SOFA_UNUSED(cParams);

    m_constraintId = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_constraintId);

    VecCoord positions = x.getValue();

    const unsigned int id_method = d_method.getValue().getSelectedId();

    if(!m_hasSlidingPoint)
    {

        if ( d_hasPullPoint.getValue())
        {
            Deriv direction = DataTypes::coordDifference(d_pullPoint.getValue(),positions[d_indices.getValue()[0]]);
            direction.normalize();
            if (id_method == 0)
                rowIterator.setCol(d_indices.getValue()[0],  direction);
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
    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void CableModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                   BaseVector *resV,
                                                   const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    d_cableLength.setValue(getCableLength(m_state->readPositions().ref()));
    Real dfree = Jdx->element(0) + d_cableInitialLength.getValue() - d_cableLength.getValue();
    resV->set(m_constraintId, dfree);
}


template<class DataTypes>
void CableModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                        core::MultiVecDerivId res,
                                        const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    d_force.setValue(lambda->element(m_constraintId));

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
            return ;

    if (!vparams->displayFlags().getShowInteractionForceFields()) return;

    drawLinesBetweenPoints(vparams);

    if(d_drawPoints.getValue())
        drawPoints(vparams);

    if(d_drawPullPoint.getValue())
        drawPullPoint(vparams);

    const unsigned int id_method = d_method.getValue().getSelectedId();
    if (id_method == 1 || id_method == 2)
        if(d_drawPulledAreas.getValue())
            drawPulledAreas(vparams);
}


template<class DataTypes>
void CableModel<DataTypes>::drawPullPoint(const VisualParams* vparams)
{
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vector3> points(1);
    points[0] = d_pullPoint.getValue();
    if(!d_hasPullPoint.getValue() && indices.size()>=1)
    {
        ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
        points[0] = positions[indices[0]];
    }

    vparams->drawTool()->drawPoints(points, 5, RGBAColor(1.f,1.f,0.f,1.f));
}


template<class DataTypes>
void CableModel<DataTypes>::drawPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vector3> points(indices.size());
    for (unsigned int i=0; i<indices.size(); i++)
        points[i] = positions[indices[i]];

    vparams->drawTool()->drawPoints(points, 5, RGBAColor(1.f,0.f,0.f,1.f));
}


template<class DataTypes>
void CableModel<DataTypes>::drawLinesBetweenPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

    const SetIndexArray &indices = d_indices.getValue();

    RGBAColor color = d_color.getValue();
    vector<Vector3> points(indices.size()*2);
    Coord previousPosition = d_pullPoint.getValue();
    unsigned int first = 0;

    if(!d_hasPullPoint.getValue() && indices.size()>=1)
    {
        previousPosition = positions[indices[0]];
        first = 1;
    }

    for (unsigned int i=first; i<indices.size(); i++)
    {
        Coord currentPosition = positions[indices[i]];
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
            vector<Vector3> point(1);
            point[0] = positions[m_areaIndices[i][j]];
            vparams->drawTool()->drawPoints(point, 40.0 * m_ratios[i][j], RGBAColor::yellow());
        }
}



} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
