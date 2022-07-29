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

#include <sofa/defaulttype/VecTypes.h>
#include "CableModel.inl"

namespace sofa
{

namespace component
{

namespace constraintset
{


using sofa::defaulttype::Vec2Types;
using sofa::type::Vector2;

template<>
unsigned int CableModel<Vec2Types>::computeClosestIndice(Coord position)
{
    double closest_distance = (position - Coord(m_topology->getPX(0), m_topology->getPY(0))).norm();
    unsigned int closest_vertice = 0;
    for(unsigned int j=1; j<m_topology->getNbPoints(); j++)
    {
        double distance = (position - Coord(m_topology->getPX(j), m_topology->getPY(j))).norm();
        if(distance < closest_distance) 
        {
            closest_distance = distance;
            closest_vertice = j;
        }
    } 
   return closest_vertice;
}


template<>
void CableModel<Vec2Types>::computePointsActionArea()
{
    // Implementing continuous Dijkstra as a geodesic distance measure
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
        Real minDistanceToTriangle = std::numeric_limits<Real>::max(); unsigned int closestTriangleId = 0; Vector2 closestProjectionOnTriangle;
        for(unsigned int j=0; j<trianglesAroundClosestVertex.size(); j++)
        {
            const Triangle triangle =  m_topology->getTriangle(trianglesAroundClosestVertex[j]);   
            Vector3 p0 = Vector3(m_topology->getPX(triangle[0]), m_topology->getPY(triangle[0]), 0.0);
            Vector3 p1 = Vector3(m_topology->getPX(triangle[1]), m_topology->getPY(triangle[1]), 0.0);
            Vector3 p2 = Vector3(m_topology->getPX(triangle[2]), m_topology->getPY(triangle[2]), 0.0);           
            proximitySolver.NewComputation(p0, p1, p2, Vector3(cablePositions[indices[i]][0], cablePositions[indices[i]][1], 0.0),projectionOnTriangle);
            Real distanceToTriangle = (Vector2(projectionOnTriangle[0], projectionOnTriangle[1]) - cablePositions[indices[i]]).norm();  
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
        Coord p = Coord(m_topology->getPX(closestTriangle[j]), m_topology->getPY(closestTriangle[j]));
        Real d = (closestProjectionOnTriangle - p).norm();
        if (d_method.getValue() == "geodesic")
            distances[closestTriangle[j]] = minDistanceToTriangle + d;
        else if (d_method.getValue() == "sphere")
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

            Coord pv = Coord(m_topology->getPX(v), m_topology->getPY(v));
            Coord pvn = Coord(m_topology->getPX(vn), m_topology->getPY(vn));
            Real d = 0.0;
            if (d_method.getValue() == "geodesic")
                d = distances[v] + (pv - pvn).norm();
            else if (d_method.getValue() == "sphere")
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
                double ratio = rabs(distances[p] - maxDistance[i]);

                //double f = 1.0 -  (distances[p]/(maxDistance[i] * maxDistance[i]));
                //double ratio = f * f * f * f; // Locally supported kernel function

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



// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
using namespace sofa::defaulttype;
template class CableModel<Vec3Types>;
//template class CableModel<Rigid3Types>;
template class CableModel<Vec2Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa

