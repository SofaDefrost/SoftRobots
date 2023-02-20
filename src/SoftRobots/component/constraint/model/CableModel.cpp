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
#define SOFTROBOTS_COMPONENT_BEHAVIOR_CONSTRAINT_MODEL_CABLEMODEL_CPP
#include <sofa/defaulttype/VecTypes.h>
#include <SoftRobots/component/constraint/model/CableModel.inl>

namespace sofa::component::constraintset
{


using sofa::defaulttype::Vec2Types;
using sofa::defaulttype::Vec3Types;
using sofa::type::Vector2;

template<>
void CableModel<Vec2Types>::getPositionFromTopology(Coord& position, sofa::Index index)
{
    position.x() = l_surfaceTopology.get()->getPX(index);
    position.y() = l_surfaceTopology.get()->getPY(index);
}

template<>
SReal CableModel<Vec2Types>::getDistanceToTriangle(const Coord& position, const Triangle& triangle, Coord& projectionOnTriangle)
{
    static sofa::helper::DistancePointTri proximitySolver;
    Coord p0 { type::NOINIT }; 
    Coord p1 { type::NOINIT }; 
    Coord p2 { type::NOINIT };
    getPositionFromTopology(p0, triangle[0]);
    getPositionFromTopology(p1, triangle[1]);
    getPositionFromTopology(p2, triangle[2]);  
    Vector3 projectionOnTriangle3D;
    proximitySolver.NewComputation(Vec3(p0[0], p0[1], 0.0), Vec3(p1[0], p1[1], 0.0), Vec3(p2[0], p2[1], 0.0), 
                                                        Vec3(position[0], position[1], 0.0), projectionOnTriangle3D);
    projectionOnTriangle = Coord(projectionOnTriangle[0], projectionOnTriangle[1]);
    const Real distanceToTriangle = (projectionOnTriangle - position).norm();  
    return distanceToTriangle;
}


template<>
void CableModel<Vec3Types>::getPositionFromTopology(Coord& position, sofa::Index index)
{
    position.x() = l_surfaceTopology.get()->getPX(index);
    position.y() = l_surfaceTopology.get()->getPY(index);
    position.z() = l_surfaceTopology.get()->getPZ(index);
}

template<>
SReal CableModel<Vec3Types>::getDistanceToTriangle(const Coord& position, const Triangle& triangle, Coord& projectionOnTriangle)
{
    static sofa::helper::DistancePointTri proximitySolver;
    Coord p0 { type::NOINIT }; 
    Coord p1 { type::NOINIT }; 
    Coord p2 { type::NOINIT };
    getPositionFromTopology(p0, triangle[0]);
    getPositionFromTopology(p1, triangle[1]);
    getPositionFromTopology(p2, triangle[2]);  
    proximitySolver.NewComputation(p0, p1, p2, position, projectionOnTriangle);
    const Real distanceToTriangle = (projectionOnTriangle - position).norm();  
    return distanceToTriangle;
}


// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
using namespace sofa::defaulttype;
template class CableModel<Vec3Types>;
//template class CableModel<Rigid3Types>;
template class CableModel<Vec2Types>;


} // namespace

