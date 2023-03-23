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
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#define SOFTROBOTS_POSITIONMODEL_CPP
#include <SoftRobots/component/constraint/model/PositionModel.inl>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using namespace sofa::defaulttype;
using core::ConstraintParams;


template<>
void PositionModel<Rigid3Types>::normalizeDirections()
{
    VecDeriv directions;
    directions.resize(6);
    for(unsigned int i=0; i<6; i++)
    {
        directions[i] = d_directions.getValue()[i];
        Vec<3, Real> vector1 = Vec<3, Real>(directions[i][0],directions[i][1],directions[i][2]);
        Vec<3, Real> vector2 = Vec<3, Real>(directions[i][3],directions[i][4],directions[i][5]);
        vector1.normalize();
        vector2.normalize();
        directions[i] = Deriv(vector1,vector2);
    }
    d_directions.setValue(directions);
}

template<>
void PositionModel<Vec3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  RGBAColor& color) {
    vparams->drawTool()->drawPoints(points, size, color);
}

template<>
void PositionModel<Vec2Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  RGBAColor& color) {
    vector<Vec3> pointsVec3;
    for (auto point: points)
        pointsVec3.push_back(Vec3(point[0], point[1], 0.));
    vparams->drawTool()->drawPoints(pointsVec3, size, color);
}

template<>
void PositionModel<Rigid3Types>::drawPoints(const VisualParams* vparams, const std::vector<Coord> &points, float size,  const  RGBAColor& color) {
    vector<Vec3> pointsVec3;
    for (auto point: points)
        pointsVec3.push_back(point.getCenter());
    vparams->drawTool()->drawPoints(pointsVec3, size, color);
}

using namespace sofa::defaulttype;
template class PositionModel<Vec3Types>;
template class PositionModel<Rigid3Types>;
template class PositionModel<Vec2Types>;

} // namespace constraintset

} // namespace component

} // namespace sofa
