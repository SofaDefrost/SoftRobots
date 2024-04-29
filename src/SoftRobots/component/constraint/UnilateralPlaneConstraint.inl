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
#include <sofa/helper/logging/Messaging.h>

#include <SoftRobots/component/constraint/UnilateralPlaneConstraint.h>

namespace softrobots::constraint
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::VecCoordId;
using sofa::core::ConstVecCoordId ;
using sofa::helper::WriteAccessor ;
using sofa::helper::ReadAccessor ;
using sofa::type::vector;
using sofa::type::Vec3;
using sofa::type::RGBAColor;

template<class DataTypes>
UnilateralPlaneConstraint<DataTypes>::UnilateralPlaneConstraint(MechanicalState* object)
    : Inherit1(object)
    , d_indices(initData(&d_indices,"indices","Four indices: \n"
                         "-First one for the constrained point \n"
                         "-The others to describe the plane"))

    , d_flipNormal(initData(&d_flipNormal,false,"flipNormal","The normal must be to the direction of the point"))
{
}


template<class DataTypes>
UnilateralPlaneConstraint<DataTypes>::~UnilateralPlaneConstraint()
{
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    Inherit1::init();

    if(m_state == nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. Component deactivated."
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    checkIndicesRegardingState();
    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    checkIndicesRegardingState();
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();

    for(int i=0; i<4; i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error() << "Indices at index " << i << " is to large regarding mechanicalState [position] size" ;
    }
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                                  DataMatrixDeriv &cMatrix,
                                                                  unsigned int &cIndex,
                                                                 const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    m_constraintIndex.setValue(cIndex);
    const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

    MatrixDeriv& column = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = column.writeLine(constraintIndex);

    ReadAccessor<sofa::Data<VecCoord>> positions = x;
    Coord p1 = positions[d_indices.getValue()[1]];
    Coord p2 = positions[d_indices.getValue()[2]];
    Coord p3 = positions[d_indices.getValue()[3]];
    Deriv normal = (p2-p1).cross(p3-p1);

    if(d_flipNormal.getValue())
        normal = -normal;

    rowIterator.setCol(d_indices.getValue()[0],  normal);
    rowIterator.setCol(d_indices.getValue()[1],  -normal/3.);
    rowIterator.setCol(d_indices.getValue()[2],  -normal/3.);
    rowIterator.setCol(d_indices.getValue()[3],  -normal/3.);

    cIndex++;
    m_nbLines = cIndex - constraintIndex;
    cMatrix.endEdit();
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                                  BaseVector *resV,
                                                                  const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    ReadAccessor<sofa::Data<VecCoord>> x = m_state->readPositions();

    Coord p0 = x[d_indices.getValue()[0]];
    Coord p1 = x[d_indices.getValue()[1]];
    Coord p2 = x[d_indices.getValue()[2]];
    Coord p3 = x[d_indices.getValue()[3]];
    Deriv normal = (p2-p1).cross(p3-p1);

    if(d_flipNormal.getValue())
        normal = -normal;

    Real dfree = Jdx->element(0) + (p0-p1)*normal/normal.norm();
    resV->set(m_constraintIndex.getValue(), dfree );
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParams,
                                                                   std::vector<ConstraintResolution*>& resTab,
                                                                   unsigned int& offset)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    resTab[offset] = new UnilateralPlaneConstraintResolution(1);
    offset++;
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if (!vparams->displayFlags().getShowCollisionModels())
        return;

    drawPoints(vparams);
    drawTriangles(vparams);
    drawArrows(vparams);
}

template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::drawPoints(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();

    unsigned int nbPoints = 4;
    vector<Vec3> points(nbPoints);
    for (unsigned int i=0; i<nbPoints; i++)
        points[i] = positions[d_indices.getValue()[i]];

    vparams->drawTool()->drawPoints(points, 5, RGBAColor(0.9f,0.4f,0.0f,1.0f));
}

template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::drawTriangles(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();

    vector<Vec3> points(3);
    for (unsigned int i=0; i<3; i++)
        points[i] = positions[d_indices.getValue()[i+1]];

    vparams->drawTool()->drawTriangles(points, RGBAColor(1.0f,0.6f,0.2f,1.0f));

}

template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::drawArrows(const VisualParams* vparams)
{
    ReadAccessor<sofa::Data<VecCoord>> positions = m_state->readPositions();

    Coord p1 = positions[d_indices.getValue()[1]];
    Coord p2 = positions[d_indices.getValue()[2]];
    Coord p3 = positions[d_indices.getValue()[3]];
    Deriv normal = (p2-p1).cross(p3-p1);
    if(d_flipNormal.getValue())
        normal = -normal;
    normal.normalize();

    static constexpr RGBAColor color(0.9f,0.4f,0.0f,1.0f);
    vparams->drawTool()->drawArrow((p1+p2+p3)/3., (p1+p2+p3)/3. + normal, 0.1, color, 4);
}

} // namespace

