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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_UNILATERALPLANECONSTRAINT_INL
#define SOFA_COMPONENT_CONSTRAINTSET_UNILATERALPLANECONSTRAINT_INL

#include "UnilateralPlaneConstraint.h"

#include <sofa/core/visual/VisualParams.h>

#include <sofa/helper/logging/Messaging.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::VecCoordId;
using sofa::core::ConstVecCoordId ;
using sofa::helper::WriteAccessor ;
using sofa::helper::ReadAccessor ;
using sofa::helper::vector ;
using defaulttype::Vec;
using defaulttype::Vec4f;
using defaulttype::Vector3;

template<class DataTypes>
UnilateralPlaneConstraint<DataTypes>::UnilateralPlaneConstraint(MechanicalState* object)
    : Inherit1(object)
    , d_indices(initData(&d_indices,"indices","Four indices: \n"
                         "-First one for the constrained point \n"
                         "-The others to describe the plane"))

    , d_flipNormal(initData(&d_flipNormal,false,"flipNormal","The normal must be to the direction of the point"))

    , m_columnIndex(0)
{
}


template<class DataTypes>
UnilateralPlaneConstraint<DataTypes>::~UnilateralPlaneConstraint()
{
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::init()
{
    m_componentstate = ComponentState::Invalid;
    Inherit1::init();

    if(mstate == nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. Component deactivated."
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    checkIndicesRegardingState();
    m_componentstate = ComponentState::Valid;
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::reinit()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    checkIndicesRegardingState();
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord> > positions = *mstate->read(VecCoordId::position());

    for(int i=0; i<4; i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error() << "Indices at index " << i << " is to large regarding mechanicalState [position] size" ;
        if (d_indices.getValue()[i] < 0)
            msg_error() << "Indices at index " << i << " is negative" ;
    }
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::buildConstraintMatrix(const core::ConstraintParams* cParams,
                                                                  DataMatrixDeriv &cMatrix,
                                                                  unsigned int &cIndex,
                                                                 const DataVecCoord &x)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_columnIndex = cIndex;

    MatrixDeriv& column = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = column.writeLine(m_columnIndex);

    ReadAccessor<Data<VecCoord> > positions = *mstate->read(VecCoordId::position());
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
    cMatrix.endEdit();
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::getConstraintViolation(const core::ConstraintParams* cParams,
                                                                   defaulttype::BaseVector *resV,
                                                                   const DataVecCoord &xfree,
                                                                   const DataVecDeriv &vfree)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(vfree);

    Coord p0 = xfree.getValue()[d_indices.getValue()[0]];
    Coord p1 = xfree.getValue()[d_indices.getValue()[1]];
    Coord p2 = xfree.getValue()[d_indices.getValue()[2]];
    Coord p3 = xfree.getValue()[d_indices.getValue()[3]];
    Deriv normal = (p2-p1).cross(p3-p1);

    if(d_flipNormal.getValue())
        normal = -normal;

    Real dfree = (p0-p1)*normal/normal.norm();

    resV->set(m_columnIndex, dfree );
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::getConstraintResolution(const ConstraintParams* cParams,
                                                                   std::vector<core::behavior::ConstraintResolution*>& resTab,
                                                                   unsigned int& offset)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    resTab[offset] = new UnilateralPlaneConstraintResolution(1);
    offset++;
}


template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::draw(const VisualParams* vparams)
{
    if(m_componentstate != ComponentState::Valid)
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
    ReadAccessor<Data<VecCoord> > positions = *mstate->read(VecCoordId::position());

    unsigned int nbPoints = 4;
    vector<Vector3> points(nbPoints);
    for (unsigned int i=0; i<nbPoints; i++)
        points[i] = positions[d_indices.getValue()[i]];

    vparams->drawTool()->drawPoints(points, 5, Vec4f(0.9,0.4,0,1));
}

template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::drawTriangles(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord> > positions = *mstate->read(VecCoordId::position());

    vector<Vector3> points(3);
    for (unsigned int i=0; i<3; i++)
        points[i] = positions[d_indices.getValue()[i+1]];

    vparams->drawTool()->drawTriangles(points, Vec4f(1,0.6,0.2,1));

}

template<class DataTypes>
void UnilateralPlaneConstraint<DataTypes>::drawArrows(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord> > positions = *mstate->read(VecCoordId::position());

    Coord p1 = positions[d_indices.getValue()[1]];
    Coord p2 = positions[d_indices.getValue()[2]];
    Coord p3 = positions[d_indices.getValue()[3]];
    Deriv normal = (p2-p1).cross(p3-p1);
    if(d_flipNormal.getValue())
        normal = -normal;
    normal.normalize();

    Vec4f color(0.9,0.4,0,1);
    vparams->drawTool()->drawArrow((p1+p2+p3)/3., (p1+p2+p3)/3. + normal, 0.1, color, 4);
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
