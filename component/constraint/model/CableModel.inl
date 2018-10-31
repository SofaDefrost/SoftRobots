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
#include <sofa/defaulttype/Vec.h>

#include "CableModel.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;
using sofa::defaulttype::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::defaulttype::Vec4f;
using sofa::defaulttype::Vector3;
using sofa::helper::vector;

template<class DataTypes>
CableModel<DataTypes>::CableModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    // To remove in SoftRobots v19.0
    ,  d_indexDeprecated(initData(&d_indexDeprecated, "index",
                                 "Deprecated, must be replaced by the field name [indices]"))
    //

    , d_indices(initData(&d_indices, "indices",
                         "List of points connected by the cable (from extremity to actuated point). \n"
                         "If no indices are given, default value is 0. \n"
                         "In case of multiple indices, one point will be actuated \n"
                         "and the others will represent sliding points for the cable."))

    , d_pullPoint(initData(&d_pullPoint, Coord(0.0, 0.0, 0.0), "pullPoint",
                                          "Fixed point from which the cable is pulled. \n"
                                          "If unspecified, the default value is {0.0,0.0,0.0}"))

    , d_hasPullPoint(initData(&d_hasPullPoint, true ,"hasPullPoint",
                              "If false, the pull point is not considered and the cable is entirely mapped \n"
                              " In that case, needs at least 2 different point in indices."))

    , d_cableInitialLength(initData(&d_cableInitialLength, (Real)0.0, "cableInitialLength"," "))

    , d_cableLength(initData(&d_cableLength, (Real)0.0, "cableLength","Computation done at the end of the time step"))

    , d_force(initData(&d_force,(double)0.0, "force",
                                         "Output force."))

    , d_displacement(initData(&d_displacement,(double)0.0, "displacement",
                          "Output displacement compared to the initial cable length."))

    , d_maxForce(initData(&d_maxForce,(Real)0.0, "maxForce",
                          "Maximum force of the actuator. \n"
                          "If unspecified no maximum value will be considered."))

    , d_minForce(initData(&d_minForce,(Real)0.0, "minForce",
                          "Minimum force of the actuator. \n"
                          "If unspecified no minimum value will be considered \n"
                          "and the cable will then be seen as a stiff rod able to push."))

    , d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement,(Real)0.0, "maxPositiveDisp",
                                         "Maximum displacement of the actuator in the positive direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement,(Real)0.0, "maxNegativeDisp",
                                         "Maximum displacement of the actuator in the negative direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxDispVariation(initData(&d_maxDispVariation,(Real)0.0, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_drawPullPoint(initData(&d_drawPullPoint,true, "drawPullPoint",
                          ""))

    , d_drawPoints(initData(&d_drawPoints,true, "drawPoints",
                          ""))

    , d_color(initData(&d_color,Vec4f(0.4,0.4,0.4,1), "color",
                          "Color of the string."))

{
    setUpData();
}


template<class DataTypes>
CableModel<DataTypes>::CableModel()
    : SoftRobotsConstraint<DataTypes>()
    // To remove in SoftRobots v19.0
    , d_indexDeprecated(initData(&d_indexDeprecated, "index",
                                 "Deprecated, must be replaced with the field name [indices]"))
    //

    , d_indices(initData(&d_indices, "indices",
                         "List of points connected by the cable (from extremity to actuated point). \n"
                         "If no indices are given, default value is 0. \n"
                         "In case of multiple indices, one point will be actuated \n"
                         "and the others will represent sliding points for the cable."))


    , d_pullPoint(initData(&d_pullPoint, Coord(0.0, 0.0, 0.0), "pullPoint",
                                          "Fixed point from which the cable is pulled. \n"
                                          "If unspecified, the default value is {0.0,0.0,0.0}"))

    , d_hasPullPoint(initData(&d_hasPullPoint, true ,"hasPullPoint",
                              "If false, the pull point is not considered and the cable is entirely mapped \n"
                              " In that case, needs at least 2 different point in indices."))

    , d_cableInitialLength(initData(&d_cableInitialLength, (Real)0.0, "initialCableLength"," "))

    , d_cableLength(initData(&d_cableLength, (Real)0.0, "cableLength","Computation done at the end of the time step"))

    , d_force(initData(&d_force,(double)0.0, "force",
                                         "Output force."))

    , d_displacement(initData(&d_displacement,(double)0.0, "displacement",
                          "Output displacement compared to the initial cable length."))

    , d_maxForce(initData(&d_maxForce,(Real)0.0, "maxForce",
                          "Maximum force of the actuator. \n"
                          "If unspecified no maximum value will be considered."))

    , d_minForce(initData(&d_minForce,(Real)0.0, "minForce",
                          "Minimum force of the actuator. \n"
                          "If unspecified no minimum value will be considered \n"
                          "and the cable will then be seen as a stiff rod able to push."))

    , d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement,(Real)0.0, "maxPositiveDisp",
                                         "Maximum displacement of the actuator in the positive direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement,(Real)0.0, "maxNegativeDisp",
                                         "Maximum displacement of the actuator in the negative direction. \n"
                                         "If unspecified no maximum value will be considered."))

    , d_maxDispVariation(initData(&d_maxDispVariation,(Real)0.0, "maxDispVariation",
                                   "Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

    , d_drawPullPoint(initData(&d_drawPullPoint,true, "drawPullPoint",
                          ""))

    , d_drawPoints(initData(&d_drawPoints,true, "drawPoints",
                          ""))

    , d_color(initData(&d_color,Vec4f(0.4,0.4,0.4,1), "color",
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
    // To remove in SoftRobots v19.0
    d_indexDeprecated.setDisplayed(false);
    //

    d_cableInitialLength.setReadOnly(true);
    d_cableLength.setReadOnly(true);

    d_force.setGroup("Vector");
    d_displacement.setGroup("Vector");
    d_force.setReadOnly(true);
    d_displacement.setReadOnly(true);

    d_drawPullPoint.setGroup("Visualization");
    d_drawPoints.setGroup("Visualization");
    d_color.setGroup("Visualization");
}

template<class DataTypes>
void CableModel<DataTypes>::init()
{
    m_componentstate = ComponentState::Invalid;
    SoftRobotsConstraint<DataTypes>::init();

    // To remove in SoftRobots v19.0
    if(d_indexDeprecated.isSet())
        msg_warning(this) << "The field of the Cable component named 'index' is now deprecated. "
                             "To remove this warning message, the field "
                             "'index' should be replaced by the field 'indices'." ;
    //

    if(m_state==nullptr)
    {
        msg_error(this) << "There is no mechanical state associated with this node. "
                           "The object is then deactivated. "
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    if(m_state->read(VecCoordId::position())==nullptr)
    {
        msg_error(this) << "There is no position vector in the MechanicalState. "
                             "The object is deactivated. " ;
        return;
    }

    internalInit();
    m_componentstate = ComponentState::Valid;
}


template<class DataTypes>
void CableModel<DataTypes>::bwdInit()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    // The initial length of the cable is computed in bwdInit so the mapping (if there is any)
    // will be considered
    VecCoord positions = (*m_state->read(VecCoordId::position())).getValue();
    VecCoord restPositions = (*m_state->read(VecCoordId::restPosition())).getValue();

    Real cableLength = getCableLength(positions);
    Real initialCableLength = getCableLength(restPositions);
    d_cableInitialLength.setValue(initialCableLength);
    d_cableLength.setValue(cableLength);
}


template<class DataTypes>
void CableModel<DataTypes>::reinit()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    internalInit();
}


template<class DataTypes>
void CableModel<DataTypes>::reset()
{
    if(m_componentstate != ComponentState::Valid)
        return ;

    d_cableLength.setValue(d_cableInitialLength.getValue());
}



template<class DataTypes>
void CableModel<DataTypes>::internalInit()
{
    initActuatedPoints();
    checkIndicesRegardingState();
}


template<class DataTypes>
void CableModel<DataTypes>::initActuatedPoints()
{
    int nbIndices = d_indices.getValue().size();
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(VecCoordId::position());
    const SetIndexArray &indices = d_indices.getValue();

    if ( !d_hasPullPoint.getValue()){
        if (nbIndices<=1)
            msg_error(this) <<"If no pull point, at least two indices of actuation should be given";
        else
        {
            nbIndices-=1;  // the first point of the list considered as the pull point
            d_pullPoint.setValue(positions[indices[0]]);
        }
    }

    if (nbIndices == 0)
    {
        SetIndexArray &list = (*d_indices.beginEdit());
        msg_warning(this) <<"No index of actuation given, set default 0";
        list.push_back(0);
        d_indices.endEdit();

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
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(VecCoordId::position());

    for(unsigned int i=0; i<d_indices.getValue().size(); i++)
    {
        if (positions.size() <= d_indices.getValue()[i])
            msg_error(this) << "Indices at index " << i << " is too large regarding mechanicalState [position] size" ;
        if (d_indices.getValue()[i] < 0)
            msg_error(this) << "Indices at index " << i << " is negative" ;
    }
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
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

    m_columnIndex = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_columnIndex);

    if(!m_hasSlidingPoint)
    {

        if ( d_hasPullPoint.getValue())
        {
            Deriv direction = d_pullPoint.getValue() - x.getValue()[d_indices.getValue()[0]];
            direction.normalize();
            rowIterator.setCol(d_indices.getValue()[0],  direction);
        }
        else
        {
            Deriv direction = x.getValue()[d_indices.getValue()[1]] - x.getValue()[d_indices.getValue()[0]];
            direction.normalize();
            rowIterator.setCol(d_indices.getValue()[1],  -direction);
            rowIterator.setCol(d_indices.getValue()[0],  direction);
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

                previousPosition = (d_hasPullPoint.getValue()) ? d_pullPoint.getValue(): x.getValue()[d_indices.getValue()[0]];
                currentPosition  = x.getValue()[currentIndex];
                nextPosition     = x.getValue()[nextIndex];

                Deriv directionBeyond = previousPosition - currentPosition;
                directionBeyond.normalize();

                Deriv directionAhead  = currentPosition - nextPosition;
                directionAhead.normalize();

                Deriv slidingDirection = directionBeyond - directionAhead;
                if (d_hasPullPoint.getValue())
                    rowIterator.setCol(currentIndex, slidingDirection);
                else
                    rowIterator.setCol(currentIndex, -directionAhead);
            }
            else if (i != indices.size()-1)  // all points except extremities
            {
                int previousIndex = indices[i-1];
                int currentIndex  = indices[i];
                int nextIndex     = indices[i+1];

                previousPosition = x.getValue()[previousIndex];
                currentPosition  = x.getValue()[currentIndex];
                nextPosition     = x.getValue()[nextIndex];

                Deriv directionBeyond = previousPosition - currentPosition;
                directionBeyond.normalize();

                Deriv directionAhead  = currentPosition - nextPosition;
                directionAhead.normalize();

                Deriv slidingDirection = directionBeyond - directionAhead;

                rowIterator.setCol(currentIndex, slidingDirection);
            }
            else // end point of the cable
            {
                int previousIndex = indices[i-1];
                int currentIndex  = indices[i];

                previousPosition = x.getValue()[previousIndex];
                currentPosition  = x.getValue()[currentIndex];

                Deriv directionOfActuation = previousPosition - currentPosition;
                directionOfActuation.normalize();

                rowIterator.setCol(currentIndex, directionOfActuation);
            }
        }
    }
    cIndex++;
    cMatrix.endEdit();
    m_nbLines = cIndex - m_columnIndex;
}


template<class DataTypes>
void CableModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams, BaseVector *resV, const DataVecCoord &xfree, const DataVecDeriv &vfree)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(vfree);
    const VecCoord& positions =  xfree.getValue();

    Real currentLength = getCableLength(positions);

    resV->set(m_columnIndex, (d_cableInitialLength.getValue() - currentLength) );
}


template<class DataTypes>
void CableModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                        core::MultiVecDerivId res,
                                        const sofa::defaulttype::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(m_componentstate != ComponentState::Valid)
            return ;

    d_force.setValue(lambda->element(m_columnIndex));

    // Compute actual cable length and displacement from updated positions of mechanical
    const VecCoord& position = m_state->read(VecCoordId::position())->getValue();
    d_cableLength.setValue(getCableLength(position));
    d_displacement.setValue(d_cableInitialLength.getValue()-d_cableLength.getValue());
}

template<class DataTypes>
void CableModel<DataTypes>::draw(const VisualParams* vparams)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if (!vparams->displayFlags().getShowInteractionForceFields()) return;

    drawLinesBetweenPoints(vparams);

    if(d_drawPoints.getValue())
        drawPoints(vparams);

    if(d_drawPullPoint.getValue())
        drawPullPoint(vparams);
}


template<class DataTypes>
void CableModel<DataTypes>::drawPullPoint(const VisualParams* vparams)
{
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vector3> points(1);
    points[0] = d_pullPoint.getValue();
    if(!d_hasPullPoint.getValue() && indices.size()>=1)
    {
        ReadAccessor<Data<VecCoord> > positions = *m_state->read(core::ConstVecCoordId::position());
        points[0] = positions[indices[0]];
    }

    vparams->drawTool()->drawPoints(points, 5, Vec4f(1.f,1.f,0.f,1.f));
}


template<class DataTypes>
void CableModel<DataTypes>::drawPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(core::ConstVecCoordId::position());
    const SetIndexArray &indices = d_indices.getValue();

    vector<Vector3> points(indices.size());
    for (unsigned int i=0; i<indices.size(); i++)
        points[i] = positions[indices[i]];

    vparams->drawTool()->drawPoints(points, 5, Vec4f(1.f,0.f,0.f,1.f));
}


template<class DataTypes>
void CableModel<DataTypes>::drawLinesBetweenPoints(const VisualParams* vparams)
{
    ReadAccessor<Data<VecCoord> > positions = *m_state->read(core::ConstVecCoordId::position());

    const SetIndexArray &indices = d_indices.getValue();

    Vec4f color = d_color.getValue();
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


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
