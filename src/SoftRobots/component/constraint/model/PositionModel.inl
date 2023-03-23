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
#pragma once

#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/logging/Messaging.h>

#include <SoftRobots/component/constraint/model/PositionModel.h>

namespace sofa::component::constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::VecCoordId;
using sofa::core::ConstVecCoordId ;
using sofa::helper::WriteAccessor ;
using sofa::helper::ReadAccessor ;
using sofa::type::vector ;
using sofa::type::Vec;
using sofa::type::Vec3;
using sofa::type::RGBAColor;

template<class DataTypes>
PositionModel<DataTypes>::PositionModel(MechanicalState* object)
    : Inherit1(object)
    , d_indices(initData(&d_indices, "indices",
                                 "If indices size is lower than target size, \n"
                                 "some target will not be considered"))

    , d_weight(initData(&d_weight, 1., "weight",
                          "The parameter sets a weight to the minimization."))

    , d_directions(initData(&d_directions,"directions",
                          "The parameter directions allows to specify the directions in \n"
                          "which you want to solve the position."))

    , d_useDirections(initData(&d_useDirections,"useDirections",
                              "The parameter useDirections allows to select the directions in \n"
                              "which you want to solve the position. If unspecified, the default \n"
                              "values are all true."))

    , d_delta(initData(&d_delta, "delta","Distance to target"))
{
    d_delta.setReadOnly(true);
}



template<class DataTypes>
PositionModel<DataTypes>::~PositionModel()
{
}


template<class DataTypes>
void PositionModel<DataTypes>::init()
{
    d_componentState = ComponentState::Valid;
    Inherit1::init();

    if(m_state==nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. "
                        "the object is deactivated. "
                        "To remove this error message fix your scene possibly by "
                        "adding a MechanicalObject." ;
        d_componentState = ComponentState::Invalid;
        return;
    }

    internalInit();
}


template<class DataTypes>
void PositionModel<DataTypes>::reinit()
{
    internalInit();
}


template<class DataTypes>
void PositionModel<DataTypes>::internalInit()
{
    if(!d_directions.isSet())
        setDefaultDirections();
    else
        normalizeDirections();


    if(!d_useDirections.isSet())
    {
        setDefaultUseDirections();
    }
    else
    {
        int count = 0;
        for(Size i=0; i<Deriv::total_size; i++)
            if(d_useDirections.getValue()[i])
                count++;

        if(count==0)
        {
            setDefaultUseDirections();
            msg_warning(this) << "No direction given in useDirection. Set default all.";
        }
    }

    if(!d_indices.isSet())
    {
        msg_warning(this) <<"Indices not defined. Default value assigned 0.";
        setIndicesDefaultValue();
    }

    if(d_indices.getValue().size() > m_state->getSize())
    {
        msg_warning(this) <<"Indices size can not be larger than the number of point in the context. Launch resize process.";
        resizeIndicesRegardingState();
    }

    if(d_indices.getValue().size() == 0)
    {
        msg_error(this) <<"Indices size is zero. The component will not work.";
        d_componentState = ComponentState::Invalid;
        return;
    }

    checkIndicesRegardingState();
}


template<class DataTypes>
void PositionModel<DataTypes>::checkIndicesRegardingState()
{
    ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();

    if(d_indices.getValue().size() > positions.size())
    {
        msg_error(this) << "Indices size is larger than mechanicalState size" ;
        d_componentState = ComponentState::Invalid;
        return;
    }

    const auto& indices = d_indices.getValue();
    for(unsigned int i=0; i<indices .size(); i++)
    {
        if (positions.size() <= indices[i])
        {
            msg_error(this) << "Indices at index " << i << " is to large regarding mechanicalState [position] size" ;
            d_componentState = ComponentState::Invalid;
            return;
        }
    }
}




template<class DataTypes>
void PositionModel<DataTypes>::setIndicesDefaultValue()
{
    WriteAccessor<Data<vector<unsigned int> > > defaultIndices = d_indices;
    defaultIndices.resize(1);
}

template<class DataTypes>
void PositionModel<DataTypes>::resizeIndicesRegardingState()
{
    WriteAccessor<Data<vector<unsigned int>>> indices = d_indices;
    indices.resize(m_state->getSize());
}



template<class DataTypes>
void PositionModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                        DataMatrixDeriv &cMatrix,
                                                        unsigned int &cIndex,
                                                        const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_constraintId = cIndex;
    MatrixDeriv& column = *cMatrix.beginEdit();
    sofa::Index sizeIndices = d_indices.getValue().size();

    unsigned int index = 0;
    for (unsigned i=0; i<sizeIndices; i++)
    {
        for(Size j=0; j<Deriv::total_size; j++)
        {
            if(d_useDirections.getValue()[j])
            {
                MatrixDerivRowIterator rowIterator = column.writeLine(m_constraintId+index);
                rowIterator.setCol(d_indices.getValue()[i], d_directions.getValue()[j]*d_weight.getValue());
                index++;
            }
        }
    }

    cIndex += index;
    cMatrix.endEdit();

    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void PositionModel<DataTypes>::storeResults(vector<double> &delta)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    d_delta.setValue(delta);
}


template<class DataTypes>
void PositionModel<DataTypes>::setDefaultDirections()
{
    VecDeriv directions;
    directions.resize(Deriv::total_size);
    for(Size i=0; i<Deriv::total_size; i++)
        directions[i][i] = 1.;
    d_directions.setValue(directions);
}


template<class DataTypes>
void PositionModel<DataTypes>::setDefaultUseDirections()
{
    Vec<Deriv::total_size,bool> useDirections;
    for(Size i=0; i<Deriv::total_size; i++)
        useDirections[i] = true;
    d_useDirections.setValue(useDirections);
}


template<class DataTypes>
void PositionModel<DataTypes>::normalizeDirections()
{
    WriteAccessor<Data<VecDeriv>> directions = d_directions;
    directions.resize(Deriv::total_size);
    for(unsigned int i=0; i<Deriv::total_size; i++)
        directions[i].normalize();
}


template<class DataTypes>
void PositionModel<DataTypes>::draw(const VisualParams* vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return;

    if (!vparams->displayFlags().getShowInteractionForceFields())
        return;

    vector<Coord> points;
    ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
    ReadAccessor<Data<type::vector<sofa::Index>> > indices = d_indices;
    for (unsigned int i=0; i<indices.size(); i++)
    {
        points.push_back(positions[indices[i]]);
    }
    drawPoints(vparams, points, 10.0f, RGBAColor(0.,1.,0.,1.));
}

} // namespace constraintset

} // namespace component

} // namespace sofa

