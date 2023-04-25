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

#include <sofa/config.h>
#include <sofa/core/visual/VisualParams.h>

#include <SoftRobots/component/engine/CenterOfMass.h>

namespace softrobots::engine
{

using sofa::core::objectmodel::ComponentState;
using sofa::helper::ReadAccessor;
using sofa::helper::WriteAccessor;
using sofa::core::ConstVecCoordId;
using sofa::core::objectmodel::BaseData ;
using sofa::core::visual::VisualParams ;
using sofa::type::RGBAColor ;
using sofa::type::Vec3 ;
using sofa::type::vector;

template <class DataTypes>
CenterOfMass<DataTypes>::CenterOfMass()
    : d_positions(initData(&d_positions,"position","If not set by user, find the context mechanical."))
    , d_centerOfMass(initData(&d_centerOfMass,"centerOfMass",""))
    , d_visualization(initData(&d_visualization,"visualization","If set to true, will draw the center of mass"))
    , d_visuSize(initData(&d_visuSize, float(1.0), "visuSize",""))
    , d_visuColor(initData(&d_visuColor, RGBAColor(1.,0.,0.,1.), "visuColor",""))
{
    d_centerOfMass.setReadOnly(true);
}


template <class DataTypes>
CenterOfMass<DataTypes>::~CenterOfMass()
{
}


template <class DataTypes>
void CenterOfMass<DataTypes>::init()
{
    d_componentState = ComponentState::Invalid;

    addInput(&d_positions);
    addOutput(&d_centerOfMass);

    m_state = dynamic_cast<MechanicalState*>(getContext()->getMechanicalState());
    m_mass  = dynamic_cast<Mass*>(getContext()->getMass());

    if(m_state)
        d_positions.setParent(m_state->findData("position")); // Links d_positions to m_state.position
    else
    {
        msg_error() << "No mechanical state found in the context. The component is deactivated.";
        return;
    }

    if(m_mass == nullptr)
    {
        msg_error() << "No mass found in the context. The component is deactivated.";
        return;
    }

    computeCenterOfMass();
    d_componentState = ComponentState::Valid;
}


template <class DataTypes>
void CenterOfMass<DataTypes>::reinit()
{
}


template <class DataTypes>
void CenterOfMass<DataTypes>::doUpdate()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    computeCenterOfMass();
}


template <class DataTypes>
void CenterOfMass<DataTypes>::computeCenterOfMass()
{
    ReadAccessor<sofa::Data<VecCoord> > positions = d_positions;
    int nbPoints = positions.size();

    Real totalMass = 0.0;
    Coord centerOfMass(0.,0.,0.);
    for(int i=0; i<nbPoints; i++)
    {
        Real mass = m_mass->getElementMass(i);
        totalMass += mass;
        centerOfMass += positions[i]*mass;
    }
    centerOfMass /= totalMass;
    d_centerOfMass.setValue(centerOfMass);
}


template<class DataTypes>
void CenterOfMass<DataTypes>::draw(const VisualParams *vparams)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    vector<Vec3> points;
    points.push_back(d_centerOfMass.getValue());
    if(d_visualization.getValue())
        vparams->drawTool()->drawPoints(points, d_visuSize.getValue(), d_visuColor.getValue());
}


} // namespace

