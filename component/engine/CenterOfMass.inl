/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture                          *
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
*                           Plugin SoftRobots    v1.0                         *
*				                                              *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ENGINE_CENTEROFMASS_INL
#define SOFA_COMPONENT_ENGINE_CENTEROFMASS_INL

#include "CenterOfMass.h"
#include <sofa/helper/helper.h>

#include <sofa/simulation/AnimateBeginEvent.h>
#include <sofa/core/visual/VisualParams.h>

namespace sofa
{

namespace component
{

namespace engine
{

using helper::ReadAccessor;
using helper::WriteAccessor;
using helper::vector;
using sofa::core::ConstVecCoordId;
using simulation::AnimateBeginEvent ;
using core::objectmodel::Event ;
using core::visual::VisualParams ;
using defaulttype::Vec4f ;
using defaulttype::Vector3 ;

template <class DataTypes>
CenterOfMass<DataTypes>::CenterOfMass()
    : d_centerOfMass(initData(&d_centerOfMass,"centerOfMass",""))
    , d_visualization(initData(&d_visualization,"visualization","If set to true, will draw the center of mass"))
    , d_visuSize(initData(&d_visuSize, (float)1., "visuSize",""))
    , d_visuColor(initData(&d_visuColor, Vec4f(1.,0.,0.,1.), "visuColor",""))
{
    d_centerOfMass.setReadOnly(true);
    setDirtyValue();
}


template <class DataTypes>
CenterOfMass<DataTypes>::~CenterOfMass()
{
}


template <class DataTypes>
void CenterOfMass<DataTypes>::init()
{
    m_state = dynamic_cast<MechanicalState*>(getContext()->getMechanicalState());
    m_mass  = dynamic_cast<Mass*>(getContext()->getMass());

    if(m_state == nullptr)
        msg_warning(this) << "No mechanical state found in the context. The component can not work.";

    if(m_mass == nullptr)
        msg_warning(this) << "No mass found in the context. The component can not work.";

    computeCenterOfMass();
}


template <class DataTypes>
void CenterOfMass<DataTypes>::reinit()
{
    computeCenterOfMass();
}


template <class DataTypes>
void CenterOfMass<DataTypes>::update()
{
    if(m_state)
    {
        cleanDirty();
        computeCenterOfMass();
    }
}


template <class DataTypes>
void CenterOfMass<DataTypes>::computeCenterOfMass()
{
    ReadAccessor<Data<VecCoord> > positions = m_state->read(ConstVecCoordId::position());
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
void CenterOfMass<DataTypes>::handleEvent(Event *event)
{
    if (AnimateBeginEvent::checkEventType(event))
    {
        setDirtyValue();
        update();
    }
}


template<class DataTypes>
void CenterOfMass<DataTypes>::draw(const VisualParams *vparams)
{
    vector<Vector3> points;
    points.push_back(d_centerOfMass.getValue());
    if(d_visualization.getValue())
        vparams->drawTool()->drawPoints(points, d_visuSize.getValue(), d_visuColor.getValue());
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
