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
#ifndef SOFA_COMPONENT_ENGINE_VOLUMEFROMTRIANGLES_INL
#define SOFA_COMPONENT_ENGINE_VOLUMEFROMTRIANGLES_INL

#include "VolumeFromTriangles.h"
#include <sofa/helper/helper.h>

#include <sofa/helper/logging/Messaging.h>
#include <sofa/simulation/AnimateBeginEvent.h>

namespace sofa
{

namespace component
{

namespace engine
{

using sofa::core::objectmodel::ComponentState;
using helper::ReadAccessor;
using helper::vector;
using sofa::core::ConstVecCoordId;
using simulation::AnimateBeginEvent ;
using core::objectmodel::Event ;

template <class DataTypes>
VolumeFromTriangles<DataTypes>::VolumeFromTriangles()
    : d_positions(initData(&d_positions,"positions","If not set by user, find the context mechanical"))
    , d_triangles(initData(&d_triangles,"triangles","If not set by user, find the context topology"))
    , d_quads(initData(&d_quads,"quads","If not set by user, find the context topology"))
    , d_volume(initData(&d_volume,Real(0.0),"volume","Relevant if closed surface"))
    , d_doUpdate(initData(&d_doUpdate,false,"update","If true, will update the volume at each time step"))
{
    d_volume.setReadOnly(true);
    setDirtyValue();
}


template <class DataTypes>
VolumeFromTriangles<DataTypes>::~VolumeFromTriangles()
{
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::init()
{
    m_componentstate = ComponentState::Invalid;

    addInput(&d_positions);
    addInput(&d_triangles);
    addInput(&d_quads);

    addOutput(&d_volume);

    if(!d_positions.isSet())
    {
        m_state = dynamic_cast<MechanicalState*>(getContext()->getMechanicalState());

        if(m_state == nullptr)
        {
            msg_error() << "No positions given by the user and no mechanical state found in the context. The component is deactivated.";
            return;
        }

        ReadAccessor<Data<VecCoord> > positions = m_state->read(ConstVecCoordId::position());
        d_positions.setValue(positions.ref());
    }

    initTopology();
    checkTopology();
    updateVolume();

    m_componentstate = ComponentState::Valid;
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::reinit()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    updateVolume();
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::initTopology()
{
    m_topology = getContext()->getMeshTopology();

    if(!d_triangles.isSet() && m_topology)
        d_triangles.setValue(m_topology->getTriangles());

    if(!d_quads.isSet() && m_topology)
        d_quads.setValue(m_topology->getQuads());

    if(!d_quads.isSet() && !d_triangles.isSet() && !m_topology)
        msg_warning() << "No quads or triangles given by the user and no topology context";
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::checkTopology()
{
    ReadAccessor<Data<VecCoord> >       positions = d_positions;
    ReadAccessor<Data<VecTriangles> >   triangles = d_triangles;
    ReadAccessor<Data<VecQuads> >       quads     = d_quads;

    /// Check that the triangles datafield does not contains indices that would crash the
    /// component.
    int nbTriangles = triangles.size() ;
    for(int i=0;i<nbTriangles;i++){
        for(int j=0;j<3;j++){
            if( triangles[i][j] < 0 )
                msg_error() << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                              <<". is too small regarding positions size of(" << positions.size() << ")" ;

            if( triangles[i][j] >= positions.size() )
                msg_error() << "triangles[" << i << "]["<< j << "]="<< triangles[i][j]
                              <<". is too large regarding positions size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the quads datafield does not contains indices that would crash the
    /// component.
    int nbQuads = quads.size() ;
    for(int i=0;i<nbQuads;i++){
        for(int j=0;j<4;j++){
            if( quads[i][j] < 0 )
                msg_error() << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                              << " is too small regarding positions size of("
                              << positions.size() << ")" ;

            if( quads[i][j] >= positions.size() )
                msg_error() << "quads [" <<i << "][" << j << "]=" << quads[i][j]
                              << " is too large regarding positions size of("
                              << positions.size() << ")" ;
        }
    }
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::update()
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if(m_state && d_doUpdate.getValue())
    {
        ReadAccessor<Data<VecCoord> > positions = m_state->read(ConstVecCoordId::position());
        d_positions.setValue(positions.ref());
        cleanDirty();
        updateVolume();
    }
}


template <class DataTypes>
void VolumeFromTriangles<DataTypes>::updateVolume()
{
    Real volume = 0;

    ReadAccessor<Data<VecCoord>>     positions = d_positions;
    ReadAccessor<Data<VecTriangles>> triangles = d_triangles;
    ReadAccessor<Data<VecQuads>>     quads     = d_quads;

    for (unsigned int t=0; t<triangles.size(); t++)
    {
        Coord p0, p1, p2;

        p0 = positions[triangles[t][0]];
        p1 = positions[triangles[t][1]];
        p2 = positions[triangles[t][2]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;
    }

    for (unsigned int q=0; q<quads.size(); q++)
    {
        Coord p0, p1, p2;

        p0 = positions[quads[q][0]];
        p1 = positions[quads[q][1]];
        p2 = positions[quads[q][2]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;

        p0 = positions[quads[q][0]];
        p1 = positions[quads[q][2]];
        p2 = positions[quads[q][3]];

        volume += ((p1[1]-p0[1])*(p2[2]-p0[2])-(p2[1]-p0[1])*(p1[2]-p0[2]))*(p0[0]+p1[0]+p2[0])/6;
    }

    if(volume<0) volume = -volume;
    d_volume.setValue(volume);
}


template<class DataTypes>
void VolumeFromTriangles<DataTypes>::handleEvent(Event *event)
{
    if(m_componentstate != ComponentState::Valid)
            return ;

    if (AnimateBeginEvent::checkEventType(event))
    {
        setDirtyValue();
        update();
    }
}


} // namespace engine

} // namespace component

} // namespace sofa

#endif
