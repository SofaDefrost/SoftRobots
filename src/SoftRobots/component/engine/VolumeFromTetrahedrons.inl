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
#pragma once

#include <sofa/config.h>
#include <SoftRobots/component/engine/VolumeFromTetrahedrons.h>

namespace sofa
{

namespace component
{

namespace engine
{

using core::objectmodel::ComponentState;
using helper::ReadAccessor;
using type::vector;
using core::ConstVecCoordId;
using core::objectmodel::BaseData;


template <class DataTypes>
VolumeFromTetrahedrons<DataTypes>::VolumeFromTetrahedrons()
    : d_positions(initData(&d_positions,"positions","If not set by user, find the context mechanical."))
    , d_tetras(initData(&d_tetras,"tetras","If not set by user, find the context topology."))
    , d_hexas(initData(&d_hexas,"hexas","If not set by user, find the context topology."))
    , d_volume(initData(&d_volume,Real(0.0),"volume",""))
    , d_doUpdate(initData(&d_doUpdate,false,"update","If true, will update the volume with the current positions."))
{
    d_volume.setReadOnly(true);
}


template <class DataTypes>
VolumeFromTetrahedrons<DataTypes>::~VolumeFromTetrahedrons()
{
}


template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);

    addInput(&d_positions);
    addInput(&d_tetras);
    addInput(&d_hexas);

    addOutput(&d_volume);

    if(!d_positions.isSet())
    {
        m_state = dynamic_cast<MechanicalState*>(getContext()->getMechanicalState());

        if(m_state == nullptr)
        {
            msg_error() << "No positions given by the user and no mechanical state found in the context. The component is deactivated.";
            return;
        }

        d_positions.setParent(m_state->findData("position")); // Links d_positions to m_state.position
    }

    initTopology();
    checkTopology();
    updateVolume();

    d_componentState.setValue(ComponentState::Valid);
}



template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    updateVolume();
}


template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::initTopology()
{
    m_topology = getContext()->getMeshTopology();

    if(!d_tetras.isSet() && m_topology)
        d_tetras.setValue(m_topology->getTetras());

    if(!d_hexas.isSet() && m_topology)
        d_hexas.setValue(m_topology->getHexas());

    if(!d_hexas.isSet() && !d_tetras.isSet() && !m_topology)
        msg_warning() << "No tetras or hexas given by the user and no topology context. Abort";
}


template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::checkTopology()
{
    ReadAccessor<Data<VecCoord> >  positions = d_positions;
    ReadAccessor<Data<VecTetras> > tetras    = d_tetras;
    ReadAccessor<Data<VecHexas> >  hexas     = d_hexas;

    /// Check that the tetras datafield does not contains indices that would crash the
    /// component.
    int nbTetras = tetras.size() ;
    for(int i=0;i<nbTetras;i++){
        for(int j=0;j<4;j++){
            if( tetras[i][j] >= positions.size() )
                msg_error() << "tetras[" << i << "]["<< j << "]="<< tetras[i][j]
                              <<". is too large regarding positions size of(" << positions.size() << ")" ;
        }
    }

    /// Check that the hexas datafield does not contains indices that would crash the
    /// component.
    int nbHexas = hexas.size() ;
    for(int i=0;i<nbHexas;i++){
        for(int j=0;j<6;j++){
            if( hexas[i][j] >= positions.size() )
                msg_error() << "hexas [" <<i << "][" << j << "]=" << hexas[i][j]
                              << " is too large regarding positions size of("
                              << positions.size() << ")" ;
        }
    }
}



template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::doUpdate()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    if(m_state && d_doUpdate.getValue())
    {
        ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
        d_positions.setValue(positions.ref());
        updateVolume();
    }
}


template <class DataTypes>
void VolumeFromTetrahedrons<DataTypes>::updateVolume()
{
    Real volume = 0.;

    ReadAccessor<Data<VecTetras>> tetras = d_tetras;
    ReadAccessor<Data<VecHexas>>  hexas  = d_hexas;

    for (unsigned int t=0; t<tetras.size(); t++)
        volume += getElementVolume(tetras[t]);

    for (unsigned int t=0; t<hexas.size(); t++)
        volume += getElementVolume(hexas[t]);

    if(volume<0) volume = -volume;
    d_volume.setValue(volume);
}


template <class DataTypes>
SReal VolumeFromTetrahedrons<DataTypes>::getElementVolume(const Tetra& tetra)
{
    ReadAccessor<Data<VecCoord> > positions = d_positions;

    Coord p0 = positions[tetra[0]];
    Coord p1 = positions[tetra[1]];
    Coord p2 = positions[tetra[2]];
    Coord p3 = positions[tetra[3]];

    Real volume = (p0-p1)*cross(p3-p1,p2-p1)/6.;

    return volume;
}


template <class DataTypes>
SReal VolumeFromTetrahedrons<DataTypes>::getElementVolume(const Hexa& hexa)
{
    ReadAccessor<Data<VecCoord> > positions = d_positions;

    Real volume = 0.;
    Coord p0, p1, p2, p3;

    p0 = positions[hexa[6]];
    p1 = positions[hexa[1]];
    p2 = positions[hexa[2]];
    p3 = positions[hexa[3]];

    volume += (p0-p1)*cross(p3-p1,p2-p1)/6.;

    p0 = positions[hexa[4]];
    p1 = positions[hexa[0]];
    p2 = positions[hexa[1]];
    p3 = positions[hexa[3]];

    volume += (p0-p1)*cross(p3-p1,p2-p1)/6.;

    p0 = positions[hexa[5]];
    p1 = positions[hexa[4]];
    p2 = positions[hexa[6]];
    p3 = positions[hexa[1]];

    volume += (p0-p1)*cross(p3-p1,p2-p1)/6.;

    p0 = positions[hexa[7]];
    p1 = positions[hexa[4]];
    p2 = positions[hexa[6]];
    p3 = positions[hexa[3]];

    volume += (p0-p1)*cross(p3-p1,p2-p1)/6.;

    return volume;
}


} // namespace engine

} // namespace component

} // namespace sofa
