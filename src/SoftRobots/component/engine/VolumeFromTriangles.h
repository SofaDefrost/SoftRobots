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
#ifndef SOFA_COMPONENT_ENGINE_VOLUMEFROMTRIANGLES_H
#define SOFA_COMPONENT_ENGINE_VOLUMEFROMTRIANGLES_H

#include <sofa/core/DataEngine.h>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>

#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class computes the volume of a given closed surfacic mesh.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template <class DataTypes>
class SOFA_SOFTROBOTS_API VolumeFromTriangles : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(VolumeFromTriangles,DataTypes),core::DataEngine);

    typedef typename DataTypes::VecCoord VecCoord;

    typedef typename DataTypes::Coord  Coord;
    typedef typename Coord::value_type Real;

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename core::topology::BaseMeshTopology           BaseMeshTopology;

    typedef core::topology::BaseMeshTopology::Triangle      Triangle;
    typedef core::topology::BaseMeshTopology::Quad          Quad;

    typedef core::topology::BaseMeshTopology::SeqTriangles      VecTriangles;
    typedef core::topology::BaseMeshTopology::SeqQuads          VecQuads;

public:

    VolumeFromTriangles();
    ~VolumeFromTriangles() override;


    ////////////////////////// Inherited from BaseObject ///////////////////
    void init() override;
    void reinit() override;
    ////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from DataEngine////////////////////
    void doUpdate() override;
    ///////////////////////////////////////////////////////////////////////

    SReal getVolume() {return d_volume.getValue();}

protected:

    MechanicalState*   m_state;
    BaseMeshTopology*  m_topology;

    Data<VecCoord>     d_positions;
    Data<VecTriangles> d_triangles;
    Data<VecQuads>     d_quads;

    Data<Real>         d_volume;
    Data<bool>         d_doUpdate;

    void updateVolume();

private:

    void initTopology();
    void checkTopology();

};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
extern template class SOFA_SOFTROBOTS_API VolumeFromTriangles<sofa::defaulttype::Vec3Types>;


} // namespace engine

} // namespace component

} // namespace sofa

#endif
