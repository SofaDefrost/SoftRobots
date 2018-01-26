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
#ifndef SOFA_COMPONENT_ENGINE_VOLUMEFROMTETRAHEDRONS_H
#define SOFA_COMPONENT_ENGINE_VOLUMEFROMTETRAHEDRONS_H

#include <sofa/core/DataEngine.h>

#include <sofa/core/topology/BaseMeshTopology.h>
#include <sofa/core/behavior/MechanicalState.h>

#include "../initSoftRobots.h"

namespace sofa
{

namespace component
{

namespace engine
{

/**
 * This class returns the volumes of a given volumic mesh.
 * Description can be found at:
 * https://project.inria.fr/softrobot/documentation/engine/VolumeFromTetrahedrons
 */
template <class DataTypes>
class SOFA_SOFTROBOTS_API VolumeFromTetrahedrons : public core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(VolumeFromTetrahedrons,DataTypes),core::DataEngine);

    typedef typename DataTypes::VecCoord VecCoord;

    typedef typename DataTypes::Coord  Coord;
    typedef typename Coord::value_type Real;

    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename core::topology::BaseMeshTopology           BaseMeshTopology;

    typedef core::topology::BaseMeshTopology::Tetra         Tetra;
    typedef core::topology::BaseMeshTopology::Hexa          Hexa;

    typedef core::topology::BaseMeshTopology::SeqTetrahedra     VecTetras;
    typedef core::topology::BaseMeshTopology::SeqHexahedra      VecHexas;

public:

    virtual std::string getTemplateName() const
    {
        return templateName(this);
    }

    static std::string templateName(const VolumeFromTetrahedrons<DataTypes>* = nullptr)
    {
        return DataTypes::Name();
    }

    VolumeFromTetrahedrons();

    virtual ~VolumeFromTetrahedrons();

    ////////////////////////// Inherited from BaseObject ///////////////////
    virtual void init() override;
    virtual void reinit() override;
    ////////////////////////////////////////////////////////////////////////


    ////////////////////////// Inherited from DDGNode /////////////////////
    virtual void update() override;
    virtual void handleEvent(core::objectmodel::Event *event) override;
    ///////////////////////////////////////////////////////////////////////

    SReal getVolume() {return d_volume.getValue();}

protected:

    MechanicalState*   m_state;
    BaseMeshTopology*  m_topology;

    Data<VecCoord>     d_positions;
    Data<VecTetras>    d_tetras;
    Data<VecHexas>     d_hexas;

    Data<Real>         d_volume;
    Data<bool>         d_doUpdate;

    void updateVolume();

private:

    void initTopology();
    void checkTopology();

    SReal getElementVolume(const Tetra& tetra);
    SReal getElementVolume(const Hexa& hexa);
};

// Declares template as extern to avoid the code generation of the template for
// each compilation unit. see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_EXTERN_TEMPLATE 
#ifdef SOFA_WITH_DOUBLE
extern template class SOFA_SOFTROBOTS_API VolumeFromTetrahedrons<sofa::defaulttype::Vec3dTypes>;
#endif

#ifdef SOFA_WITH_DOUBLE
extern template class SOFA_SOFTROBOTS_API VolumeFromTetrahedrons<sofa::defaulttype::Vec3dTypes>;
#endif
#endif

} // namespace engine

} // namespace component

} // namespace sofa

#endif
