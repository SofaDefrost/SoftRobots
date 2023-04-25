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

#include <sofa/core/DataEngine.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/behavior/Mass.h>

#include <SoftRobots/component/initSoftRobots.h>

namespace softrobots::engine
{

/**
 * This class computes the volume of a given closed surfacic mesh.
 * Description can be found at:
 * https://softrobotscomponents.readthedocs.io
 */
template <class DataTypes>
class SOFA_SOFTROBOTS_API CenterOfMass : public sofa::core::DataEngine
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(CenterOfMass,DataTypes), sofa::core::DataEngine);

    typedef typename DataTypes::VecCoord VecCoord;

    typedef typename DataTypes::Coord  Coord;
    typedef typename Coord::value_type Real;

    typedef typename sofa::core::behavior::MechanicalState<DataTypes> MechanicalState;
    typedef typename sofa::core::behavior::Mass<DataTypes>            Mass;

public:

    CenterOfMass();
    ~CenterOfMass() override;


    ////////////////////////// Inherited from BaseObject ///////////////////
    void init() override;
    void reinit() override;
    void draw(const sofa::core::visual::VisualParams* vparams) override;
    ////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from DataEngine////////////////////
    void doUpdate() override;
    ///////////////////////////////////////////////////////////////////////

    Coord getCenterOfMass() {return d_centerOfMass.getValue();}

protected:

    MechanicalState* m_state;
    Mass* m_mass;

    sofa::Data<VecCoord>d_positions;
    sofa::Data<Coord>   d_centerOfMass;
    sofa::Data<bool>    d_visualization;
    sofa::Data<float>   d_visuSize;
    sofa::Data<sofa::type::RGBAColor>   d_visuColor;

    void computeCenterOfMass();

};

extern template class SOFA_SOFTROBOTS_API CenterOfMass<sofa::defaulttype::Vec3Types>;

} // namespace

namespace sofa::component::engine
{
    template <class DataTypes>
    using CenterOfMass SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::engine::CenterOfMass<DataTypes>;
}

