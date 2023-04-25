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
#define SOFA_COMPONENT_ENGINE_DATAVARIATIONLIMITER_CPP

#include <SoftRobots/component/controller/DataVariationLimiter.inl>
#include <sofa/core/ObjectFactory.h>
#include <sofa/defaulttype/VecTypes.h>

namespace softrobots::controller
{

using namespace sofa::defaulttype;
using namespace sofa::type;
using sofa::core::RegisterObject;

int DataVariationLimiterClass = RegisterObject("This component interpolates between two consecutive inputs when a jump is detected.")
        .add< DataVariationLimiter<Vec3Types::Coord> >(true)
        .add< DataVariationLimiter<Vec2Types::Coord> >()
        .add< DataVariationLimiter<Vec1Types::Coord> >()

        .add< DataVariationLimiter<sofa::type::Vec1i> >()
        .add< DataVariationLimiter<sofa::type::Vec2i> >()
        .add< DataVariationLimiter<sofa::type::Vec3i> >()
        ;

template class SOFA_SOFTROBOTS_API DataVariationLimiter<Vec3Types::Coord>;
template class SOFA_SOFTROBOTS_API DataVariationLimiter<Vec2Types::Coord>;
template class SOFA_SOFTROBOTS_API DataVariationLimiter<Vec1Types::Coord>;

template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec1i>;
template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec2i>;
template class SOFA_SOFTROBOTS_API DataVariationLimiter<sofa::type::Vec3i>;

} // namespace
