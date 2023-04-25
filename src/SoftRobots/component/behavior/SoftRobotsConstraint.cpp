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
#define SOFTROBOTS_SOFTROBOTSCONSTRAINT_CPP

#include <SoftRobots/component/behavior/SoftRobotsConstraint.inl>

namespace softrobots::behavior
{

using namespace sofa::defaulttype;

template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<Vec3Types>;
template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<Vec2Types>;
template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<Vec1Types>;
template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<Rigid3Types>;


} // softrobots::behavior
