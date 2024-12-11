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
#define SOFTROBOTS_PARTIALRIGIDIFICATIONCONSTRAINT_CPP
#include <SoftRobots/component/constraint/PartialRigidificationConstraint.inl>
#include <sofa/core/ObjectFactory.h>

namespace softrobots::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

PartialRigidificationConstraintResolution6Dof::PartialRigidificationConstraintResolution6Dof()
    : ConstraintResolution(6)
{
}

void registerPartialRigidificationConstraint(sofa::core::ObjectFactory* factory)
{
    factory->registerObjects(sofa::core::ObjectRegistrationData("PartialRigidificationConstraint")
    .add< PartialRigidificationConstraint<Rigid3Types> >(true));
}

template class SOFA_SOFTROBOTS_API PartialRigidificationConstraint<Rigid3Types>;


} // namespace
