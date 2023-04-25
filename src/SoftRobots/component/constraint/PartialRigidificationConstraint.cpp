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

namespace sofa::component::constraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

//////////////////////// DEFINITIONS //////////////////////////////////////////////////////////////

/////////////////// PartialRigidificationConstraintResolution6Dof /////////////////////////////////
PartialRigidificationConstraintResolution6Dof::PartialRigidificationConstraintResolution6Dof()
    : ConstraintResolution(6)
{
}



/////////////////// PartialRigidificationConstraint //////////////////////////////////////////////


//////////////////////// FACTORY //////////////////////////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int PartialRigidificationConstraintClass = core::RegisterObject("PartialRigidificationConstraint")
        .add< PartialRigidificationConstraint<Rigid3Types> >(true)

        ;
//////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_API PartialRigidificationConstraint<Rigid3Types>;


} // namespace
