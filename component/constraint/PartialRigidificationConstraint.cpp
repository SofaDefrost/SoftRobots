/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
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
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/

#include "PartialRigidificationConstraint.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;

//////////////////////// DEFINITIONS //////////////////////////////////////////////////////////////

/////////////////// PartialRigidificationConstraintResolution6Dof /////////////////////////////////
PartialRigidificationConstraintResolution6Dof::PartialRigidificationConstraintResolution6Dof()
    : ConstraintResolution(6)
{
}

void PartialRigidificationConstraintResolution6Dof::init(int line, double** w, double *force)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(force);
}

void PartialRigidificationConstraintResolution6Dof::initForce(int line, double* force)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(force);
}

void PartialRigidificationConstraintResolution6Dof::resolution(int line, double** w,
                                                               double* d, double* dfree)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(d);
    SOFA_UNUSED(dfree);
}

void PartialRigidificationConstraintResolution6Dof::store(int line, double* force, bool convergence)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(force);
    SOFA_UNUSED(convergence);
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
template class PartialRigidificationConstraint<Rigid3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa
