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
#include "MappedMatrixForceField.inl"
#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace interactionforcefield
{

using namespace sofa::defaulttype;


////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-SOFA_DECL_CLASS(componentName) : Set the class name of the component
// 2-RegisterObject("description") + .add<> : Register the component
// 3-.add<>(true) : Set default template
SOFA_DECL_CLASS(MappedMatrixForceField)

int MappedMatrixForceFieldClass = core::RegisterObject("Partially rigidify a mechanical object using a rigid mapping.")
#ifdef SOFA_WITH_FLOAT
        .add< MappedMatrixForceField<Vec3fTypes, Rigid3fTypes> >()
        .add< MappedMatrixForceField<Vec3fTypes, Vec3fTypes> >()
        .add< MappedMatrixForceField<Vec1fTypes, Rigid3fTypes> >()
        .add< MappedMatrixForceField<Vec1fTypes, Vec1fTypes> >()
#endif
#ifdef SOFA_WITH_DOUBLE
        .add< MappedMatrixForceField<Vec3dTypes, Rigid3dTypes> >(true)
        .add< MappedMatrixForceField<Vec3dTypes, Vec3dTypes> >(true)
        .add< MappedMatrixForceField<Vec1dTypes, Rigid3dTypes> >(true)
        .add< MappedMatrixForceField<Vec1dTypes, Vec1dTypes> >(true)
#endif
        ;
////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class MappedMatrixForceField<Vec3dTypes, Rigid3dTypes>;
template class MappedMatrixForceField<Vec3dTypes, Vec3dTypes>;
template class MappedMatrixForceField<Vec1dTypes, Rigid3dTypes>;
template class MappedMatrixForceField<Vec1dTypes, Vec1dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
template class MappedMatrixForceField<Vec3fTypes, Rigid3fTypes>;
template class MappedMatrixForceField<Vec3fTypes, Vec3fTypes>;
template class MappedMatrixForceField<Vec1fTypes, Rigid3fTypes>;
template class MappedMatrixForceField<Vec1fTypes, Vec1fTypes>;
#endif

} // namespace forcefield

} // namespace component

} // namespace sofa
