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
#define SOFTROBOTS_JOINTMODELCONSTRAINT_CPP

#include <SoftRobots/component/constraint/JointModelConstraint.inl>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/ObjectFactory.h>

namespace softrobots::constraint
{

using namespace sofa::defaulttype;
using namespace sofa::core::objectmodel;

/////////////////////////////////////////// FACTORY //////////////////////////////////////////////
///
/// Register the component to the ObjectFactory
///
int JointModelConstraintClass = sofa::core::RegisterObject("Joint constraint based on JointModel for direct simulation")
        .add< JointModelConstraint<Vec3Types> >(true)
        .add< JointModelConstraint<Vec2Types> >()
        .add< JointModelConstraint<Rigid3Types> >();

/////////////////////////////////////////// TEMPLATE INSTANTIATION ///////////////////////////////
///
/// Instanciate the templates
///
template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Vec3Types>;
template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Vec2Types>;
template class SOFA_SOFTROBOTS_API JointModelConstraint<sofa::defaulttype::Rigid3Types>;

} // namespace softrobots::constraint