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

#include "UnilateralPlaneConstraint.inl"
#include <sofa/core/ObjectFactory.h>

namespace sofa
{

namespace component
{

namespace constraintset
{

using namespace sofa::defaulttype;
using core::ConstraintParams;

/////////////////// PositionEffectorPassiveConstraintResolution ///////////////////////////////////////
UnilateralPlaneConstraintResolution::UnilateralPlaneConstraintResolution(const unsigned int _nbLines)
: ConstraintResolution(_nbLines)
{
}

void UnilateralPlaneConstraintResolution::init(int line, double**w, double*force)
{
    SOFA_UNUSED(line);
    SOFA_UNUSED(w);
    SOFA_UNUSED(force);
}

void UnilateralPlaneConstraintResolution::resolution(int line, double**w, double*d, double*force, double*dFree)
{
    SOFA_UNUSED(dFree);

    force[line] -= d[line] / w[line][line];
    if(force[line] < 0)
        force[line] = 0.0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////    FACTORY    //////////////////////////////////////////////
using namespace sofa::helper;
// Registering the component
// see: http://wiki.sofa-framework.org/wiki/ObjectFactory
// 1-RegisterObject("description") + .add<> : Register the component
// 2-.add<>(true) : Set default template

int UnilateralPlaneConstraintClass = core::RegisterObject("This component is a simple point plane collision model. \n"
                                                          "By providing 4 points to the component, the first point will \n"
                                                          "be constrained to stay in one side of the plane described \n"
                                                          "by the three other points (in the direction of the plane normal). \n"
                                                          "All the four points, the triangle and the normal can be \n"
                                                          "seen by allowing the 'Collision Model' in the 'View' tab.")
                .add< UnilateralPlaneConstraint<Vec3Types> >(true)
        
        ;

////////////////////////////////////////////////////////////////////////////////////////////////////////

// Force template specialization for the most common sofa floating point related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_API UnilateralPlaneConstraint<sofa::defaulttype::Vec3Types>;


} // namespace constraintset

} // namespace component

} // namespace sofa
