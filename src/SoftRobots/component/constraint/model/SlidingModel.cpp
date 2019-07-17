/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Modules                               *
*                                                                             *
* This component is not open-source                                           *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/

#include <sofa/defaulttype/Vec3Types.h>
#include <sofa/defaulttype/RigidTypes.h>

#include "SlidingModel.inl"

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::defaulttype::Rigid3Types;

template<>
void SlidingModel<Rigid3Types>::initDatas()
{
	if (!d_direction.isSet())
	{
		msg_warning() << "No direction of actuation provided by user. Default (1. 0. 0. 0. 0. 0.)";
		Deriv x;
		x[0] = 1;
		d_direction.setValue(x);
	}
	else
	{
		Deriv direction = d_direction.getValue();
		Vec<3, double> center = direction.getVCenter();
		Vec<3, double> orientation = direction.getVOrientation();
		center.normalize();
		orientation.normalize();
		direction.getVCenter() = center;
		direction.getVOrientation() = orientation;

		d_direction.setValue(direction);
	}

	d_displacement.setValue(0.0);
	d_force.setValue(0.0);
}



// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
template class SOFA_SOFTROBOTS_API SlidingModel<Vec3Types>;
template class SOFA_SOFTROBOTS_API SlidingModel<Rigid3Types>;

} // namespace constraintset

} // namespace component

} // namespace sofa

