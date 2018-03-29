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
#include <SofaBaseMechanics/MechanicalObject.h>

#include "SlidingModel.inl"

namespace sofa
{

namespace component
{

namespace constraintset
{

using namespace sofa::defaulttype;
using namespace sofa::helper;
using namespace sofa::core;

// specialisation for rigid ?
#ifdef SOFA_WITH_DOUBLE
template<>
void SlidingModel<Rigid3dTypes>::initDatas()
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
		Vec<3, double> Ntrans = direction.getVCenter();
		Vec<3, double> Nrot = direction.getVOrientation();
		Ntrans.normalize();
		Nrot.normalize();
		direction.getVCenter() = Ntrans;
		direction.getVOrientation() = Nrot;

		d_direction.setValue(direction);
	}

	d_displacement.setValue(0.0);
	d_force.setValue(0.0);
}


template<>
void SlidingModel<Rigid3dTypes>::computeViolation(Deriv &result, const Coord &PosRef, const Coord &PosIn)
{

	result.getVCenter() = PosIn.getCenter() - PosRef.getCenter();
	Quater<double> q, qref, qin;
	qin = PosIn.getOrientation();
	qref = PosRef.getOrientation();
	q = qin.inverse()*qref;
	result.getVOrientation() = q.quatToRotationVector();



}
#endif




// Force template specialization for the most common sofa type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates
#ifdef SOFA_WITH_DOUBLE
template class SOFA_SOFTROBOTS_API SlidingModel<Vec3dTypes>;
template class SOFA_SOFTROBOTS_API SlidingModel<Rigid3dTypes>;
#endif
#ifdef SOFA_WITH_FLOAT
template class SOFA_SOFTROBOTS_API SlidingModel<Vec3fTypes>;
#endif

} // namespace constraintset

} // namespace component

} // namespace sofa
