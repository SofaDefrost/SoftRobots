/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2017 INRIA, USTL, UJF, CNRS, MGH                    *
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
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_FORCEFIELD_AFFINERESTSHAPESPRINGFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_AFFINERESTSHAPESPRINGFORCEFIELD_INL

#include "AffineRestShapeSpringForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/system/config.h>
#include <sofa/defaulttype/VecTypes.h>
#include <sofa/defaulttype/RigidTypes.h>

#include <sofa/defaulttype/RGBAColor.h>

#include <assert.h>
#include <iostream>


namespace sofa
{

namespace component
{

namespace forcefield
{

using helper::WriteAccessor;
using helper::ReadAccessor;
using core::behavior::BaseMechanicalState;
using core::behavior::MultiMatrixAccessor;
using core::behavior::ForceField;
using defaulttype::BaseMatrix;
using core::VecCoordId;
using core::MechanicalParams;
using defaulttype::Vector3;
using defaulttype::Vec4f;
using helper::vector;
using core::visual::VisualParams;

template<class DataTypes>
AffineRestShapeSpringForceField<DataTypes>::AffineRestShapeSpringForceField()
	: points(initData(&points, "points", "points controlled by the rest shape springs"))
	, stiffness(initData(&stiffness, "stiffness", "stiffness values between the actual position and the rest shape position"))
	, angularStiffness(initData(&angularStiffness, "angularStiffness", "angularStiffness assigned when controlling the rotation of the points"))
	, external_points(initData(&external_points, "external_points", "points from the external Mechancial State that define the rest shape springs"))
	, biasForce(initData(&biasForce, "biasForce", "constant bias force applied to the specified degrees of freedom"))
	, biasTorque(initData(&biasTorque, "biasTorque", "constant bias torque applied to the specified degrees of freedom"))
	, recompute_indices(initData(&recompute_indices, true, "recompute_indices", "Recompute indices (should be false for BBOX)"))
	, restMState(initLink("external_rest_shape", "rest_shape can be defined by the position of an external Mechanical State"))
{
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::parse(core::objectmodel::BaseObjectDescription *arg)
{
	const char* attr = arg->getAttribute("external_rest_shape");
	if (attr != nullptr && attr[0] != '@')
	{
		msg_error() << "AffineRestShapeSpringForceField have changed since 17.06. The parameter 'external_rest_shape' is now a Link. To fix your scene you need to add and '@' in front of the provided path. See PR#315";
	}

	Inherit::parse(arg);
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::bwdInit()
{
	ForceField<DataTypes>::init();

	if (stiffness.getValue().empty())
	{
		msg_info() << "No stiffness is defined, assuming equal stiffness on each node, k = 100.0 ";

		VecReal stiffs;
		stiffs.push_back(100.0);
		stiffness.setValue(stiffs);
	}

	if (restMState.get() == NULL)
	{
		useRestMState = false;

		if (!restMState.empty())
			msg_warning() << "external_rest_shape in node " << this->getContext()->getName() << " not found";
	}
	else
	{
		useRestMState = true;
	}

	k = stiffness.getValue();

	recomputeIndices();

	BaseMechanicalState* state = this->getContext()->getMechanicalState();
	assert(state);
	matS.resize(state->getMatrixSize(), state->getMatrixSize());
	lastUpdatedStep = -1.0;
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::reinit()
{
	if (stiffness.getValue().empty())
	{
		msg_info() << "No stiffness is defined, assuming equal stiffness on each node, k = 100.0 ";

		VecReal stiffs;
		stiffs.push_back(100.0);
		stiffness.setValue(stiffs);
	}

	k = stiffness.getValue();
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::recomputeIndices()
{
	m_indices.clear();
	m_ext_indices.clear();

	for (unsigned int i = 0; i < points.getValue().size(); i++)
		m_indices.push_back(points.getValue()[i]);

	for (unsigned int i = 0; i < external_points.getValue().size(); i++)
		m_ext_indices.push_back(external_points.getValue()[i]);

	if (m_indices.size() == 0)
	{
		for (unsigned int i = 0; i < (unsigned)this->mstate->getSize(); i++)
		{
			m_indices.push_back(i);
		}
	}

	if (m_ext_indices.size() == 0)
	{
		if (useRestMState)
		{
			for (unsigned int i = 0; i < (unsigned)restMState->getSize(); i++)
			{
				m_ext_indices.push_back(i);
			}
		}
		else
		{
			for (unsigned int i = 0; i < (unsigned)this->mstate->getSize(); i++)
			{
				m_ext_indices.push_back(i);
			}
		}
	}

	if (m_indices.size() > m_ext_indices.size())
	{
		msg_error() << "The dimention of the source and the targeted points are different ";
		m_indices.clear();
	}
}

template<class DataTypes>
const typename AffineRestShapeSpringForceField<DataTypes>::DataVecCoord* AffineRestShapeSpringForceField<DataTypes>::getExtPosition() const
{
	return (useRestMState ? restMState->read(VecCoordId::position()) : this->mstate->read(VecCoordId::restPosition()));
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::addForce(const core::MechanicalParams* /* mparams */, DataVecDeriv& f, const DataVecCoord& x, const DataVecDeriv& /* v */)
{
	sofa::helper::WriteAccessor< DataVecDeriv > f1 = f;
	sofa::helper::ReadAccessor< DataVecCoord > p1 = x;

	sofa::helper::ReadAccessor< DataVecCoord > p0 = *getExtPosition();

	f1.resize(p1.size());

	if (recompute_indices.getValue())
	{
		recomputeIndices();
	}

	const VecReal& k = stiffness.getValue();
	const VecReal& k_a = angularStiffness.getValue();
	const Vec3& biasF = biasForce.getValue();
	const Vec3& biasT = biasTorque.getValue();

	for (unsigned int i = 0; i < m_indices.size(); i++)
	{
		const unsigned int index = m_indices[i];
		unsigned int ext_index = m_indices[i];
		if (useRestMState)
			ext_index = m_ext_indices[i];

		Vec3d dx = p1[index].getCenter() - p0[ext_index].getCenter();
		getVCenter(f1[index]) -= dx * (i < k.size() ? k[i] : k[0]);
		getVCenter(f1[index]) += biasF;

		// rotation
		Quatd dq = p1[index].getOrientation() * p0[ext_index].getOrientation().inverse();
		Vec3d dir;
		double angle = 0;
		dq.normalize();

		if (dq[3] < 0)
		{
			dq = dq * -1.0;
		}

		if (dq[3] < 0.999999999999999)
			dq.quatToAxis(dir, angle);

		getVOrientation(f1[index]) -= dir * angle * (i < k_a.size() ? k_a[i] : k_a[0]);
		getVOrientation(f1[index]) += biasT;
	}
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::addDForce(const core::MechanicalParams* mparams, DataVecDeriv& df, const DataVecDeriv& dx)
{
	sofa::helper::WriteAccessor< DataVecDeriv > df1 = df;
	sofa::helper::ReadAccessor< DataVecDeriv > dx1 = dx;

	const VecReal& k = stiffness.getValue();
	const VecReal& k_a = angularStiffness.getValue();
	Real kFactor = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

	unsigned int curIndex = 0;

	for (unsigned int i = 0; i<m_indices.size(); i++)
	{
		curIndex = m_indices[i];
		getVCenter(df1[curIndex]) -= getVCenter(dx1[curIndex]) * ((i < k.size()) ? k[i] : k[0]) * kFactor;
		getVOrientation(df1[curIndex]) -= getVOrientation(dx1[curIndex]) * (i < k_a.size() ? k_a[i] : k_a[0]) * kFactor;
	}
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::addKToMatrix(const core::MechanicalParams* mparams, const sofa::core::behavior::MultiMatrixAccessor* matrix)
{
	const VecReal& k = stiffness.getValue();
	const VecReal& k_a = angularStiffness.getValue();
	const int N = 6;
	sofa::core::behavior::MultiMatrixAccessor::MatrixRef mref = matrix->getMatrix(this->mstate);
	sofa::defaulttype::BaseMatrix* mat = mref.matrix;
	unsigned int offset = mref.offset;
	Real kFact = (Real)mparams->kFactorIncludingRayleighDamping(this->rayleighStiffness.getValue());

	unsigned int curIndex = 0;

	for (unsigned int index = 0; index < m_indices.size(); index++)
	{
		curIndex = m_indices[index];

		// translation
		for (int i = 0; i < 3; i++)
		{
			mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * (index < k.size() ? k[index] : k[0]));
		}

		// rotation
		for (int i = 3; i < 6; i++)
		{
			mat->add(offset + N * curIndex + i, offset + N * curIndex + i, -kFact * (index < k_a.size() ? k_a[index] : k_a[0]));
		}
	}
}

template<class DataTypes>
void AffineRestShapeSpringForceField<DataTypes>::updateForceMask()
{
	for (unsigned int i = 0; i<m_indices.size(); i++)
		this->mstate->forceMask.insertEntry(m_indices[i]);
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_FORCEFIELD_AFFINERESTSHAPESPRINGFORCEFIELD_INL



