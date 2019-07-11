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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL
#define SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL

#include <sofa/core/visual/VisualParams.h>
#include <sofa/defaulttype/Vec.h>

#include "SlidingModel.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::visual::VisualParams;
using sofa::defaulttype::BaseVector;
using sofa::helper::ReadAccessor;
using sofa::helper::rabs;
using sofa::defaulttype::Vec4f;
using sofa::defaulttype::Vec;

template<class DataTypes>
SlidingModel<DataTypes>::SlidingModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
	, d_maxPositiveDisplacement(initData(&d_maxPositiveDisplacement, Real(0.0), "maxPositiveDisp",
		"Maximum displacement of the actuator in the given direction. \n"
		"If unspecified no maximum value will be considered."))

	, d_maxNegativeDisplacement(initData(&d_maxNegativeDisplacement, Real(0.0), "maxNegativeDisp",
		"Maximum displacement of the actuator in the negative direction. \n"
		"If unspecified no maximum value will be considered."))

	, d_maxDispVariation(initData(&d_maxDispVariation, Real(0.0), "maxDispVariation",
		"Maximum variation of the displacement allowed. If not set, no max variation will be concidered."))

	, d_maxForce(initData(&d_maxForce, Real(0.0), "maxForce",
		"Maximum force of the actuator. \n"
		"If unspecified no maximum value will be considered."))

	, d_minForce(initData(&d_minForce, Real(0.0), "minForce",
		"Minimum force of the actuator. \n"
		"If unspecified no minimum value will be considered."))

    , d_direction(initData(&d_direction, "direction",
                          "Direction of the actuation."))

    , d_indices(initData(&d_indices, "indices",
                          "Indices of the nodes subjected to the force. \n"
                          "If no indices given, mechanical context considered."))

	, d_initProjectedDisplacement(initData(&d_initProjectedDisplacement, "initialProjectedDisplacement",
									"Displacement in the direction at the rest position (cannot be changed by user)."))

	, d_projectedDisplacement(initData(&d_projectedDisplacement, "projectedDisplacement",
									"Displacement in the direction at the current position."))
	
    , d_force(initData(&d_force,(double)0.0, "force",
                          "Output force."))

    , d_displacement(initData(&d_displacement,(double)0.0, "displacement",
                          "Output displacement compared to the initial position."))

    , d_showDirection(initData(&d_showDirection,false, "showDirection",
                          "Draw the direction."))

    , d_showVisuScale(initData(&d_showVisuScale,(double)0.001, "showVisuScale",
                          "Visualization scale."))

{
	d_maxNegativeDisplacement.setGroup("Input");
	d_maxPositiveDisplacement.setGroup("Input");
	d_maxDispVariation.setGroup("Input");
	d_maxForce.setGroup("Input");
	d_minForce.setGroup("Input");

	d_initProjectedDisplacement.setReadOnly(true);
	d_projectedDisplacement.setReadOnly(true);

	d_direction.setGroup("Input");
	d_indices.setGroup("Input");

	d_force.setGroup("Output");
	d_displacement.setGroup("Output");
	d_force.setReadOnly(true);
	d_displacement.setReadOnly(true);

	d_showDirection.setGroup("Visualization");
	d_showVisuScale.setGroup("Visualization");
}


template<class DataTypes>
SlidingModel<DataTypes>::~SlidingModel()
{
}

template<class DataTypes>
void SlidingModel<DataTypes>::init()
{
	m_componentstate = ComponentState::Invalid;
	SoftRobotsConstraint<DataTypes>::init();

	if (m_state == nullptr)
	{
		msg_error(this) << "There is no mechanical state associated with this node. "
			"the object is deactivated. "
			"To remove this error message fix your scene possibly by "
			"adding a MechanicalObject.";
		return;
	}

	if (!d_indices.isSet())
	{
		SetIndexArray &list = (*d_indices.beginEdit());
		msg_warning(this) << "No index of actuation given, set default (all points of context MechanicalState).";

		for (unsigned int i = 0; i<m_state->getSize(); i++)
			list.push_back(i);
		d_indices.endEdit();
	}
	else
		checkIndicesRegardingState();

	initDatas();

	m_componentstate = ComponentState::Valid;
}

template<class DataTypes>
void SlidingModel<DataTypes>::bwdInit()
{
	if (m_componentstate != ComponentState::Valid)
		return;
	ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
	ReadAccessor<Data<VecCoord> > restPositions = m_state->readRestPositions();
	Real projectedDisplacement = getProjectedDisplacement(positions.ref(), restPositions.ref());
	Real initProjectedDisplacement = getProjectedDisplacement(restPositions.ref(), restPositions.ref());
	d_initProjectedDisplacement.setValue(initProjectedDisplacement);
	d_projectedDisplacement.setValue(projectedDisplacement);
	
}

template<class DataTypes>
void SlidingModel<DataTypes>::reinit()
{
	if (m_componentstate != ComponentState::Valid)
		return;

	checkIndicesRegardingState();
	initDatas();
}

template<class DataTypes>
void SlidingModel<DataTypes>::reset()
{
    reinit();
}


template<class DataTypes>
void SlidingModel<DataTypes>::initDatas()
{
    if (!d_direction.isSet())
    {
        msg_warning()<<"No direction of actuation provided by user. Default (1. 0. 0.)";
        Deriv x;
        x[0]=1;
        d_direction.setValue(x);
    }
    else
    {
        Deriv direction = d_direction.getValue();
        direction.normalize();
        d_direction.setValue(direction);
    }

    d_displacement.setValue(0.0);
    d_force.setValue(0.0);
}


template<class DataTypes>
void SlidingModel<DataTypes>::checkIndicesRegardingState()
{
	ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();

	for (unsigned int i = 0; i<d_indices.getValue().size(); i++)
	{
		if (positions.size() <= d_indices.getValue()[i])
			msg_error(this) << "Indices at index " << i << " is too large regarding mechanicalState [position] size";
		if (d_indices.getValue()[i] < 0)
			msg_error(this) << "Indices at index " << i << " is negative";
	}
}

template<class DataTypes>
SReal SlidingModel<DataTypes>::getProjectedDisplacement(const VecCoord &positions, const VecCoord &restPositions)
{
	const SetIndexArray &indices = d_indices.getValue();
	const Deriv         &direction = d_direction.getValue();

	// Projection of the global displacement along the direction (normalized) of the actuation
	Deriv d = DataTypes::coordDifference(positions[indices[0]], restPositions[indices[0]]);
	const int size = Deriv::total_size;

	Real projection = 0.;
	for (unsigned int i = 0; i<size; i++)
		projection += d[i]*direction[i];

	return projection;
}

template<class DataTypes>
void SlidingModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
{
	SOFA_UNUSED(cParams);
	SOFA_UNUSED(x);

	if (m_componentstate != ComponentState::Valid)
		return;

	m_constraintId = cIndex;

	MatrixDeriv& matrix = *cMatrix.beginEdit();

	MatrixDerivRowIterator rowIterator = matrix.writeLine(m_constraintId);

	Deriv direction = d_direction.getValue();

	ReadAccessor<Data<SetIndexArray>> indices = d_indices;

	for (unsigned int i = 0; i<indices.size(); i++)
		rowIterator.setCol(indices[i], direction / indices.size());

	cIndex++;
	cMatrix.endEdit();
	m_nbLines = cIndex - m_constraintId;
}

template<class DataTypes>
void SlidingModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
	BaseVector *resV,
	const BaseVector *Jdx)
{
	SOFA_UNUSED(cParams);

	if (m_componentstate != ComponentState::Valid)
		return;

	ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
	ReadAccessor<Data<VecCoord> > restPositions = m_state->readRestPositions();
	Real projection = getProjectedDisplacement(positions.ref(), restPositions.ref());
	d_projectedDisplacement.setValue(projection);
	
	const int size = Deriv::total_size;
	const SetIndexArray &indices = d_indices.getValue();
	const Deriv         &direction = d_direction.getValue();
	for (unsigned int i = 0; i<size; i++)
		projection += Jdx->element(i)*direction[i];
	
	if (indices.size())
		resV->set(m_constraintId, projection);
}


/*template<class DataTypes>
void SlidingModel<DataTypes>::storeResults(helper::vector<double> &lambda,
	helper::vector<double> &delta)
{
	if (m_componentstate != ComponentState::Valid)
		return;

	d_force.setValue(lambda[0]);
	d_displacement.setValue(delta[0]); // MISK: check this -> should recompute?, should this be storeLambda instead? who calls storeresults and how do they compute delta

}*/
template<class DataTypes>
void SlidingModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
	core::MultiVecDerivId res,
	const sofa::defaulttype::BaseVector* lambda)
{
	SOFA_UNUSED(res);
	SOFA_UNUSED(cParams);
	if (m_componentstate != ComponentState::Valid)
		return;

	d_force.setValue(lambda->element(m_constraintId));
	d_displacement.setValue(d_projectedDisplacement.getValue() - d_initProjectedDisplacement.getValue());
	//d_displacement.setValue(delta[0]); // MISK: check this -> should recompute?, should this be storeLambda instead? who calls storeresults and how do they compute delta

}

template<class DataTypes>
void SlidingModel<DataTypes>::draw(const VisualParams* vparams)
{
	if (m_componentstate != ComponentState::Valid)
		return;

	if (!vparams->displayFlags().getShowInteractionForceFields()) return;
	if (!d_showDirection.getValue()) return;

	ReadAccessor<Data<VecCoord> > positions = m_state->readPositions();
	const SetIndexArray &indices = d_indices.getValue();

	Vec<3, SReal> bary(0., 0., 0.);
	for (unsigned int i = 0; i<indices.size(); i++)
		for (unsigned int j = 0; j<3; j++)
			bary[j] += positions[indices[i]][j] / indices.size();

	Vec<3, SReal> baryArrow(0., 0., 0.);
	for (unsigned int j = 0; j<3; j++)
		baryArrow[j] = bary[j] + d_direction.getValue()[j] * d_showVisuScale.getValue();

	vparams->drawTool()->setLightingEnabled(true);
	vparams->drawTool()->drawArrow(bary, baryArrow, d_showVisuScale.getValue() / 20.0f, Vec4f(1, 0, 0, 1));
	vparams->drawTool()->restoreLastState();
}


} // namespace constraintset

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONSTRAINTSET_SLIDINGMODEL_INL
