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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_AFFINEFUNCTIONMODEL_INL
#define SOFA_COMPONENT_CONSTRAINTSET_AFFINEFUNCTIONMODEL_INL


#include "AffineFunctionModel.h"

namespace sofa
{

namespace component
{

namespace constraintset
{

using sofa::core::objectmodel::ComponentState;
using sofa::core::visual::VisualParams;
using sofa::linearalgebra::BaseVector;
using sofa::helper::ReadAccessor;

template<class DataTypes>
AffineFunctionModel<DataTypes>::AffineFunctionModel(MechanicalState* object)
    : SoftRobotsConstraint<DataTypes>(object)
    , d_indices(initData(&d_indices, "indices",
                         "List of points included in the affine function. \n"
                         "If no indices are given, default value is 0. \n" ))

	, d_coefficients(initData(&d_coefficients,"coefficients","Coefficients for each dof for each index in indices."))

	, d_offset(initData(&d_offset,Real(0.0),"offset", "Constant value of the affine function."))

	, d_initFunctionValue(initData(&d_initFunctionValue, Real(0.0),"initialFunctionValue", "Set to be the function value in the rest positions."))

    , d_functionValue(initData(&d_functionValue, Real(0.0), "functionValue","Function value."))

    , d_force(initData(&d_force,double(0.0), "force", "Output force."))

    , d_displacement(initData(&d_displacement,double(0.0), "displacement",
                          "Output function value compared to the initial function value."))

{
    setUpData();
}

template<class DataTypes>
AffineFunctionModel<DataTypes>::~AffineFunctionModel()
{
}

template<class DataTypes>
void AffineFunctionModel<DataTypes>::setUpData()
{
    d_initFunctionValue.setReadOnly(true);

    d_force.setGroup("Output");
    d_displacement.setGroup("Output");
}

template<class DataTypes>
void AffineFunctionModel<DataTypes>::init()
{
    d_componentState.setValue(ComponentState::Invalid);
    SoftRobotsConstraint<DataTypes>::init();

    if(m_state==nullptr)
    {
        msg_error() << "There is no mechanical state associated with this node. "
                           "The object is then deactivated. "
                           "To remove this error message, fix your scene possibly by "
                           "adding a MechanicalObject." ;
        return;
    }

    internalInit();
    d_componentState.setValue(ComponentState::Valid);
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::bwdInit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    // The initial function value is set or computed in bwdInit so the mapping (if there is any)
    // will be considered
    ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();
    ReadAccessor<Data<VecCoord>> restPositions = m_state->readRestPositions();

    Real functionValue = getAffineFunctionValue(positions.ref());
	Real initialFunctionValue = getAffineFunctionValue(restPositions.ref());
    d_initFunctionValue.setValue(initialFunctionValue);
    d_functionValue.setValue(functionValue);
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::reinit()
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    internalInit();
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::reset()
{
    if(d_componentState.getValue() != ComponentState::Valid)
        return ;

    d_functionValue.setValue(d_initFunctionValue.getValue());
}



template<class DataTypes>
void AffineFunctionModel<DataTypes>::internalInit()
{
    checkSizes();
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::checkSizes()
{
	ReadAccessor<Data<VecCoord>> positions = m_state->readPositions();

	for (unsigned int i = 0; i<d_indices.getValue().size(); i++)
	{
		if (positions.size() <= d_indices.getValue()[i])
			msg_error() << "Indices at index " << i << " is too large regarding mechanicalState [position] size";
		if (d_indices.getValue()[i] < 0)
			msg_error() << "Indices at index " << i << " is negative";
	}

    int nbIndices = d_indices.getValue().size();
	int nbCoefficients = d_coefficients.getValue().size();
   
    if ( nbIndices != nbCoefficients){
		msg_error() << "The number of coefficients do not match the number of indices";
    }
}



template<class DataTypes>
SReal AffineFunctionModel<DataTypes>::getAffineFunctionValue(const VecCoord &positions)
{
    const SetIndexArray &indices = d_indices.getValue();
    VecDeriv coefficientsList = d_coefficients.getValue();
	Coord zero;
	
    Real value = d_offset.getValue();
    for (unsigned int i=0; i<indices.size(); i++)
    {
        Coord currentPosition  = positions[indices[i]];
		Deriv positionAxisAngle = DataTypes::coordDifference(currentPosition, zero);
		Deriv coefficients = coefficientsList[i];
		value += positionAxisAngle*coefficients;
    }

    return value;
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams, DataMatrixDeriv &cMatrix, unsigned int &cIndex, const DataVecCoord &x)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);
    SOFA_UNUSED(x);

    m_constraintId = cIndex;

    MatrixDeriv& matrix = *cMatrix.beginEdit();

    MatrixDerivRowIterator rowIterator = matrix.writeLine(m_constraintId);

	const SetIndexArray &indices = d_indices.getValue();
	VecDeriv coefficients = d_coefficients.getValue();

	for (unsigned int i = 0; i < indices.size(); i++)
	{
		rowIterator.setCol(indices[i], coefficients[i]);
	}

    
    cIndex++;
    cMatrix.endEdit();
    m_nbLines = cIndex - m_constraintId;
}


template<class DataTypes>
void AffineFunctionModel<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                   BaseVector *resV,
                                                   const BaseVector *Jdx)
{
    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    SOFA_UNUSED(cParams);

	d_functionValue.setValue(getAffineFunctionValue(m_state->readPositions().ref()));
    Real dfree = Jdx->element(0) + d_functionValue.getValue();
    resV->set(m_constraintId, dfree);
}



template<class DataTypes>
void AffineFunctionModel<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                        core::MultiVecDerivId res,
                                        const sofa::linearalgebra::BaseVector* lambda)
{
    SOFA_UNUSED(res);
    SOFA_UNUSED(cParams);

    if(d_componentState.getValue() != ComponentState::Valid)
            return ;

    d_force.setValue(lambda->element(m_constraintId));

    //Note: this is one step behind
    d_displacement.setValue(d_functionValue.getValue());
}

} // namespace constraintset

} // namespace component

} // namespace sofa

#endif
