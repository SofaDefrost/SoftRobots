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
#pragma once

#include <sofa/linearalgebra/FullMatrix.h>
#include <sofa/core/ConstraintParams.h>

#include <SoftRobots/component/behavior/SoftRobotsConstraint.h>

namespace softrobots::behavior
{

using sofa::linearalgebra::FullVector;
using sofa::helper::ReadAccessor;

template<class DataTypes>
SoftRobotsConstraint<DataTypes>::SoftRobotsConstraint(sofa::core::behavior::MechanicalState<DataTypes> *mm)
    : d_endTime( initData(&d_endTime,(Real)-1, "endTime",
                          "The SoftRobotsConstraint stops acting after the given value.\n"
                          "Use a negative value for infinite SoftRobotsConstraints") )
    , m_state(mm)
{
}

template<class DataTypes>
SoftRobotsConstraint<DataTypes>::~SoftRobotsConstraint()
{
}


template <class DataTypes>
bool SoftRobotsConstraint<DataTypes>::isActive() const
{
    if( d_endTime.getValue()<0 ) return true;
    return d_endTime.getValue()>getContext()->getTime();
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::init()
{
    BaseConstraint::init();

    /// This throw a LogicException (logic exception are not meant for users but for
    /// developpers).
    if(getContext()==nullptr)
        dmsg_error() << "A constraint assumes that there is a valid context. Please fix your code. " ;

    m_state = dynamic_cast<sofa::core::behavior::MechanicalState<DataTypes>* >(getContext()->getMechanicalState());
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::getConstraintViolation(const sofa::core::ConstraintParams* cParams,
                                                             BaseVector *resV)
{
    if (cParams)
    {
        const DataVecCoord& xfree = *cParams->readX(m_state);
        ReadAccessor<sofa::Data<VecCoord>> x = m_state->readPositions();
        const auto& constraintIndex = sofa::helper::getReadAccessor(m_constraintIndex);

        VecDeriv dx;
        dx.resize(m_state->getSize());
        for(unsigned int i=0; i<dx.size(); i++)
            dx[i] = DataTypes::coordDifference(xfree.getValue()[i],x[i]);

        const sofa::Data<MatrixDeriv> *J = cParams->j()[m_state].read();
        FullVector<Real> Jdx;
        Jdx.resize(m_nbLines);
        Jdx.clear();
        for(unsigned int i=0; i<m_nbLines; i++)
        {
            MatrixDerivRowConstIterator rowIt = J->getValue().readLine(constraintIndex + i);
            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
                Jdx.add(i, colIt.val() * dx[colIt.index()]);
        }

        getConstraintViolation(cParams, resV, &Jdx);
    }
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::buildConstraintMatrix(const sofa::core::ConstraintParams* cParams,
                                                            sofa::core::MultiMatrixDerivId cId,
                                                            unsigned int &cIndex)
{
    if (cParams)
    {
        buildConstraintMatrix(cParams, *cId[m_state].write(), cIndex, m_state->readPositions().ref());
        // Give *cParams->readX(m_state) -> xfree
        //buildConstraintMatrix(cParams, *cId[m_state].write(), cIndex, *cParams->readX(m_state));
    }
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::storeLambda(const sofa::core::ConstraintParams* cParams,
                                                  sofa::core::MultiVecDerivId res,
                                                  const BaseVector* lambda)
{
    if (cParams)
        storeLambda(cParams, *res[m_state].write(), *cParams->readJ(m_state), lambda);
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::storeLambda(const sofa::core::ConstraintParams* cParams,
                                                  sofa::Data<VecDeriv>& result,
                                                  const sofa::Data<MatrixDeriv>& jacobian,
                                                  const BaseVector* lambda)
{
    SOFA_UNUSED(cParams);
    auto res = sofa::helper::getWriteAccessor(result);
    const MatrixDeriv& j = jacobian.getValue();
    j.multTransposeBaseVector(res, lambda ); // lambda is a vector of scalar value so block size is one.
}

} // namespace softrobots::behavior

