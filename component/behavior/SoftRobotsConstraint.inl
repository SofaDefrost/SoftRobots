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
#ifndef SOFA_CORE_BEHAVIOR_SoftRobotsConstraint_INL
#define SOFA_CORE_BEHAVIOR_SoftRobotsConstraint_INL

#include "SoftRobotsConstraint.h"
#include <SofaBaseLinearSolver/FullMatrix.h>

namespace sofa
{

namespace core
{

namespace behavior
{

using component::linearsolver::FullVector;

template<class DataTypes>
SoftRobotsConstraint<DataTypes>::SoftRobotsConstraint(MechanicalState<DataTypes> *mm)
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
        dmsg_error() << "An actuator constraint assume that there is a valid"
                            " context. please fix your code. " ;

    m_state = dynamic_cast< MechanicalState<DataTypes>* >(getContext()->getMechanicalState());
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::getConstraintViolation(const ConstraintParams* cParams,
                                                             BaseVector *resV)
{
    if (cParams)
    {
        const DataVecCoord& xfree = *cParams->readX(m_state);
        const VecCoord& x = m_state->read(VecCoordId::position())->getValue();
        const VecDeriv& v = m_state->read(VecDerivId::velocity())->getValue();

        VecDeriv dx(v);
        for(unsigned int i=0; i<dx.size(); i++)
            dx[i] = DataTypes::coordDifference(xfree.getValue()[i],x[i]);

        const Data<MatrixDeriv> *J = cParams->j()[m_state].read();
        FullVector<Real> Jdx;
        Jdx.resize(m_nbLines);
        Jdx.clear();
        for(unsigned int i=0; i<m_nbLines; i++)
        {
            MatrixDerivRowConstIterator rowIt = J->getValue().readLine(m_constraintId+i);
            for (MatrixDerivColConstIterator colIt = rowIt.begin(); colIt != rowIt.end(); ++colIt)
                Jdx.add(i, colIt.val() * dx[colIt.index()]);
        }

        getConstraintViolation(cParams, resV, &Jdx);
    }
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::buildConstraintMatrix(const ConstraintParams* cParams,
                                                            MultiMatrixDerivId cId,
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
void SoftRobotsConstraint<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                  MultiVecDerivId res,
                                                  const BaseVector* lambda)
{
    if (cParams)
        storeLambda(cParams, *res[m_state].write(), *cParams->readJ(m_state), lambda);
}


template<class DataTypes>
void SoftRobotsConstraint<DataTypes>::storeLambda(const ConstraintParams* cParams,
                                                  Data<VecDeriv>& result,
                                                  const Data<MatrixDeriv>& jacobian,
                                                  const BaseVector* lambda)
{
    auto res = sofa::helper::write(result, cParams);
    const MatrixDeriv& j = jacobian.getValue(cParams);
    j.multTransposeBaseVector(res, lambda ); // lambda is a vector of scalar value so block size is one.
}


template<class DataTypes>
std::string SoftRobotsConstraint<DataTypes>::getTemplateName() const
{
    return templateName(this);
}

template<class DataTypes>
std::string SoftRobotsConstraint<DataTypes>::templateName(const SoftRobotsConstraint<DataTypes>*)
{
    return DataTypes::Name();
}


} // namespace behavior

} // namespace core

} // namespace sofa

#endif // SOFA_CORE_BEHAVIOR_SoftRobotsConstraint_INL
