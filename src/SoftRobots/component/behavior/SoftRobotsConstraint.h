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
#ifndef SOFA_CORE_BEHAVIOR_SOFTROBOTSCONSTRAINT_H
#define SOFA_CORE_BEHAVIOR_SOFTROBOTSCONSTRAINT_H

#include <sofa/core/behavior/MechanicalState.h>
#include "SoftRobotsBaseConstraint.h"

#include <SoftRobots/component/initSoftRobots.h>

namespace sofa
{

namespace core
{

namespace behavior
{

using sofa::core::objectmodel::BaseContext ;
using sofa::core::objectmodel::BaseObjectDescription ;
using sofa::linearalgebra::BaseVector;

/**
 *  \brief Component computing inverse problem constraints within a simulated body.
 *
 *  This class defines the abstract API common to inverse problem constraints using a given type
 *  of DOFs.
 */

template<class DataTypes>
class SOFA_SOFTROBOTS_API SoftRobotsConstraint : public SoftRobotsBaseConstraint
{
public:

    SOFA_CLASS(SOFA_TEMPLATE(SoftRobotsConstraint,DataTypes), SoftRobotsBaseConstraint);

    typedef typename DataTypes::Real        Real;
    typedef typename DataTypes::VecCoord    VecCoord;
    typedef typename DataTypes::VecDeriv    VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

    typedef Data<VecCoord>		DataVecCoord;
    typedef Data<VecDeriv>		DataVecDeriv;
    typedef Data<MatrixDeriv>   DataMatrixDeriv;

public:
    //////////////////////////////// Came from BaseObject ///////////////////////////
    void init() override;
    /////////////////////////////////////////////////////////////////////////////////

    ///< if false, the constraint does nothing
    virtual bool isActive() const;

    /// Retrieve the associated MechanicalState
    MechanicalState<DataTypes>* getMState() { return m_state; }

    /// Construct the constraint violations vector of each constraint
    ///
    /// \param resV is the result vector that contains the whole constraint violations
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV) override;

    /// Construct the constraint violations vector of each constraint
    ///
    /// \param resV is the result vector that contains the whole constraint violations
    /// \param Jdx = J(xfree - x) for the constraint, has the size of m_nbLines
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    ///
    /// This is the method that should be implemented by the component
    using SoftRobotsBaseConstraint::getConstraintViolation; // to avoid warning about hidden overloaded virtual function
    virtual void getConstraintViolation(const ConstraintParams* cParams,
                                        BaseVector *resV,
                                        const BaseVector *Jdx) = 0;

    /// Construct the Jacobian Matrix
    ///
    /// \param cId is the result constraint sparse matrix Id
    /// \param cIndex is the index of the next constraint equation: when building the constraint matrix, you have to use this index, and then update it
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    ///
    /// Warning: is constraint matrix is built with the current position. Free configuration available in cParams
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               MultiMatrixDerivId cId,
                               unsigned int &cIndex) override;

    /// Construct the Jacobian Matrix
    ///
    /// \param c is the result constraint sparse matrix
    /// \param cIndex is the index of the next constraint equation: when building the constraint matrix, you have to use this index, and then update it
    /// \param x is the position vector used for contraint equation computation
    /// \param cParams defines the state vectors to use for positions and velocities. Also defines the order of the constraint (POS, VEL, ACC)
    ///
    /// This is the method that should be implemented by the component
    virtual void buildConstraintMatrix(const ConstraintParams* cParams,
                                       DataMatrixDeriv &c,
                                       unsigned int &cIndex,
                                       const DataVecCoord &x) = 0;


    void storeLambda(const ConstraintParams* cParams, MultiVecDerivId res, const linearalgebra::BaseVector* lambda) override;


protected:
    Data<Real> d_endTime;  ///< Time when the constraint becomes inactive (-1 for infinitely active)

    SoftRobotsConstraint(MechanicalState<DataTypes> *mm = NULL);
    ~SoftRobotsConstraint() override;

    MechanicalState<DataTypes> *m_state; ///< Associated mechanical state

private:
    void storeLambda(const ConstraintParams* cParams, Data<VecDeriv>& resId, const Data<MatrixDeriv>& jacobian, const sofa::linearalgebra::BaseVector* lambda);
};


// Force template specialization for the most common sofa float related type.
// This goes with the extern template declaration in the .h. Declaring extern template
// avoid the code generation of the template for each compilation unit.
// see: http://www.stroustrup.com/C++11FAQ.html#extern-templates

extern template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<defaulttype::Vec3Types>;
extern template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<defaulttype::Vec2Types>;
extern template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<defaulttype::Vec1Types>;
extern template class SOFA_SOFTROBOTS_API SoftRobotsConstraint<defaulttype::Rigid3Types>;



} // namespace behavior

} // namespace core

} // namespace sofa

#endif
