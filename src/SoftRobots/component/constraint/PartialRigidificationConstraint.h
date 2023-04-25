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

#include <SoftRobots/component/initSoftRobots.h>
#include <sofa/core/behavior/Constraint.h>
#include <sofa/core/behavior/ConstraintResolution.h>
#include <sofa/linearalgebra/BaseVector.h>

namespace sofa::component::constraintset
{

using sofa::core::behavior::ConstraintResolution ;
using sofa::core::ConstraintParams ;
using sofa::core::behavior::Constraint ;
using sofa::linearalgebra::BaseVector ;
using sofa::type::Vec ;

class PartialRigidificationConstraintResolution6Dof : public ConstraintResolution
{
public:
    PartialRigidificationConstraintResolution6Dof() ;
};

template< class DataTypes >
class PartialRigidificationConstraint : public Constraint<DataTypes>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE(PartialRigidificationConstraint,DataTypes),
               SOFA_TEMPLATE(core::behavior::Constraint,DataTypes));

    typedef typename DataTypes::VecCoord        VecCoord;
    typedef typename DataTypes::VecDeriv        VecDeriv;
    typedef typename DataTypes::Coord           Coord;
    typedef typename DataTypes::Deriv           Deriv;
    typedef typename DataTypes::MatrixDeriv     MatrixDeriv;
    typedef typename Coord::value_type          Real;
    typedef Vec<3, Real>                        Vec3;
    typedef typename core::behavior::MechanicalState<DataTypes> MechanicalState;

    typedef typename MatrixDeriv::RowIterator   MatrixDerivRowIterator;
    typedef Data<VecCoord>           DataVecCoord;
    typedef Data<VecDeriv>           DataVecDeriv;
    typedef Data<MatrixDeriv>        DataMatrixDeriv;

public:
    //////////////////// Inherited from BaseObject /////////////////////
    void init() override;
    ////////////////////////////////////////////////////////////////////

    //////////////////// Inherited from Constraint /////////////////////
    void buildConstraintMatrix(const ConstraintParams* cParams,
                               DataMatrixDeriv &cMatrix,
                               unsigned int &cIndex,
                               const DataVecCoord &x) override;

    void getConstraintViolation(const ConstraintParams* cParams,
                                BaseVector *resV,
                                const DataVecCoord &xfree,
                                const DataVecDeriv &vfree) override;

    void getConstraintResolution(std::vector<ConstraintResolution*>& resTab,
                                 unsigned int& offset) override;
    ////////////////////////////////////////////////////////////////////

protected:
    unsigned int m_cid;

    PartialRigidificationConstraint(MechanicalState* object = nullptr) ;
    ~PartialRigidificationConstraint() override;

private:

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using Constraint<DataTypes>::mstate ;
    ////////////////////////////////////////////////////////////////////////////
};

#if !defined(SOFTROBOTS_PARTIALRIGIDIFICATIONCONSTRAINT_CPP)
extern template class SOFA_SOFTROBOTS_API PartialRigidificationConstraint<sofa::defaulttype::Rigid3Types>;
#endif
} // namespace sofa::component::constraintset

