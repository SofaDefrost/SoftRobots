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
#include <sofa/core/behavior/MixedInteractionForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/core/behavior/DefaultMultiMatrixAccessor.h>
#include <sofa/component/mapping/nonlinear/RigidMapping.h>
#include <sofa/component/mapping/linear/SubsetMultiMapping.h>

namespace softrobots::forcefield
{

/// This class can be overridden if needed for additionnal storage within template specializations.
template<class DataTypes1, class DataTypes2>
class PartialRigidificationForceFieldInternalData
{
};

using sofa::linearalgebra::CompressedRowSparseMatrix ;
using sofa::core::behavior::MixedInteractionForceField ;
using sofa::core::behavior::BaseForceField ;
using sofa::component::mapping::linear::SubsetMultiMapping ;
using sofa::core::behavior::MultiMatrixAccessor ;
using sofa::component::mapping::nonlinear::RigidMapping ;
using sofa::linearalgebra::BaseMatrix ;
using sofa::core::MechanicalParams ;
using sofa::type::Mat ;


template<typename TDataTypes1, typename TDataTypes2>
class PartialRigidificationForceField : public MixedInteractionForceField<TDataTypes1, TDataTypes2>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(PartialRigidificationForceField, TDataTypes1, TDataTypes2), SOFA_TEMPLATE2(MixedInteractionForceField, TDataTypes1, TDataTypes2));

    typedef MixedInteractionForceField<TDataTypes1, TDataTypes2> Inherit;
    // Vec3
    typedef TDataTypes1 DataTypes1;
    typedef typename DataTypes1::VecCoord VecCoord1;
    typedef typename DataTypes1::VecDeriv VecDeriv1;
    typedef typename DataTypes1::Coord    Coord1;
    typedef typename DataTypes1::Deriv    Deriv1;
    typedef typename DataTypes1::Real     Real1;
    typedef typename DataTypes1::MatrixDeriv MatrixDeriv1;
    typedef sofa::Data<MatrixDeriv1>  DataMatrixDeriv1;

    // Rigid
    typedef TDataTypes2 DataTypes2;
    typedef typename DataTypes2::VecCoord VecCoord2;
    typedef typename DataTypes2::VecDeriv VecDeriv2;
    typedef typename DataTypes2::Coord    Coord2;
    typedef typename DataTypes2::Deriv    Deriv2;
    typedef typename DataTypes2::Real     Real2;

    typedef sofa::Data<VecCoord1>    DataVecCoord1;
    typedef sofa::Data<VecDeriv1>    DataVecDeriv1;
    typedef sofa::Data<VecCoord2>    DataVecCoord2;
    typedef sofa::Data<VecDeriv2>    DataVecDeriv2;

    typedef Mat<6, 3, Real1> _6_3_Matrix_Type;
    typedef Mat<6, 6, Real2> _6_6_Matrix_Type;
    typedef Mat<3, 6, Real2> _3_6_Matrix_Type;
    typedef Mat<3, 3, Real1> _3_3_Matrix_Type;

    typedef typename CompressedRowSparseMatrix<_6_6_Matrix_Type>::ColBlockConstIterator _6_6_ColBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_6_3_Matrix_Type>::ColBlockConstIterator _6_3_ColBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::ColBlockConstIterator _3_3_ColBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_6_Matrix_Type>::ColBlockConstIterator _3_6_ColBlockConstIterator;

    typedef typename CompressedRowSparseMatrix<_6_6_Matrix_Type>::RowBlockConstIterator _6_6_RowBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_6_3_Matrix_Type>::RowBlockConstIterator _6_3_RowBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::RowBlockConstIterator _3_3_RowBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_6_Matrix_Type>::RowBlockConstIterator _3_6_RowBlockConstIterator;

    typedef typename CompressedRowSparseMatrix<_6_6_Matrix_Type>::BlockConstAccessor _6_6_BlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<_6_3_Matrix_Type>::BlockConstAccessor _6_3_BlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::BlockConstAccessor _3_3_BlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<_3_6_Matrix_Type>::BlockConstAccessor _3_6_BlockConstAccessor;

    typedef typename CompressedRowSparseMatrix<_6_6_Matrix_Type>::BlockAccessor _6_6_BlockAccessor;
    typedef typename CompressedRowSparseMatrix<_6_3_Matrix_Type>::BlockAccessor _6_3_BlockAccessor;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::BlockAccessor _3_3_BlockAccessor;
    typedef typename CompressedRowSparseMatrix<_3_6_Matrix_Type>::BlockAccessor _3_6_BlockAccessor;


protected:
    sofa::SingleLink < PartialRigidificationForceField<DataTypes1, DataTypes2>, RigidMapping<DataTypes2, DataTypes1>, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK > d_rigidMapping;
    sofa::SingleLink < PartialRigidificationForceField<DataTypes1, DataTypes2>, SubsetMultiMapping<DataTypes1, DataTypes1>, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK > d_subsetMultiMapping;
    sofa::SingleLink < PartialRigidificationForceField<DataTypes1, DataTypes2>, BaseForceField, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK > d_mappedForceField;

    PartialRigidificationForceField() ;


    /**
     * \brief Compressed sparse row matrices product
     * \param A : Left hand side matrix
     * \param B : Right hand side matrix
     * \return A times B
     */
    template<unsigned int M, unsigned int N, unsigned int K>
    static void multMatrices(const CompressedRowSparseMatrix<Mat<M, N, Real1> >& A,
                             const CompressedRowSparseMatrix<Mat<N, K, Real1> >& B,
                             CompressedRowSparseMatrix<Mat<M, K, Real1> >& R) ;

    void testBuildJacobian(const MechanicalParams* mparams);
    /**
     * \brief Compressed sparse row matrices transposed product
     * \param A : Left hand side matrix
     * \param B : Right hand side matrix
     * \return A^t times B
     */
    template<unsigned int M, unsigned int N, unsigned int K>
    static void multMatricesT(const CompressedRowSparseMatrix<Mat<M, N, Real1> >& At,
                              const CompressedRowSparseMatrix<Mat<M, K, Real1> >& B,
                              CompressedRowSparseMatrix<Mat<N, K, Real1> >& R) ;
public:

    ///////////////////////// Inherited from MixedInteractionForceField ///////////////////
    void addForce(const MechanicalParams* mparams,
                  DataVecDeriv1& f1,
                  DataVecDeriv2& f2,
                  const DataVecCoord1& x1,
                  const DataVecCoord2& x2,
                  const DataVecDeriv1& v1,
                  const DataVecDeriv2& v2) override;

    void addDForce(const MechanicalParams* mparams,
                   DataVecDeriv1& df1,
                   DataVecDeriv2& df2,
                   const DataVecDeriv1& dx1,
                   const DataVecDeriv2& dx2) override;

    double getPotentialEnergy(const MechanicalParams* mparams,
                              const DataVecCoord1& x1, const DataVecCoord2& x2) const override;
    ////////////////////////////////////////////////////////////////////////////////////


    ///////////////////////// Inherited from BaseInteractionForceField ///////////////////
    void addKToMatrix(const MechanicalParams* mparams,
                      const MultiMatrixAccessor* matrix ) override;
    ////////////////////////////////////////////////////////////////////////////////////
protected:


    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using MixedInteractionForceField<TDataTypes1, TDataTypes2>::f_printLog ;
    using MixedInteractionForceField<TDataTypes1, TDataTypes2>::mstate1 ;
    using MixedInteractionForceField<TDataTypes1, TDataTypes2>::mstate2 ;
    using MixedInteractionForceField<TDataTypes1, TDataTypes2>::getContext ;
    ////////////////////////////////////////////////////////////////////////////
};

#if !defined(SOFTROBOTS_PARTIALRIGIDIFICATIONFORCEFIELD_CPP)
extern template class SOFA_SOFTROBOTS_API PartialRigidificationForceField<sofa::defaulttype::Vec3Types, sofa::defaulttype::Rigid3Types>;
#endif

} // namespace

namespace sofa::component::forcefield
{
    template <class DataTypes1, class DataTypes2>
    using PartialRigidificationForceField SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::forcefield::PartialRigidificationForceField<DataTypes1, DataTypes2>;
}

