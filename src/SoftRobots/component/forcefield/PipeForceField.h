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
#include <sofa/component/solidmechanics/spring/MeshSpringForceField.h>
#include <sofa/type/Vec.h>
#include <sofa/type/Mat.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/MechanicalParams.h>
#include <sofa/linearalgebra/CompressedRowSparseMatrix.h>
#include <sofa/core/behavior/DefaultMultiMatrixAccessor.h>
#include <sofa/component/mapping/linear/BarycentricMapping.h>
#include <sofa/core/behavior/ForceField.h>

namespace softrobots::forcefield
{
using sofa::type::Vec ;
using sofa::type::Mat ;
using sofa::type::vector;
using sofa::core::MechanicalParams;
using sofa::linearalgebra::BaseMatrix;
using sofa::core::behavior::ForceField ;
using sofa::linearalgebra::CompressedRowSparseMatrix ;
using sofa::component::mapping::linear::BarycentricMapping;
using sofa::component::solidmechanics::spring::MeshSpringForceField ;
using sofa::core::behavior::MultiMatrixAccessor ;


/**
 * This component is used to pull up mapped string forces (from a child node)
 * Remove this component and change the simulations: project/siggraph/tentacle... when a clean implementation has been done
*/
template<typename DataTypes>
class PipeForceField : public ForceField<DataTypes>
{
public :
    SOFA_CLASS(SOFA_TEMPLATE(PipeForceField, DataTypes), SOFA_TEMPLATE(ForceField, DataTypes));

    typedef typename DataTypes::Real     Real;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::Coord    Coord;
    typedef typename DataTypes::Deriv    Deriv;

    typedef sofa::Data<VecCoord>    DataVecCoord;
    typedef sofa::Data<VecDeriv>    DataVecDeriv;

    typedef Vec<3, Real>                Vec3;
    typedef Vec<6, Real>                Vec6;
    typedef Vec<12, Real>               Vec12;
    typedef Mat<3, 3, Real>             Mat33;
    typedef Mat<3, 3, Real>             Mat44;
    typedef Mat<6, 6, Real>             Mat66;
    typedef Mat<12, 12, Real>           Mat12x12;

    typedef CompressedRowSparseMatrix<Mat66> CSRMatB66;

    typedef typename CompressedRowSparseMatrix<Mat66>::ColBlockConstIterator CSRMatB66ColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat66>::RowBlockConstIterator CSRMatB66RowBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat66>::BlockConstAccessor CSRMatB66BlockConstAccessor;

    typedef sofa::type::vector<Vec6> VecVec6;
    typedef sofa::type::vector<Mat66> VecMat66;
    typedef sofa::type::vector<Mat12x12> VecMat12;

    typedef Mat<3, 3, Real> _3_3_Matrix_Type;

    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::ColBlockConstIterator _3_3_ColBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::RowBlockConstIterator _3_3_RowBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::BlockConstAccessor _3_3_BlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<_3_3_Matrix_Type>::BlockAccessor _3_3_BlockAccessor;


public :
    PipeForceField();
    virtual ~PipeForceField();

    ////////////////////////// Inherited from BaseObject /////////////////////////
    void init() override;
    ///////////////////////////////////////////////////////////////////////////

    ////////////////////////// Inherited from ForceField /////////////////////////
    void addForce(const MechanicalParams* mparams,
                  DataVecDeriv& f ,
                  const DataVecCoord& x ,
                  const DataVecDeriv& v) override;

    void addDForce(const MechanicalParams* mparams,
                   DataVecDeriv&   df ,
                   const DataVecDeriv&
                   dx ) override;


    void addKToMatrix(const MechanicalParams* mparams,
                      const MultiMatrixAccessor* matrix) override;

    double getPotentialEnergy(const MechanicalParams* mparams,
                              const DataVecCoord& x) const override;
    ////////////////////////////////////////////////////////////////////////////

protected:

    sofa::core::behavior::MechanicalState<DataTypes> * m_mstate;

    sofa::SingleLink < PipeForceField<DataTypes>, BarycentricMapping<DataTypes, DataTypes>, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK > l_barycentricMapping;
    sofa::SingleLink < PipeForceField<DataTypes>, MeshSpringForceField<DataTypes>, sofa::BaseLink::FLAG_STOREPATH | sofa::BaseLink::FLAG_STRONGLINK > l_mappedForceField;


    /**
     * \brief Compressed sparse row matrices product
     * \param A : Left hand side matrix
     * \param B : Right hand side matrix
     * \return A times B
     */
    template<unsigned int M, unsigned int N, unsigned int K>
    static void multMatrices(const CompressedRowSparseMatrix<Mat<M, N, Real> >& A,
                             const CompressedRowSparseMatrix<Mat<N, K, Real> >& B,
                             CompressedRowSparseMatrix<Mat<M, K, Real> >& R) ;


    /**
     * \brief Compressed sparse row matrices transposed product
     * \param A : Left hand side matrix
     * \param B : Right hand side matrix
     * \return A^t times B
     */
    template<unsigned int M, unsigned int N, unsigned int K>
    static void multMatricesT(const CompressedRowSparseMatrix<Mat<M, N, Real> >& At,
                              const CompressedRowSparseMatrix<Mat<M, K, Real> >& B,
                              CompressedRowSparseMatrix<Mat<N, K, Real> >& R) ;


private :

    ////////////////////////// Inherited attributes ////////////////////////////
    /// https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    /// Bring inherited attributes and function in the current lookup context.
    /// otherwise any access to the base::attribute would require
    /// the "this->" approach.
    using ForceField<DataTypes>::getContext ;
    using ForceField<DataTypes>::f_printLog ;
    using ForceField<DataTypes>::mstate ;
    ////////////////////////////////////////////////////////////////////////////
};

#if !defined(SOFTROBOTS_PIPEFORCEFIELD_CPP)
extern template class SOFA_SOFTROBOTS_API PipeForceField<sofa::defaulttype::Vec3Types>;
#endif

} // namespace

namespace sofa::component::forcefield
{
    template <class DataTypes>
    using PipeForceField SOFA_ATTRIBUTE_DEPRECATED__RENAME_NAMESPACE_SOFTROBOTS()
        = softrobots::forcefield::PipeForceField<DataTypes>;
}
