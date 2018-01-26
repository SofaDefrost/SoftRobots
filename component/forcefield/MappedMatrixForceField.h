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
#ifndef SOFA_COMPONENT_FORCEFIELD_MappedMatrixForceField_H
#define SOFA_COMPONENT_FORCEFIELD_MappedMatrixForceField_H

#include <sofa/core/behavior/MixedInteractionForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/core/objectmodel/Link.h>
#include <sofa/core/MechanicalParams.h>
#include <SofaBaseLinearSolver/CompressedRowSparseMatrix.h>
#include <SofaBaseLinearSolver/DefaultMultiMatrixAccessor.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaMiscMapping/SubsetMultiMapping.h>

#include <sofa/core/topology/BaseMeshTopology.h>



// add visitor implementation
#include <sofa/simulation/MechanicalVisitor.h>
#include <sofa/core/ConstraintParams.h>
#include <sofa/core/MultiVecId.h>
#include <sofa/core/BaseMapping.h>
#include <sofa/defaulttype/BaseMatrix.h>


namespace sofa
{

namespace component
{

namespace interactionforcefield
{


class SOFA_CONSTRAINT_API MechanicalAccumulateJacobian : public simulation::BaseMechanicalVisitor
{
public:
    MechanicalAccumulateJacobian(const core::ConstraintParams* _cparams, core::MultiMatrixDerivId _res)
        : simulation::BaseMechanicalVisitor(_cparams)
        , res(_res)
        , cparams(_cparams)
    {

    }

    virtual void bwdMechanicalMapping(simulation::Node* node, core::BaseMapping* map)
    {
        ctime_t t0 = begin(node, map);
        map->applyJT(cparams, res, res);
        end(node, map, t0);
    }

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const { return "MechanicalAccumulateJacobian"; }

    virtual bool isThreadSafe() const
    {
        return false;
    }
    // This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    virtual bool stopAtMechanicalMapping(simulation::Node* /*node*/, core::BaseMapping* /*map*/)
    {
        return false; // !map->isMechanical();
    }

#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors()
    {
    }
#endif

protected:
    core::MultiMatrixDerivId res;
    const sofa::core::ConstraintParams *cparams;
};






using sofa::component::linearsolver::CompressedRowSparseMatrix ;
using sofa::core::behavior::MixedInteractionForceField ;
using sofa::core::behavior::BaseForceField ;
using sofa::core::behavior::MultiMatrixAccessor ;
using sofa::defaulttype::BaseMatrix ;
using sofa::core::MechanicalParams ;
using sofa::defaulttype::Mat ;


template<typename TDataTypes1, typename TDataTypes2>
class MappedMatrixForceField : public MixedInteractionForceField<TDataTypes1, TDataTypes2>
{
public:
    SOFA_CLASS(SOFA_TEMPLATE2(MappedMatrixForceField, TDataTypes1, TDataTypes2), SOFA_TEMPLATE2(MixedInteractionForceField, TDataTypes1, TDataTypes2));

    typedef MixedInteractionForceField<TDataTypes1, TDataTypes2> Inherit;
    // Vec3
    typedef TDataTypes1 DataTypes1;
    typedef typename DataTypes1::VecCoord VecCoord1;
    typedef typename DataTypes1::VecDeriv VecDeriv1;
    typedef typename DataTypes1::Coord    Coord1;
    typedef typename DataTypes1::Deriv    Deriv1;
    typedef typename DataTypes1::Real     Real1;
    typedef typename DataTypes1::MatrixDeriv MatrixDeriv1;
    typedef Data<MatrixDeriv1>  DataMatrixDeriv1;
    typedef typename DataTypes1::MatrixDeriv::RowIterator MatrixDeriv1RowIterator;
    typedef typename DataTypes1::MatrixDeriv::ColIterator MatrixDeriv1ColIterator;
    typedef typename DataTypes1::MatrixDeriv::RowConstIterator MatrixDeriv1RowConstIterator;
    typedef typename DataTypes1::MatrixDeriv::ColConstIterator MatrixDeriv1ColConstIterator;
    static const unsigned int DerivSize1 = Deriv1::total_size;


    // Rigid
    typedef TDataTypes2 DataTypes2;
    typedef typename DataTypes2::VecCoord VecCoord2;
    typedef typename DataTypes2::VecDeriv VecDeriv2;
    typedef typename DataTypes2::Coord    Coord2;
    typedef typename DataTypes2::Deriv    Deriv2;
    typedef typename DataTypes2::Real     Real2;
    typedef typename DataTypes2::MatrixDeriv MatrixDeriv2;
    typedef Data<MatrixDeriv2>  DataMatrixDeriv2;
    typedef typename DataTypes2::MatrixDeriv::RowIterator MatrixDeriv2RowIterator;
    typedef typename DataTypes2::MatrixDeriv::ColIterator MatrixDeriv2ColIterator;
    typedef typename DataTypes2::MatrixDeriv::RowConstIterator MatrixDeriv2RowConstIterator;
    typedef typename DataTypes2::MatrixDeriv::ColConstIterator MatrixDeriv2ColConstIterator;
    static const unsigned int DerivSize2 = Deriv2::total_size;

    typedef Data<VecCoord1>    DataVecCoord1;
    typedef Data<VecDeriv1>    DataVecDeriv1;
    typedef Data<VecCoord2>    DataVecCoord2;
    typedef Data<VecDeriv2>    DataVecDeriv2;

    typedef Mat<6, 3, Real1> _6_3_Matrix_Type;
    typedef Mat<6, 6, Real2> _6_6_Matrix_Type;
    typedef Mat<3, 6, Real2> _3_6_Matrix_Type;
    typedef Mat<3, 3, Real1> _3_3_Matrix_Type;

    typedef sofa::defaulttype::BaseVector::Index  Index;

    typedef typename CompressedRowSparseMatrix<Real1>::Range  Range;
    typedef typename CompressedRowSparseMatrix<Real1>::ColBlockConstIterator _1_1_ColBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<Real1>::RowBlockConstIterator _1_1_RowBlockConstIterator;
    typedef typename CompressedRowSparseMatrix<Real1>::BlockConstAccessor _1_1_BlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Real1>::BlockAccessor _1_1_BlockAccessor;


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
    SingleLink < MappedMatrixForceField<DataTypes1, DataTypes2>, BaseForceField, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK > d_mappedForceField;
    SingleLink < MappedMatrixForceField<DataTypes1, DataTypes2>, BaseForceField, BaseLink::FLAG_STOREPATH | BaseLink::FLAG_STRONGLINK > d_mappedForceField2;

    MappedMatrixForceField() ;

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

    virtual void init();

    virtual void addForce(const MechanicalParams* mparams,
                          DataVecDeriv1& f1,
                          DataVecDeriv2& f2,
                          const DataVecCoord1& x1,
                          const DataVecCoord2& x2,
                          const DataVecDeriv1& v1,
                          const DataVecDeriv2& v2) ;

    virtual void addDForce(const MechanicalParams* mparams,
                           DataVecDeriv1& df1,
                           DataVecDeriv2& df2,
                           const DataVecDeriv1& dx1,
                           const DataVecDeriv2& dx2) ;

    virtual void addKToMatrix(const MechanicalParams* mparams,
                              const MultiMatrixAccessor* matrix ) ;

    virtual double getPotentialEnergy(const MechanicalParams* mparams,
                                      const DataVecCoord1& x1, const DataVecCoord2& x2) const ;




protected:


    void accumulateJacobians(const MechanicalParams* mparams);


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

    sofa::helper::vector<unsigned int> listActiveNodes;
    Data< bool > performECSW;
    sofa::core::objectmodel::DataFileName listActiveNodesPath;


};

} // namespace interactionforcefield

} // namespace component

} // namespace sofa


#endif
