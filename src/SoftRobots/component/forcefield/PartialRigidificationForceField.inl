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

#include <sofa/core/visual/VisualParams.h>
#include <sofa/config.h>
#include <sofa/helper/rmath.h>
#include <assert.h>
#include <iostream>
#include <fstream>

#include <SoftRobots/component/forcefield/PartialRigidificationForceField.h>
#include <sofa/core/behavior/MixedInteractionForceField.inl>

namespace softrobots::forcefield
{


using sofa::core::behavior::DefaultMultiMatrixAccessor ;
using sofa::core::behavior::BaseMechanicalState ;


template<class DataTypes1, class DataTypes2>
PartialRigidificationForceField<DataTypes1, DataTypes2>::PartialRigidificationForceField()
    :
      d_rigidMapping(initLink("rigidMapping",
                              "link to the rigidMapping that does the rigidification")),
      d_subsetMultiMapping(initLink("subsetMultiMapping",
                                    "link to the subsetMultiMapping that unifies rigid and deformable parts")),
      d_mappedForceField(initLink("mappedForceField",
                                  "link to the forcefield that is mapped under the subsetMultiMapping"))
{
}



template<class DataTypes1, class DataTypes2>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::addKToMatrix(const MechanicalParams* mparams,
                                                                           const MultiMatrixAccessor* matrix)
{
    MultiMatrixAccessor::MatrixRef mat11 = matrix->getMatrix(mstate1);
    MultiMatrixAccessor::MatrixRef mat22 = matrix->getMatrix(mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat12 = matrix->getMatrix(mstate1, mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat21 = matrix->getMatrix(mstate2, mstate1);

    const sofa::type::vector<BaseMatrix*>* J0J1 = d_subsetMultiMapping.get()->getJs();

    if(J0J1 == nullptr)
        dmsg_warning()<<"J0J1 null";

    if (J0J1->size() != 2)
    {
        msg_error() << "SubsetMultiMapping does not have two output mechanical states. This is not handled in PartialRigidification. AddKToMatrix not computed.";
        return;
    }
    CompressedRowSparseMatrix<_3_3_Matrix_Type>* J0;
    CompressedRowSparseMatrix<_3_3_Matrix_Type>* J1;
    J0 = dynamic_cast< CompressedRowSparseMatrix<_3_3_Matrix_Type> *> ( (*J0J1)[0] );
    J1 = dynamic_cast< CompressedRowSparseMatrix<_3_3_Matrix_Type> *> ( (*J0J1)[1] );


    //Get the jacobian matrix from the rigidMapping
    const CompressedRowSparseMatrix<_3_6_Matrix_Type>* Jr;
    Jr = dynamic_cast<const CompressedRowSparseMatrix<_3_6_Matrix_Type> *> (d_rigidMapping.get()->getJ() );

    if(J0 == NULL) {
        dmsg_error() << "J0 null";
    }
    if(J1 == NULL) {
        dmsg_error() << "J1 null";
    }
    if(Jr == NULL) {
        dmsg_error() << "Jr null";
    }
    if((J0 == NULL) || (J1 == NULL)  || (Jr == NULL) ) {
        msg_error() << "A jacobian matrix from Mapping is missing. AddKToMatrix not computed";
        return;
    }

    // get the stiffness matrix from the mapped ForceField
    CompressedRowSparseMatrix< _3_3_Matrix_Type >* mappedFFMatrix = new CompressedRowSparseMatrix< _3_3_Matrix_Type > ( );

    sofa::core::behavior::BaseMechanicalState* mstate = d_mappedForceField.get()->getContext()->getMechanicalState();
    mappedFFMatrix->resizeBloc( mstate->getSize() ,  mstate->getSize());

    DefaultMultiMatrixAccessor* mappedFFMatrixAccessor;
    mappedFFMatrixAccessor = new DefaultMultiMatrixAccessor;

    mappedFFMatrixAccessor->addMechanicalState(  d_mappedForceField.get()->getContext()->getMechanicalState() );
    mappedFFMatrixAccessor->setGlobalMatrix(mappedFFMatrix);
    mappedFFMatrixAccessor->setupMatrices();

    MultiMatrixAccessor::MatrixRef r = mappedFFMatrixAccessor->getMatrix(  d_mappedForceField.get()->getContext()->getMechanicalState()  );

    d_mappedForceField.get()->addKToMatrix(mparams, mappedFFMatrixAccessor);

    CompressedRowSparseMatrix<_3_6_Matrix_Type> J1Jr ;
    CompressedRowSparseMatrix<_3_3_Matrix_Type> J0tK ;
    CompressedRowSparseMatrix<_3_3_Matrix_Type> J0tKJ0 ;
    CompressedRowSparseMatrix<_3_6_Matrix_Type> J0tKJ1Jr ;
    CompressedRowSparseMatrix<_6_6_Matrix_Type> JrtJ1tKJ1Jr ;
    CompressedRowSparseMatrix<_6_3_Matrix_Type> JrtJ1tK ;
    CompressedRowSparseMatrix<_6_3_Matrix_Type> JrtJ1tKJ0 ;

    CompressedRowSparseMatrix<_3_3_Matrix_Type>* K ;
    K = dynamic_cast<CompressedRowSparseMatrix<_3_3_Matrix_Type>*>(r.matrix);

    multMatrices <3, 3, 6>(*J1 ,    *Jr , J1Jr);
    multMatricesT<3, 3, 3>(*J0 ,    *K  , J0tK);
    multMatrices <3, 3, 3>(J0tK,    *J0 , J0tKJ0);     //mat11
    multMatrices <3, 3, 6>(J0tK,    J1Jr, J0tKJ1Jr);   // mat12
    multMatricesT<3, 6, 3>(J1Jr,    *K  , JrtJ1tK);
    multMatrices <6, 3, 6>(JrtJ1tK, J1Jr, JrtJ1tKJ1Jr);// mat22
    multMatrices <6, 3, 3>(JrtJ1tK, *J0 , JrtJ1tKJ0);  // mat21


    /**************************************** Add J0tKJ0 to global system **********************************************/

    _3_3_Matrix_Type K11MatrixBuffer;
    for(size_t k11RowIndex = 0 ; k11RowIndex < (unsigned int)J0tKJ0.nBlockRow ; ++k11RowIndex)
    {
        for(_3_3_ColBlockConstIterator k11ColIter = J0tKJ0.bRowBegin(k11RowIndex); k11ColIter < J0tKJ0.bRowEnd(k11RowIndex) ; k11ColIter++)
        {

            _3_3_BlockConstAccessor k11Block = k11ColIter.bloc();

            const _3_3_Matrix_Type& k11BlockData = *(const _3_3_Matrix_Type*) k11Block.elements(K11MatrixBuffer.ptr()); // get the block

            int k11ColIndex = k11Block.getCol(); // get the block column index

            for (int i = 0; i < 3 ; i++)
                for (int j = 0; j < 3; j++)
                { mat11.matrix->add(mat11.offset + 3 * k11RowIndex + i, mat11.offset + 3 * k11ColIndex + j, k11BlockData(i, j)); }
        }
    }

    /**************************************** Add JrtJ1tKJ1Jr to global system **********************************************/

    _6_6_Matrix_Type K22MatrixBuffer;
    for(size_t k22RowIndex = 0 ; k22RowIndex < (unsigned int)JrtJ1tKJ1Jr.nBlockRow ; ++k22RowIndex)
    {
        for(_6_6_ColBlockConstIterator k22ColIter = JrtJ1tKJ1Jr.bRowBegin(k22RowIndex); k22ColIter < JrtJ1tKJ1Jr.bRowEnd(k22RowIndex) ; k22ColIter++)
        {

            _6_6_BlockConstAccessor k22Block = k22ColIter.bloc();

            const _6_6_Matrix_Type& k22BlockData = *(const _6_6_Matrix_Type*) k22Block.elements(K22MatrixBuffer.ptr()); // get the block

            int k22ColIndex = k22Block.getCol(); // get the block column index

            for (int i = 0; i < 6 ; i++)
                for (int j = 0; j < 6; j++)
                { mat22.matrix->add(mat22.offset + 6 * k22RowIndex + i, mat22.offset + 6 * k22ColIndex + j, k22BlockData(i, j)); }
        }
    }

    _3_6_Matrix_Type K12MatrixBuffer;

    for(size_t k12RowIndex = 0 ; k12RowIndex < (unsigned int)J0tKJ1Jr.nBlockRow ; ++k12RowIndex)
    {
        for(_3_6_ColBlockConstIterator k12ColIter = J0tKJ1Jr.bRowBegin(k12RowIndex) ; k12ColIter < J0tKJ1Jr.bRowEnd(k12RowIndex) ; k12ColIter++ )
        {
            _3_6_BlockConstAccessor k12Block = k12ColIter.bloc();
            const _3_6_Matrix_Type& k12BlockData = *(const _3_6_Matrix_Type*) k12Block.elements(K12MatrixBuffer.ptr());

            int k12ColIndex = k12Block.getCol();

            for(int i = 0 ; i < 3 ; ++i)
                for(int j = 0 ; j < 6 ; ++j)
                    mat12.matrix->add(mat12.offRow + 3 * k12RowIndex + i, mat12.offCol + 6 * k12ColIndex + j, k12BlockData(i, j));
        }
    }

    _6_3_Matrix_Type K21MatrixBuffer;

    for(size_t k21RowIndex = 0 ; k21RowIndex < (unsigned int)JrtJ1tKJ0.nBlockRow ; ++k21RowIndex) {
        for(_6_3_ColBlockConstIterator k21ColIter = JrtJ1tKJ0.bRowBegin(k21RowIndex) ; k21ColIter < JrtJ1tKJ0.bRowEnd(k21RowIndex) ; k21ColIter++)
        {
            _6_3_BlockConstAccessor k21Block = k21ColIter.bloc();
            const _6_3_Matrix_Type k21BlockData = *(const _6_3_Matrix_Type*) k21Block.elements(K21MatrixBuffer.ptr());

            int k21ColIndex = k21Block.getCol();

            for(int i = 0 ; i < 6 ; ++i)
                for(int j = 0 ; j < 3 ; ++j)
                    mat21.matrix->add(mat21.offRow + 6 * k21RowIndex + i, mat21.offCol + 3 * k21ColIndex + j, k21BlockData(i, j));
        }
    }

    delete K;
    delete mappedFFMatrix;
    delete mappedFFMatrixAccessor;
}

template<class DataTypes1, class DataTypes2>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::testBuildJacobian(const MechanicalParams* mparams)
{
    CompressedRowSparseMatrix< _3_3_Matrix_Type >* mappedFFMatrix = new CompressedRowSparseMatrix< _3_3_Matrix_Type > ( );

    sofa::core::behavior::BaseMechanicalState* mstate = d_mappedForceField.get()->getContext()->getMechanicalState();
    mappedFFMatrix->resizeBloc( mstate->getSize() ,  mstate->getSize());

    DefaultMultiMatrixAccessor* mappedFFMatrixAccessor;
    mappedFFMatrixAccessor = new DefaultMultiMatrixAccessor;

    mappedFFMatrixAccessor->addMechanicalState(  d_mappedForceField.get()->getContext()->getMechanicalState() );
    mappedFFMatrixAccessor->setGlobalMatrix(mappedFFMatrix);
    mappedFFMatrixAccessor->setupMatrices();

    //--Warning r set but unused : TODO clean or refactorize

    //MultiMatrixAccessor::MatrixRef r = mappedFFMatrixAccessor->getMatrix(  d_mappedForceField.get()->getContext()->getMechanicalState()  );

    //--

    d_mappedForceField.get()->addKToMatrix(mparams, mappedFFMatrixAccessor);

    sofa::core::State<DataTypes1> * state = dynamic_cast<sofa::core::State<DataTypes1> *>(mstate);
    unsigned int stateSize = mstate->getSize();

    sofa::core::MultiMatrixDerivId c = sofa::core::MatrixDerivId::mappingJacobian();

    DataMatrixDeriv1* out = c[state].write();

    MatrixDeriv1 * IdentityTestMatrix = out->beginEdit();

    for(unsigned int i = 0; i < stateSize; i++)
    {
        typename MatrixDeriv1::RowIterator o = IdentityTestMatrix->writeLine(3 * i);
        Deriv1 v(1, 0, 0);
        o.addCol(i, v);

        o = IdentityTestMatrix->writeLine(3 * i +1 );
        Deriv1 w(0, 1, 0);
        o.addCol(i, w);

        o = IdentityTestMatrix->writeLine(3 * i +2 );
         Deriv1 u(0, 0, 1);
         o.addCol(i, u);
    }

    out->endEdit();

    delete mappedFFMatrix;
    delete mappedFFMatrixAccessor;
}

template<class DataTypes1, class DataTypes2>
template<unsigned int M, unsigned int N, unsigned int K>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::multMatrices(const CompressedRowSparseMatrix<Mat<M, N, Real1> >& A,
                                                                           const CompressedRowSparseMatrix<Mat<N, K, Real1> >& B,
                                                                           CompressedRowSparseMatrix<Mat<M, K, Real1> >& R)
{
    size_t rBlockR = A.nBlockRow;
    size_t cBlockR = B.nBlockCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real1> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real1> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real1> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real1> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real1> AMatrixBuffer;
    Mat<N, K, Real1> BMatrixBuffer;
    for (int ArBlockId = 0; ArBlockId < A.nBlockRow; ArBlockId++)
    {// for each line block of A
        for (AColBlockIter AColIter = A.bRowBegin(ArBlockId); AColIter < A.bRowEnd(ArBlockId); AColIter++)
        {// for each column in row
            ABlockConstAccessor ABlock = AColIter.bloc(); //take non zero blocks in row

            const Mat<M, N, Real1>& ABlockData = *(const Mat<M, N, Real1>*) ABlock.elements(AMatrixBuffer.ptr()); // get the block

            int AColIndex = ABlock.getCol(); // get the block column index

            for (BColBlockIter BColIter = B.bRowBegin(AColIndex); BColIter < B.bRowEnd(AColIndex); BColIter++) {
                BBlockConstAccessor BBlock = BColIter.bloc();

                const Mat<N, K, Real1>& BBlockData = *(const Mat<N, K, Real1>*)BBlock.elements(BMatrixBuffer.ptr());

                int BColIndex = BBlock.getCol();

                Mat<M, K, Real1> RBlockData(0.0);
                //multiply the block, could be done more efficiently

                for (unsigned int i = 0; i < M ; i++)
                    for (unsigned int j = 0; j < K; j++)
                        for (unsigned int k = 0; k < N; k++)
                        { RBlockData(i, j) += ABlockData(i, k) * BBlockData(k, j); }

                R.blocAdd(ArBlockId, BColIndex, RBlockData.ptr());
            }
        }
    }
}


template<class DataTypes1, class DataTypes2>
template<unsigned int M, unsigned int N, unsigned int K>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::multMatricesT(const CompressedRowSparseMatrix<Mat<M, N, Real1> >& At,
                                                                            const CompressedRowSparseMatrix<Mat<M, K, Real1> >& B,
                                                                            CompressedRowSparseMatrix<Mat<N, K, Real1> >& R)
{
    size_t rBlockR = At.nBlockCol;
    size_t cBlockR = B.nBlockCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real1> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real1> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real1> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real1> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real1> AMatrixBuffer;
    Mat<M, K, Real1> BMatrixBuffer;

    for(size_t AtrBlockId = 0 ; AtrBlockId < (unsigned int)At.nBlockRow ; ++AtrBlockId) {
        for(AColBlockIter AtColIter = At.bRowBegin(AtrBlockId); AtColIter < At.bRowEnd(AtrBlockId) ; AtColIter++) {
            ABlockConstAccessor ABlock = AtColIter.bloc(); // is now a column block

            const Mat<M, N, Real1>& ABlockData = *(const Mat<M, N, Real1>*) ABlock.elements(AMatrixBuffer.ptr()); // get the block

            int AColIndex = ABlock.getCol(); // get the block column index

            for (BColBlockIter BColIter = B.bRowBegin(AtrBlockId); BColIter < B.bRowEnd(AtrBlockId); BColIter++)
            {
                BBlockConstAccessor BBlock = BColIter.bloc();

                const Mat<M, K, Real1>& BBlockData = *(const Mat<M, K, Real1>*)BBlock.elements(BMatrixBuffer.ptr());

                int BColIndex = BBlock.getCol();

                Mat<N, K, Real1> RBlockData(0.0);
                //multiply the block, could be done more efficiently

                for (unsigned int i = 0; i < N ; i++)
                    for (unsigned int j = 0; j < K; j++)
                        for (unsigned int k = 0; k < M; k++)
                        { RBlockData(i, j) += ABlockData(k, i) * BBlockData(k, j); }

                R.blocAdd(AColIndex, BColIndex, RBlockData.ptr());
            }

        }
    }
}


// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::addForce(const MechanicalParams* mparams,
                                                                       DataVecDeriv1& f1,
                                                                       DataVecDeriv2& f2,
                                                                       const DataVecCoord1& x1,
                                                                       const DataVecCoord2& x2,
                                                                       const DataVecDeriv1& v1,
                                                                       const DataVecDeriv2& v2)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(f1);
    SOFA_UNUSED(f2);
    SOFA_UNUSED(x1);
    SOFA_UNUSED(x2);
    SOFA_UNUSED(v1);
    SOFA_UNUSED(v2);
}

// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
void PartialRigidificationForceField<DataTypes1, DataTypes2>::addDForce(const MechanicalParams* mparams,
                                                                        DataVecDeriv1& df1,
                                                                        DataVecDeriv2& df2,
                                                                        const DataVecDeriv1& dx1,
                                                                        const DataVecDeriv2& dx2)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(df1);
    SOFA_UNUSED(df2);
    SOFA_UNUSED(dx1);
    SOFA_UNUSED(dx2);
}

// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
double PartialRigidificationForceField<DataTypes1, DataTypes2>::getPotentialEnergy(const MechanicalParams* mparams,
                                                                                   const DataVecCoord1& x1,
                                                                                   const DataVecCoord2& x2) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x1);
    SOFA_UNUSED(x2);

    return 0.0;
}

} // namespace

