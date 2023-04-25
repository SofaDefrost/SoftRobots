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

#include <SoftRobots/component/forcefield/PipeForceField.h>

#include <sofa/linearalgebra/FullVector.h>
#include <sofa/core/behavior/MechanicalState.h>
using sofa::core::behavior::MechanicalState ;
using sofa::core::objectmodel::BaseContext ;

#include <sofa/core/behavior/ForceField.inl>
#include <sofa/helper/RandomGenerator.h>
using sofa::helper::ReadAccessor ;
using sofa::helper::WriteAccessor ;

#include <string>
using std::string ;

#include <fstream>
using std::filebuf ;

#include <iostream>
using std::cout ;
using std::endl ;

#include <algorithm>
#include <ctime>

namespace softrobots::forcefield
{

using sofa::core::behavior::DefaultMultiMatrixAccessor ;
using sofa::core::behavior::MultiMatrixAccessor ;
using sofa::core::behavior::BaseMechanicalState ;

template<typename DataTypes>
PipeForceField<DataTypes>::PipeForceField()
    : Inherit1(),
      l_barycentricMapping(initLink("barycentricMapping", " link to the BarycentricMapping")),
      l_mappedForceField(initLink("mappedForceField", "link to the MeshSpringForceField"))
{}

template<typename DataTypes>
PipeForceField<DataTypes>::~PipeForceField()
{}

template<typename DataTypes>
void PipeForceField<DataTypes>::init()
{
    Inherit1::init();
}

template<typename DataTypes>
void PipeForceField<DataTypes>::addForce(const MechanicalParams* mparams,
                                         DataVecDeriv& f,
                                         const DataVecCoord& x,
                                         const DataVecDeriv& v)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(f);
    SOFA_UNUSED(x);
    SOFA_UNUSED(v);
}

template<typename DataTypes>
void PipeForceField<DataTypes>::addDForce(const MechanicalParams* mparams,
                                          DataVecDeriv&  d_f ,
                                          const DataVecDeriv&  d_x)
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(d_f);
    SOFA_UNUSED(d_x);
}

template<typename DataTypes>
double PipeForceField<DataTypes>::getPotentialEnergy(const MechanicalParams* mparams,
                                                     const DataVecCoord& x) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x);

    return 0.0;
}

template<typename DataTypes>
void PipeForceField<DataTypes>::addKToMatrix(const MechanicalParams* mparams,
                                               const MultiMatrixAccessor* matrix)
{
    if (l_barycentricMapping.get() == nullptr )
    {
        msg_error() << "Link to barycentric mapping is missing or incorrect. AddKToMatrix not computed.";
        return;
    }
    if (l_mappedForceField.get() == nullptr )
    {
        msg_error() << "Link to mapped force field is missing or incorrect. AddKToMatrix not computed.";
        return;
    }


    //Get the jacobian matrix from the BarycentricMapping
    const CompressedRowSparseMatrix<_3_3_Matrix_Type>* J;
    J = dynamic_cast<const CompressedRowSparseMatrix<_3_3_Matrix_Type> *> (l_barycentricMapping.get()->getJ() );

    if (J == nullptr )
    {
        msg_error() << "Jacobian matrix from barycentric mapping is missing. AddKToMatrix not computed.";
        return;
    }


    //Get the stiffness matrix from the mapped ForceField
    CompressedRowSparseMatrix< _3_3_Matrix_Type >* mappedFFMatrix = new CompressedRowSparseMatrix< _3_3_Matrix_Type > ( );
    BaseMechanicalState* springState = l_mappedForceField.get()->getContext()->getMechanicalState();

    if (springState == nullptr )
    {
        msg_error() << "Mstate from mappedFF is missing. AddKToMatrix not computed.";
        delete mappedFFMatrix;
        return;
    }

    DefaultMultiMatrixAccessor* mappedFFMatrixAccessor = new DefaultMultiMatrixAccessor;
    mappedFFMatrixAccessor->addMechanicalState(springState);
    mappedFFMatrix->resizeBloc(springState->getSize(),springState->getSize());
    mappedFFMatrixAccessor->setGlobalMatrix(mappedFFMatrix);
    mappedFFMatrixAccessor->setupMatrices();

    MultiMatrixAccessor::MatrixRef mappedMatrixRef = mappedFFMatrixAccessor->getMatrix(springState);

    l_mappedForceField.get()->addKToMatrix(mparams, mappedFFMatrixAccessor);
    CompressedRowSparseMatrix<_3_3_Matrix_Type> JtK ;
    CompressedRowSparseMatrix<_3_3_Matrix_Type> JtKJ ;

    CompressedRowSparseMatrix<_3_3_Matrix_Type>* K ;
    K = dynamic_cast<CompressedRowSparseMatrix<_3_3_Matrix_Type>*>(mappedMatrixRef.matrix);
    multMatricesT<3, 3, 3>(*J, *K, JtK);
    multMatrices<3, 3, 3>(JtK, *J, JtKJ);

    //Add JtKJ to global system
    if(mstate==nullptr)
        msg_error() << "Missing context mechanical state.";

    MultiMatrixAccessor::MatrixRef matrixRef = matrix->getMatrix(mstate);
    _3_3_Matrix_Type KMatrixBuffer;
    for(size_t kRowIndex = 0 ; kRowIndex < (unsigned int)JtKJ.nBlockRow ; ++kRowIndex) {
        for(_3_3_ColBlockConstIterator kColIter = JtKJ.bRowBegin(kRowIndex); kColIter < JtKJ.bRowEnd(kRowIndex) ; kColIter++) {

            _3_3_BlockConstAccessor kBlock = kColIter.bloc();

            const _3_3_Matrix_Type& kBlockData = *(const _3_3_Matrix_Type*) kBlock.elements(KMatrixBuffer.ptr()); //Get the block

            int kColIndex = kBlock.getCol(); //Get the block column index

            for (int i = 0; i < 3 ; i++)
                for (int j = 0; j < 3; j++)
                    matrixRef.matrix->add(matrixRef.offset + 3 * kRowIndex + i,matrixRef.offset + 3 * kColIndex + j, kBlockData(i, j));
        }
    }

    delete mappedFFMatrix;
    delete mappedFFMatrixAccessor;
}


template<class DataTypes>
template<unsigned int M, unsigned int N, unsigned int K>
void PipeForceField<DataTypes>::multMatrices(const CompressedRowSparseMatrix<Mat<M, N, Real> >& A,
                                             const CompressedRowSparseMatrix<Mat<N, K, Real> >& B,
                                             CompressedRowSparseMatrix<Mat<M, K, Real> >& R)
{
    size_t rBlockR = A.nBlockRow;
    size_t cBlockR = B.nBlockCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real> AMatrixBuffer;
    Mat<N, K, Real> BMatrixBuffer;
    for (int ArBlockId = 0; ArBlockId < A.nBlockRow; ArBlockId++) {  //For each line block of A
        for (AColBlockIter AColIter = A.bRowBegin(ArBlockId); AColIter < A.bRowEnd(ArBlockId); AColIter++) { //For each column in row
            ABlockConstAccessor ABlock = AColIter.bloc(); //Take non zero blocks in row

            const Mat<M, N, Real>& ABlockData = *(const Mat<M, N, Real>*) ABlock.elements(AMatrixBuffer.ptr()); //Get the block

            int AColIndex = ABlock.getCol(); //Get the block column index

            for (BColBlockIter BColIter = B.bRowBegin(AColIndex); BColIter < B.bRowEnd(AColIndex); BColIter++) {
                BBlockConstAccessor BBlock = BColIter.bloc();

                const Mat<N, K, Real>& BBlockData = *(const Mat<N, K, Real>*)BBlock.elements(BMatrixBuffer.ptr());

                int BColIndex = BBlock.getCol();

                Mat<M, K, Real> RBlockData(0.0);
                //Multiply the block, could be done more efficiently

                for (unsigned int i = 0; i < M ; i++)
                    for (unsigned int j = 0; j < K; j++)
                        for (unsigned int k = 0; k < N; k++)
                        { RBlockData(i, j) += ABlockData(i, k) * BBlockData(k, j); }

                R.blocAdd(ArBlockId, BColIndex, RBlockData.ptr());
            }
        }
    }
}


template<class DataTypes>
template<unsigned int M, unsigned int N, unsigned int K>
void PipeForceField<DataTypes>::multMatricesT(const CompressedRowSparseMatrix<Mat<M, N, Real> >& At,
                                              const CompressedRowSparseMatrix<Mat<M, K, Real> >& B,
                                              CompressedRowSparseMatrix<Mat<N, K, Real> >& R)
{
    size_t rBlockR = At.nBlockCol;
    size_t cBlockR = B.nBlockCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real> AMatrixBuffer;
    Mat<M, K, Real> BMatrixBuffer;

    for(size_t AtrBlockId = 0 ; AtrBlockId < (unsigned int)At.nBlockRow ; ++AtrBlockId) {
        for(AColBlockIter AtColIter = At.bRowBegin(AtrBlockId); AtColIter < At.bRowEnd(AtrBlockId) ; AtColIter++) {
            ABlockConstAccessor ABlock = AtColIter.bloc(); //Is now a column block

            const Mat<M, N, Real>& ABlockData = *(const Mat<M, N, Real>*) ABlock.elements(AMatrixBuffer.ptr()); //Get the block

            int AColIndex = ABlock.getCol(); //Get the block column index

            for (BColBlockIter BColIter = B.bRowBegin(AtrBlockId); BColIter < B.bRowEnd(AtrBlockId); BColIter++) { //Modifier
                BBlockConstAccessor BBlock = BColIter.bloc();

                const Mat<M, K, Real>& BBlockData = *(const Mat<M, K, Real>*)BBlock.elements(BMatrixBuffer.ptr());

                int BColIndex = BBlock.getCol();

                Mat<N, K, Real> RBlockData(0.0);
                //Multiply the block, could be done more efficiently

                for (unsigned int i = 0; i < N ; i++)
                    for (unsigned int j = 0; j < K; j++)
                        for (unsigned int k = 0; k < M; k++)
                        { RBlockData(i, j) += ABlockData(k, i) * BBlockData(k, j); }

                R.blocAdd(AColIndex, BColIndex, RBlockData.ptr());
            }

        }
    }
}

} // namespace

