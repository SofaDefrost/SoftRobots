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
#ifndef SOFA_COMPONENT_FORCEFIELD_PIPEFORCEFIELD_INL
#define SOFA_COMPONENT_FORCEFIELD_PIPEFORCEFIELD_INL

#include "PipeForceField.h"
#include <SofaBaseLinearSolver/FullVector.h>
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

namespace sofa
{

namespace component
{

namespace forcefield
{

using sofa::component::linearsolver::DefaultMultiMatrixAccessor ;
using sofa::core::behavior::MultiMatrixAccessor ;
using sofa::core::behavior::BaseMechanicalState ;

template<typename DataTypes>
PipeForceField<DataTypes>::PipeForceField()
    : Inherit(),
      d_barycentricMapping(initLink("barycentricMapping", " link to the BarycentricMapping")),
      d_mappedForceField(initLink("mappedForceField", "link to the MeshSpringForceField"))
{}

template<typename DataTypes>
PipeForceField<DataTypes>::~PipeForceField()
{}

template<typename DataTypes>
void PipeForceField<DataTypes>::init()
{
    Inherit::init();
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
    if (d_barycentricMapping.get() == NULL )
    {
        msg_error() << "Mapping is missing. AddKToMatrix not computed";
        return;
    }
    if (d_mappedForceField.get() == NULL )
    {
        msg_error() << "MappedFF is missing. AddKToMatrix not computed";
        return;
    }


    //Get the jacobian matrix from the BarycentricMapping
    const CompressedRowSparseMatrix<_3_3_Matrix_Type>* J;
    J = dynamic_cast<const CompressedRowSparseMatrix<_3_3_Matrix_Type> *> (d_barycentricMapping.get()->getJ() );

    if (J == NULL )
    {
        msg_error() << "Jacobian matrix from Mapping is missing. AddKToMatrix not computed";
        return;
    }


    //Get the stiffness matrix from the mapped ForceField
    CompressedRowSparseMatrix< _3_3_Matrix_Type >* mappedFFMatrix = new CompressedRowSparseMatrix< _3_3_Matrix_Type > ( );
    BaseMechanicalState* springState = d_mappedForceField.get()->getContext()->getMechanicalState();

    if (springState == NULL )
    {
        msg_error() << "Mstate from mappedFF is missing. AddKToMatrix not computed";
        delete mappedFFMatrix;
        return;
    }

    DefaultMultiMatrixAccessor* mappedFFMatrixAccessor = new DefaultMultiMatrixAccessor;
    mappedFFMatrixAccessor->addMechanicalState(springState);
    mappedFFMatrix->resizeBloc(springState->getSize(),springState->getSize());
    mappedFFMatrixAccessor->setGlobalMatrix(mappedFFMatrix);
    mappedFFMatrixAccessor->setupMatrices();

    MultiMatrixAccessor::MatrixRef r = mappedFFMatrixAccessor->getMatrix(springState);

    d_mappedForceField.get()->addKToMatrix(mparams, mappedFFMatrixAccessor);
    CompressedRowSparseMatrix<_3_3_Matrix_Type> JtK ;
    CompressedRowSparseMatrix<_3_3_Matrix_Type> JtKJ ;

    CompressedRowSparseMatrix<_3_3_Matrix_Type>* K ;
    K = dynamic_cast<CompressedRowSparseMatrix<_3_3_Matrix_Type>*>(r.matrix);
    multMatricesT<3, 3, 3>(*J, *K, JtK);
    multMatrices<3, 3, 3>(JtK, *J, JtKJ);


    //Add JtKJ to global system
    MultiMatrixAccessor::MatrixRef mat = matrix->getMatrix(mstate);
    _3_3_Matrix_Type KMatrixBuffer;
    for(size_t kRowIndex = 0 ; kRowIndex < (unsigned int)JtKJ.nBlocRow ; ++kRowIndex) {
        for(_3_3_ColBlockConstIterator kColIter = JtKJ.bRowBegin(kRowIndex); kColIter < JtKJ.bRowEnd(kRowIndex) ; kColIter++) {

            _3_3_BlockConstAccessor kBlock = kColIter.bloc();

            const _3_3_Matrix_Type& kBlockData = *(const _3_3_Matrix_Type*) kBlock.elements(KMatrixBuffer.ptr()); //Get the block

            int kColIndex = kBlock.getCol(); //Get the block column index

            for (int i = 0; i < 3 ; i++)
                for (int j = 0; j < 3; j++)
                    mat.matrix->add(mat.offset + 3 * kRowIndex + i,mat.offset + 3 * kColIndex + j, kBlockData(i, j));
        }
    }

    delete K;
    delete mappedFFMatrix;
    delete mappedFFMatrixAccessor;
}


template<class DataTypes>
template<unsigned int M, unsigned int N, unsigned int K>
void PipeForceField<DataTypes>::multMatrices(const CompressedRowSparseMatrix<Mat<M, N, Real> >& A,
                                             const CompressedRowSparseMatrix<Mat<N, K, Real> >& B,
                                             CompressedRowSparseMatrix<Mat<M, K, Real> >& R)
{
    size_t rBlockR = A.nBlocRow;
    size_t cBlockR = B.nBlocCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<N, K, Real> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real> AMatrixBuffer;
    Mat<N, K, Real> BMatrixBuffer;
    for (int ArBlockId = 0; ArBlockId < A.nBlocRow; ArBlockId++) {  //For each line block of A
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
    size_t rBlockR = At.nBlocCol;
    size_t cBlockR = B.nBlocCol;

    R.resizeBloc(rBlockR, cBlockR);

    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::ColBlockConstIterator AColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, N, Real> >::BlockConstAccessor ABlockConstAccessor;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real> >::ColBlockConstIterator BColBlockIter;
    typedef typename CompressedRowSparseMatrix<Mat<M, K, Real> >::BlockConstAccessor BBlockConstAccessor;

    Mat<M, N, Real> AMatrixBuffer;
    Mat<M, K, Real> BMatrixBuffer;

    for(size_t AtrBlockId = 0 ; AtrBlockId < (unsigned int)At.nBlocRow ; ++AtrBlockId) {
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

} // forcefield
} // component
} // sofa

#endif // SOFA_COMPONENT_FORCEFIELD_PIPEFORCEFIELD_INL
