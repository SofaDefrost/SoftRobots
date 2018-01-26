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
#ifndef SOFA_COMPONENT_FORCEFIELD_MappedMatrixForceField_INL
#define SOFA_COMPONENT_FORCEFIELD_MappedMatrixForceField_INL


#include "MappedMatrixForceField.h"
#include <sofa/core/visual/VisualParams.h>
#include <sofa/helper/gl/template.h>
#include <sofa/helper/system/config.h>
#include <sofa/helper/system/glut.h>
#include <sofa/helper/rmath.h>
#include <assert.h>
#include <iostream>
#include <fstream>

// accumulate jacobian
#include <sofa/core/ExecParams.h>
#include <sofa/core/objectmodel/BaseContext.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/defaulttype/MapMapSparseMatrix.h>

// verify timing
#include <sofa/helper/system/thread/CTime.h>

namespace sofa
{

namespace component
{

namespace interactionforcefield
{


using sofa::component::linearsolver::DefaultMultiMatrixAccessor ;
using sofa::core::behavior::BaseMechanicalState ;


template<class DataTypes1, class DataTypes2>
MappedMatrixForceField<DataTypes1, DataTypes2>::MappedMatrixForceField()
    :
      d_mappedForceField(initLink("mappedForceField",
                                  "link to the forcefield that is mapped under the subsetMultiMapping")),
      d_mappedForceField2(initLink("mappedForceField2",
                                   "link to an other forcefield defined at the same node than mappedForceField")),
      listActiveNodesPath(initData(&listActiveNodesPath,"listActiveNodesPath","Path to the list of active nodes when performing the ECSW method")),
      performECSW(initData(&performECSW,false,"performECSW","Use the reduced model with the ECSW method"))

{
}

template<class DataTypes1, class DataTypes2>
void MappedMatrixForceField<DataTypes1, DataTypes2>::init()
{
    sofa::core::behavior::BaseInteractionForceField::init();

    if (mstate1.get() == NULL || mstate2.get() == NULL)
    {
        serr<< "Init of MixedInteractionForceField " << getContext()->getName() << " failed!" << sendl;
        //getContext()->removeObject(this);
        return;
    }

    if (performECSW.getValue())
    {
        listActiveNodes.resize(0);
        std::ifstream listActiveNodesFile(listActiveNodesPath.getValue(), std::ios::in);
        //nbLine = 0;
        std::string lineValues;  // déclaration d'une chaîne qui contiendra la ligne lue
        while (getline(listActiveNodesFile, lineValues))
        {
            listActiveNodes.push_back(std::stoi(lineValues));
            //nbLine++;
        }
        listActiveNodesFile.close();
        std::cout << "list of Active nodes : " << listActiveNodes << std::endl;
    }

}

template<class DataTypes1, class DataTypes2>
void MappedMatrixForceField<DataTypes1, DataTypes2>::accumulateJacobians(const MechanicalParams* mparams)
{

    // STEP1 : accumulate Jacobians J1 and J2

    const core::ExecParams* eparams = dynamic_cast<const core::ExecParams *>( mparams );
    core::ConstraintParams cparams = core::ConstraintParams(*eparams);

    core::behavior::BaseMechanicalState* mstate = d_mappedForceField.get()->getContext()->getMechanicalState();


    sofa::helper::vector<unsigned int> list;
    if (!performECSW.getValue())
    {
        std::cout << "mstate->getSize()" << mstate->getSize() << std::endl;
        for (unsigned int i=0; i<mstate->getSize(); i++)
            list.push_back(i);
    }


    sofa::core::MatrixDerivId Id= sofa::core::MatrixDerivId::mappingJacobian();

    core::objectmodel::BaseContext* context = this->getContext();
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
    simulation::MechanicalResetConstraintVisitor(eparams).execute(gnode);

    if (performECSW.getValue())
    {
        mstate->buildIdentityBlocksInJacobian(listActiveNodes, Id);
    }
    else
    {
        mstate->buildIdentityBlocksInJacobian(list, Id);
    }
    MechanicalAccumulateJacobian(&cparams, core::MatrixDerivId::mappingJacobian()).execute(gnode);



 /* From uncoupled constraint correction

    for (MatrixDerivRowConstIterator rowIt = constraints.begin(), rowItEnd = constraints.end(); rowIt != rowItEnd; ++rowIt)
    {
        int indexCurRowConst = rowIt.index();

        if (f_verbose.getValue())
            sout << "C[" << indexCurRowConst << "]";

        for (MatrixDerivColConstIterator colIt = rowIt.begin(), colItEnd = rowIt.end(); colIt != colItEnd; ++colIt)
        {
            unsigned int dof = colIt.index();
            Deriv n = colIt.val();

            */




/*
    sofa::core::State<DataTypes1> * state1 = dynamic_cast<sofa::core::State<DataTypes1> *>(mstate);
    unsigned int stateSize = mstate->getSize();



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

*/
















    /*
    MultiMatrixDerivId cId = core::MatrixDerivId::mappingMatrix();
    buildConstraintMatrix(cParams, *cId[mstate].write(), cIndex, *cParams->readX(mstate));

// calling applyConstraint on each constraint

MechanicalSetConstraint(&cparams, core::MatrixDerivId::mappingMatrix(), numConstraints).execute(context);

sofa::helper::AdvancedTimer::valSet("numConstraints", numConstraints);

// calling accumulateConstraint on the mappings
MechanicalAccumulateConstraint2(&cparams, core::MatrixDerivId::mappingMatrix()).execute(context);

*/

/* /////////////////////////////////////////// GARBAGE //////////////////////////////////////////////


        /*

          ////////////
          // look if the row corresponds to rows in J1 and/or in J2
          // ALTERNATIVE IMPLEMENTATION with linear complexity (no use of funtion readline that leads to n log n complexity)
          // (we suppose that rows of J1 and J2 are sorted in ascending order)
          // problem complex impl
          ////////////

        bool j1 = false;
        bool j2 = false;

        int indexRow1 = rowItJ1.index();
        int indexRow2 = rowItJ2.index();



        //std::cout <<"indexRow1 = "<<indexRow1<<"  indexRow2 = "<<indexRow2<<std::endl;
        while (indexRow1<row)
        {
            if(rowItJ1 != J1.end())
                rowItJ1++;
            indexRow1 = rowItJ1.index();
        }
        while (indexRow2<row)
        {
            if(rowItJ2 != J2.end())
                rowItJ2++;
            indexRow2 = rowItJ2.index();
        }

        if (indexRow1 == row)
            j1=true;
        if (indexRow2 == row)
            j2=true;



        // if no correspondance with J1 nor J2 go to the following row
        if (!j1 && !j2)
            continue;


      //print verificaction
        std::cout<<"multiplies K row "<<row<<" with";
        if (j1)
            std::cout<<" J1 row "<<indexRow1<<std::endl;
        if (j2)
            std::cout<<" J2 row "<<indexRow2<<std::endl;

            */





}



template<class DataTypes1, class DataTypes2>
void MappedMatrixForceField<DataTypes1, DataTypes2>::addKToMatrix(const MechanicalParams* mparams,
                                                                           const MultiMatrixAccessor* matrix)
{

    sofa::helper::system::thread::CTime *timer;
    double timeScale, time ;
    timeScale = 1000.0 / (double)sofa::helper::system::thread::CTime::getTicksPerSec();

    time = (double)timer->getTime();

    if(f_printLog.getValue())
        sout << "entering addKToMatrix" << sendl;


    sofa::core::behavior::MechanicalState<DataTypes1>* ms1 = this->getMState1();
    sofa::core::behavior::MechanicalState<DataTypes2>* ms2 = this->getMState2();

    sofa::core::behavior::BaseMechanicalState*  bms1 = this->getMechModel1();
    sofa::core::behavior::BaseMechanicalState*  bms2 = this->getMechModel2();

    MultiMatrixAccessor::MatrixRef mat11 = matrix->getMatrix(mstate1);
    MultiMatrixAccessor::MatrixRef mat22 = matrix->getMatrix(mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat12 = matrix->getMatrix(mstate1, mstate2);
    MultiMatrixAccessor::InteractionMatrixRef mat21 = matrix->getMatrix(mstate2, mstate1);


    //////////////////////////////////////// NEW VERSION ////////////////////////////////////////

    // STEP 1 compute jacobians using generic implementation
    this->accumulateJacobians(mparams);
    std::cout<<" accumulate J : "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();


    // STEP2 compute the stiffness K of the forcefield and put it in a rowsparseMatrix
    // get the stiffness matrix from the mapped ForceField
    // TODO: use the template of the FF for Real
    core::behavior::BaseMechanicalState* mstate = d_mappedForceField.get()->getContext()->getMechanicalState();
    CompressedRowSparseMatrix< Real1 >* K = new CompressedRowSparseMatrix< Real1 > ( );
    // TODO:
    K->resizeBloc( 3*mstate->getSize() ,  3*mstate->getSize());
    K->clear();
    DefaultMultiMatrixAccessor* KAccessor;
    KAccessor = new DefaultMultiMatrixAccessor;
    KAccessor->addMechanicalState(  d_mappedForceField.get()->getContext()->getMechanicalState() );
    KAccessor->setGlobalMatrix(K);
    KAccessor->setupMatrices();
    std::cout<<" time get K : "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();
    d_mappedForceField.get()->addKToMatrix(mparams, KAccessor);
    d_mappedForceField2.get()->addKToMatrix(mparams, KAccessor);
    std::cout<<" time addKtoMatrix K : "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();

    if (!K)
    {
        std::cout<<"matrix of the force-field system not found"<<std::endl;
        return;
    }
    K->compress();

    std::cout<<" time compress K : "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();


    //std::cout<< "+++++++++++++++++++++++++++++++++++++"<<std::endl;
    //std::cout<< "matrix ="<< (*K)<<std::endl;
    //std::cout<< "+++++++++++++++++++++++++++++++++++++"<<std::endl;

    // we have the K matrix from the mappedForceField in compressed row sparse format

    // STEP3: we now get the matrices J1 and J2
    sofa::core::MultiMatrixDerivId c = sofa::core::MatrixDerivId::mappingJacobian();
    const MatrixDeriv1 &J1 = c[ms1].read()->getValue();
    MatrixDeriv1RowConstIterator rowItJ1= J1.begin();
    const MatrixDeriv2 &J2 = c[ms2].read()->getValue();
    MatrixDeriv2RowConstIterator rowItJ2= J2.begin();


    // STEP4: perform the multiplication with [J1t J2t] * K * [J1 J2]

    std::cout<<" time get J : "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();
//    std::cout<<" nRow "<< K->nRow << "nCol" << K->nCol <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<" nBlocRow "<< K->nBlocRow << std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<"nBlocCol" << K->nBlocCol <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<"rowIndex.size" << K->rowIndex.size() <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<"rowBegin.size" << K->rowBegin.size() <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<" rowIndex "<< K->rowIndex <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<" rowBegin" << K->rowBegin <<std::endl;
//    std::cout<<" ++++++++++++++++++++++++++++++++++++++++++++++++ " <<std::endl;
//    std::cout<<" colsIndex "<< K->colsIndex <<std::endl;//<< "colsValue" << K->colsValue <<std::endl;



    for (unsigned int it_rows_k=0; it_rows_k < K->rowIndex.size() ; it_rows_k ++)
    {

        Index row = K->rowIndex[it_rows_k] ;
//        std::cout << "row is " << row << " of " << K->rowIndex[K->rowIndex.size()-1] << " ****************************** " << std::endl;
        // look if the row corresponds to rows in J1 and/or in J2
        // we know that rows of J1 and J2 are sorted in ascending order
        bool j1 = false;
        bool j2 = false;
        rowItJ1 = J1.readLine(row);
        if(rowItJ1 != J1.end())
            j1=true; // matrix J1 contains line number "row"
        rowItJ2 = J2.readLine(row);
        if(rowItJ2 != J2.end())
            j2=true; // matrix J2 contains line number "row"
        // if no correspondance with J1 nor J2 go to the following row
        if (!j1 && !j2)
            continue;

        // row of matrix J corresponds to columns of matrix J transpose
        MatrixDeriv1RowConstIterator colItJ1t= J1.begin();
        MatrixDeriv2RowConstIterator colItJ2t= J2.begin();


        Range rowRange( K->rowBegin[it_rows_k], K->rowBegin[it_rows_k+1] );
        for( Index xj = rowRange.begin() ; xj < rowRange.end() ; xj++ )  // for each non-null block
        {
            Index col = K->colsIndex[xj];     // block column
//            std::cout << "col " << col << " of " << K->colsIndex[rowRange.end()-1]<< std::endl;
            // we look if this column exists in J1t or J2t
            // rmq !! use of  function readLine: log(n) coplexity !!
            bool j1t = false;
            bool j2t = false;
            colItJ1t = J1.readLine(col); // reading the line of J1 corresponds to the col of J1t
            //std::cout << "colItJ1t" << colItJ1t.begin()<< " " <<colItJ1t.end() << std::endl;
            if(colItJ1t != J1.end())
                j1t=true; // matrix J1t contains the column  number "col"
            colItJ2t = J2.readLine(col); // reading the line of J2 corresponds to the col of J1t
            if(colItJ2t != J2.end())
                j2t=true; // matrix J2t contains the column number "col"
            // if no correspondance with Jt1 nor J2t go to the following col
            if (!j1t && !j2t)
                continue;


            const Real1& k = K->colsValue[xj]; // non-null element of the matrix

            unsigned int line=0;
            unsigned int column=0;

            // compute the multiplication with the corresponding block value of J k Jt
            if (j1)
            {
                for (MatrixDeriv1ColConstIterator colItJ1 = rowItJ1.begin(), colItJ1End = rowItJ1.end(); colItJ1 != colItJ1End; ++colItJ1)
                {
                    unsigned int dofJ1 = colItJ1.index();
                    Deriv1 j1_d = colItJ1.val();


                    if(j1t)
                    {
                        line = mat11.offset + DerivSize1 * dofJ1;

                        for (MatrixDeriv1ColConstIterator rowItJ1t = colItJ1t.begin(), rowItJ1End = colItJ1t.end(); rowItJ1t != rowItJ1End; ++rowItJ1t)
                        {
                            unsigned int dofJ1t = rowItJ1t.index();
                            Deriv1 j1t_d = rowItJ1t.val();

                            column = mat11.offset + DerivSize1 * dofJ1t;
             //               std::cout << "About to add in row offset " << line << " and col offset " << column << std::endl;
                            for (unsigned int i=0; i<DerivSize1; i++)
                            {
                                if(j1_d[i]==(Real1)0.0)
                                    continue;

                                for (unsigned int j=0; j<DerivSize1; j++)
                                {
                                    if(j1t_d[j]==(Real1)0.0)
                                        continue;
//                                    std::cout << "About to add " << j1_d[i]*k*j1t_d[j] << "in row " << line +i << "and col " << column + j << std::endl;
                                    mat11.matrix->add(line +i, column + j, j1_d[i]*k*j1t_d[j] );
                                }

                            }
                        }
                    }

                    if(bms1== bms2)
                        continue;


                    if(j2t)
                    {
                        line = mat12.offRow + DerivSize1 * dofJ1;
                        for (MatrixDeriv2ColConstIterator rowItJ2t = colItJ2t.begin(), rowItJ2End = colItJ2t.end(); rowItJ2t != rowItJ2End; ++rowItJ2t)
                        {
                            unsigned int dofJ2t = rowItJ2t.index();
                            Deriv2 j2t_d = rowItJ2t.val();
                            column = mat12.offCol + DerivSize2 * dofJ2t;

                            for (unsigned int i=0; i<DerivSize1; i++)
                            {
                                if(j1_d[i]==(Real1)0.0)
                                    continue;

                                for (unsigned int j=0; j<DerivSize2; j++)
                                {
                                    if(j2t_d[j]==(Real2)0.0)
                                        continue;
                                    mat12.matrix->add(line + i, column + j, j1_d[i]*k*j2t_d[j] );
                                }

                            }
                        }
                    }
                }
            }
            if(bms1== bms2)
                continue;

            if (j2)
            {

                for (MatrixDeriv2ColConstIterator colItJ2 = rowItJ2.begin(), colItJ2End = rowItJ2.end(); colItJ2 != colItJ2End; ++colItJ2)
                {
                    unsigned int dofJ2 = colItJ2.index();
                    Deriv2 j2_d = colItJ2.val();

                    if(j1t)
                    {
                        line = mat21.offRow + DerivSize2 * dofJ2;
                        for (MatrixDeriv1ColConstIterator rowItJ1t = colItJ1t.begin(), rowItJ1End = colItJ1t.end(); rowItJ1t != rowItJ1End; ++rowItJ1t)
                        {
                            unsigned int dofJ1t = rowItJ1t.index();
                            Deriv1 j1t_d = rowItJ1t.val();
                            column = mat21.offCol + DerivSize1 * dofJ1t;

                            for (unsigned int i=0; i<DerivSize2; i++)
                            {
                                if(j2_d[i]==(Real2)0.0)
                                    continue;

                                for (unsigned int j=0; j<DerivSize1; j++)
                                {
                                    if(j1t_d[j]==(Real1)0.0)
                                        continue;
                                    mat21.matrix->add(line + i, column + j, j2_d[i]*k*j1t_d[j] );
                                }

                            }
                        }
                    }


                    if(j2t)
                    {
                        line = mat22.offset + DerivSize2 * dofJ2 ;
                        for (MatrixDeriv2ColConstIterator rowItJ2t = colItJ2t.begin(), rowItJ2End = colItJ2t.end(); rowItJ2t != rowItJ2End; ++rowItJ2t)
                        {
                            unsigned int dofJ2t = rowItJ2t.index();
                            Deriv2 j2t_d = rowItJ2t.val();
                            column = mat22.offset + DerivSize2 * dofJ2t;

                            for (unsigned int i=0; i<DerivSize2; i++)
                            {
                                if(j2_d[i]==(Real2)0.0)
                                    continue;

                                for (unsigned int j=0; j<DerivSize2; j++)
                                {
                                    if(j2t_d[j]==(Real2)0.0)
                                        continue;
                                    mat22.matrix->add(line + i, column + j, j2_d[i]*k*j2t_d[j] );
                                }

                            }
                        }
                    }
                }

            }
        }
    }

    std::cout<<" time compute J() * K * J: "<<( (double)timer->getTime() - time)*timeScale<<" ms"<<std::endl;
    time= (double)timer->getTime();

    //std::cout<<"matrix11"<<(*mat11.matrix)<<std::endl;

    delete KAccessor;
    delete K;


    if(f_printLog.getValue())
        sout << "exit addKToMatrix" << sendl;


    const core::ExecParams* eparams = dynamic_cast<const core::ExecParams *>( mparams );
    core::ConstraintParams cparams = core::ConstraintParams(*eparams);

    core::objectmodel::BaseContext* context = this->getContext();
    simulation::Node* gnode = dynamic_cast<simulation::Node*>(context);
    simulation::MechanicalResetConstraintVisitor(eparams).execute(gnode);

}

template<class DataTypes1, class DataTypes2>
void MappedMatrixForceField<DataTypes1, DataTypes2>::testBuildJacobian(const MechanicalParams* /*mparams*/)
{

    /*  TODO => ADAPT
    CompressedRowSparseMatrix< _3_3_Matrix_Type >* mappedFFMatrix = new CompressedRowSparseMatrix< _3_3_Matrix_Type > ( );

    core::behavior::BaseMechanicalState* mstate = d_mappedForceField.get()->getContext()->getMechanicalState();
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

    // CompressedRowSparseMatrix<_3_6_Matrix_Type> J1Jr ;
    // CompressedRowSparseMatrix<_3_3_Matrix_Type> J0tK ;
    // CompressedRowSparseMatrix<_3_3_Matrix_Type> J0tKJ0 ;
    // CompressedRowSparseMatrix<_3_6_Matrix_Type> J0tKJ1Jr ;
    // CompressedRowSparseMatrix<_6_6_Matrix_Type> JrtJ1tKJ1Jr ;
    // CompressedRowSparseMatrix<_6_3_Matrix_Type> JrtJ1tK ;
    // CompressedRowSparseMatrix<_6_3_Matrix_Type> JrtJ1tKJ0 ;

    //--Warning K set but unused : TODO clean or refactorize

    //CompressedRowSparseMatrix<_3_3_Matrix_Type>* K ;
    //K = dynamic_cast<CompressedRowSparseMatrix<_3_3_Matrix_Type>*>(r.matrix);

    //--

    sofa::core::State<DataTypes1> * state = dynamic_cast<sofa::core::State<DataTypes1> *>(mstate);
    unsigned int stateSize = mstate->getSize();

    sofa::core::MultiMatrixDerivId c = sofa::core::MatrixDerivId::nonHolonomicC();

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

    */
}



// Even though it does nothing, this method has to be implemented
// since it's a pure virtual in parent class
template<class DataTypes1, class DataTypes2>
void MappedMatrixForceField<DataTypes1, DataTypes2>::addForce(const MechanicalParams* mparams,
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
void MappedMatrixForceField<DataTypes1, DataTypes2>::addDForce(const MechanicalParams* mparams,
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
double MappedMatrixForceField<DataTypes1, DataTypes2>::getPotentialEnergy(const MechanicalParams* mparams,
                                                                                   const DataVecCoord1& x1,
                                                                                   const DataVecCoord2& x2) const
{
    SOFA_UNUSED(mparams);
    SOFA_UNUSED(x1);
    SOFA_UNUSED(x2);

    return 0.0;
}

} // namespace forcefield

} // namespace component

} // namespace sofa

#endif
