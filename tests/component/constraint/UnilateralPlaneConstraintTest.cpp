/******************************************************************************
*               SOFA, Simulation Open-Framework Architecture                  *
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
*                           Plugin SoftRobots v1.0                            *
*                                                                             *
* This plugin is also distributed under the GNU LGPL (Lesser General          *
* Public License) license with the same conditions than SOFA.                 *
*                                                                             *
* Contributors: Defrost team  (INRIA, University of Lille, CNRS,              *
*               Ecole Centrale de Lille)                                      *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
*                                                                             *
******************************************************************************/

#include <string>
using std::string ;

#include <SofaTest/Sofa_test.h>
#include <sofa/helper/BackTrace.h>
#include <SofaBaseMechanics/MechanicalObject.h>

#include <SofaBaseLinearSolver/FullVector.h>
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

using sofa::helper::WriteAccessor ;
using sofa::defaulttype::Vec3Types ;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::container::MechanicalObject ;

#include <SoftRobots/component/constraint/UnilateralPlaneConstraint.h>
using sofa::component::constraintset::UnilateralPlaneConstraint ;


namespace sofa
{

template <typename _DataTypes>
struct UnilateralPlaneConstraintTest : public Sofa_test<typename _DataTypes::Real>,
        UnilateralPlaneConstraint<_DataTypes>
{
    typedef UnilateralPlaneConstraint<_DataTypes> ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename DataTypes::Deriv Deriv;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

    typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
    typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

    typedef BaseMeshTopology::Quad Quad;

    ////////////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using UnilateralPlaneConstraint<_DataTypes>::m_columnIndex ;
    using UnilateralPlaneConstraint<_DataTypes>::d_flipNormal ;
    using UnilateralPlaneConstraint<_DataTypes>::d_indices ;
    /////////////////////////////////////////////////////////////////////


    void normalTests(){
        Simulation* simu;
        setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;

        node->addObject(mecaobject) ;
        mecaobject->init() ;
        node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_TRUE( thisobject->findData("indices") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("flipNormal") != nullptr ) ;

        EXPECT_NO_THROW( thisobject->init() ) ;
        EXPECT_NO_THROW( thisobject->bwdInit() ) ;
        EXPECT_NO_THROW( thisobject->reinit() ) ;
        EXPECT_NO_THROW( thisobject->reset() ) ;

        return ;
    }

    bool buildMatrixTests(){
        Simulation* simu;
        setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;

        node->addObject(mecaobject) ;
        mecaobject->findData("position")->read("0. 0. 0.   1. 0. 0.   1. 1. 1.   1. -1. 1.");
        mecaobject->init();
        node->addObject(thisobject) ;
        thisobject->findData("indices")->read("0 1 2 3");
        thisobject->init();

        core::ConstraintParams* cparams = NULL;
        core::objectmodel::Data<MatrixDeriv> columns;
        unsigned int columnsIndex;
        core::objectmodel::Data<VecCoord> x;

        columnsIndex = 0;

        MatrixDeriv& column = *columns.beginEdit();
        MatrixDerivRowIterator rowIterator = column.begin();

        rowIterator = column.writeLine(columnsIndex);

        rowIterator.setCol(0, Deriv(0,0,0));
        rowIterator.setCol(1, Deriv(0,0,0));
        rowIterator.setCol(2, Deriv(0,0,0));
        rowIterator.setCol(3, Deriv(0,0,0));
        columns.endEdit();

        thisobject->buildConstraintMatrix(cparams, columns, columnsIndex, x);

        MatrixDeriv columnExpected;
        MatrixDerivRowIterator rowIteratorExpected = columnExpected.begin();

        rowIteratorExpected = columnExpected.writeLine(0);

        rowIteratorExpected.setCol(0, Deriv(2,0,0));
        rowIteratorExpected.setCol(1, Deriv(-0.666667,0,0));
        rowIteratorExpected.setCol(2, Deriv(-0.666667,0,0));
        rowIteratorExpected.setCol(3, Deriv(-0.666667,0,0));

        MatrixDerivRowConstIterator rowIt2 = columnExpected.readLine(0);
        MatrixDerivColConstIterator rowEnd2 = rowIt2.end();

        MatrixDerivRowConstIterator rowIt1 = column.readLine(0);
        MatrixDerivColConstIterator rowEnd1 = rowIt1.end();

        for (MatrixDerivColConstIterator colIt1 = rowIt1.begin(), colIt2 = rowIt2.begin(); colIt1 != rowEnd1 && colIt2 != rowEnd2; ++colIt1, colIt2++)
        {
            if(colIt1.val() != colIt2.val())
                return false;
        }

        return true;
    }

};

using testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_CASE(UnilateralPlaneConstraintTest, DataTypes);


TYPED_TEST(UnilateralPlaneConstraintTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(UnilateralPlaneConstraintTest, BuildMatrixTests) {
    EXPECT_TRUE(this->buildMatrixTests()) ;
}

}

