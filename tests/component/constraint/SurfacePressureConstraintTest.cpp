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

#include <SofaTest/Sofa_test.h>

#include "../../../component/constraint/SurfacePressureConstraint.h"
using sofa::component::constraintset::SurfacePressureConstraint;

#include <SofaBaseLinearSolver/FullVector.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/Locale.h>

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;

using std::string;
using std::fabs;
using std::stof;

using sofa::core::objectmodel::ComponentState;

namespace sofa {

    template <typename _DataTypes>
    struct SurfacePressureConstraintTest : public Sofa_test<typename _DataTypes::Real>, SurfacePressureConstraint<_DataTypes>
    {

//        using SurfacePressureConstraint<_DataTypes>::m_componentstate;

        simulation::Node::SPtr m_root;                 ///< Root of the scene graph, created by the constructor an re-used in the tests
        simulation::Simulation* m_simulation;          ///< created by the constructor an re-used in the tests

        typedef _DataTypes DataTypes;
        typedef typename DataTypes::Deriv Deriv;
        typedef typename DataTypes::VecDeriv VecDeriv;
        typedef typename DataTypes::MatrixDeriv MatrixDeriv;
        typedef typename DataTypes::Coord Coord;
        typedef typename DataTypes::VecCoord VecCoord;
        typedef typename DataTypes::MatrixDeriv::RowIterator MatrixDerivRowIterator;

        typedef typename DataTypes::MatrixDeriv::RowConstIterator MatrixDerivRowConstIterator;
        typedef typename DataTypes::MatrixDeriv::ColConstIterator MatrixDerivColConstIterator;

        typedef sofa::core::topology::BaseMeshTopology::Quad Quad;

        bool testBuildConstraintMatrix() {

            core::ConstraintParams* cparams = NULL;
            core::objectmodel::Data<MatrixDeriv> columns;
            unsigned int columnsIndex = 0;
            core::objectmodel::Data<VecCoord> x;

            helper::WriteAccessor<Data<VecCoord> > positions = x;
            positions.clear();
            positions.push_back(Coord(0,0,0));
            positions.push_back(Coord(0,1,0));
            positions.push_back(Coord(1,1,0));
            positions.push_back(Coord(1,0,0));
            positions.push_back(Coord(0,0,1));
            positions.push_back(Coord(1,0,1));
            positions.push_back(Coord(1,1,1));
            positions.push_back(Coord(0,1,1));

            helper::WriteAccessor<Data<sofa::helper::vector<Quad> > > quads = this->d_quads;
            quads.clear();
            quads.push_back(Quad(0,1,2,3));
            quads.push_back(Quad(4,5,6,7));
            quads.push_back(Quad(2,6,5,3));
            quads.push_back(Quad(7,1,0,4));
            quads.push_back(Quad(4,0,3,5));
            quads.push_back(Quad(1,7,6,2));

            MatrixDeriv& column = *columns.beginEdit();
            columns.endEdit();

//            m_componentstate = ComponentState::Valid;
            this->buildConstraintMatrix(cparams, columns, columnsIndex, x);

            MatrixDeriv columnExpected;
            MatrixDerivRowIterator rowIteratorExpected = columnExpected.begin();

            rowIteratorExpected = columnExpected.writeLine(0);

            rowIteratorExpected.setCol(0, Deriv(-0.25,-0.25,-0.25));
            rowIteratorExpected.setCol(1, Deriv(-0.25, 0.25,-0.25));
            rowIteratorExpected.setCol(2, Deriv( 0.25, 0.25,-0.25));
            rowIteratorExpected.setCol(3, Deriv( 0.25,-0.25,-0.25));
            rowIteratorExpected.setCol(4, Deriv(-0.25,-0.25, 0.25));
            rowIteratorExpected.setCol(5, Deriv( 0.25,-0.25, 0.25));
            rowIteratorExpected.setCol(6, Deriv( 0.25, 0.25, 0.25));
            rowIteratorExpected.setCol(7, Deriv(-0.25, 0.25, 0.25));


            MatrixDerivRowConstIterator rowIt2 = columnExpected.readLine(0);
            MatrixDerivColConstIterator rowEnd2 = rowIt2.end();

            MatrixDerivRowConstIterator rowIt1 = column.readLine(0);
            MatrixDerivColConstIterator rowEnd1 = rowIt1.end();

            for (MatrixDerivColConstIterator colIt1 = rowIt1.begin(), colIt2 = rowIt2.begin(); colIt1 != rowEnd1, colIt2 != rowEnd2; ++colIt1, colIt2++)
            {
                if(colIt1.val() != colIt2.val())
                    return false;
            }

            if(columnsIndex != 1)
                return false;

            return true;
        }


        void SetUp()
        {
            simulation::setSimulation(m_simulation = new simulation::graph::DAGSimulation());

            /// Load the scene
            string sceneName = "SurfacePressureConstraint.scn";

            string fileName  = string(SOFTROBOTS_TEST_DIR) + "/component/constraint/scenes/" + sceneName;

            m_root = core::objectmodel::SPtr_dynamic_cast<simulation::Node>( simulation::getSimulation()->load(fileName.c_str()));

            /// Test if load has succeededls
            simulation::SceneLoaderXML scene;

            if(!m_root || !scene.loadSucceed)
                ADD_FAILURE() << "Error while loading the scene: " << sceneName << std::endl;
        }


        // Test the behavior of the algorithm
        void behaviorTests()
        {
            helper::system::TemporaryLocale locale(LC_NUMERIC, "C");

            SetUp();

            m_simulation->init(m_root.get());

            int nbTimeStep = 15;
            string deltaString;
            double tolerance = 1e-1;

            // Test normal behavior
            m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("valueType")->read("volumeGrowth");
            m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("value")->read("40");
            string initialVolume = m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("initialCavityVolume")->getValueString();

            for(int i=0; i<nbTimeStep; i++)
                m_simulation->animate(m_root.get());

            m_root->getChild("bunny")->getChild("cavity")->getObject("VolumeFromTriangles")->findData("update")->read("1");
            deltaString = m_root->getChild("bunny")->getChild("cavity")->getObject("VolumeFromTriangles")->findData("volume")->getValueString();

            double error = fabs(stof(deltaString.c_str())-(stof(initialVolume.c_str())+40))/stof(deltaString.c_str());
            EXPECT_TRUE(error <= tolerance) << "Error is " << error << ". Expected error < " << tolerance ;
        }


    };

    using testing::Types;
    typedef Types<sofa::defaulttype::Vec3Types> DataTypes;

    TYPED_TEST_CASE(SurfacePressureConstraintTest, DataTypes);

    TYPED_TEST(SurfacePressureConstraintTest, BuildConstraintMatrixTest) {
        ASSERT_TRUE( this->testBuildConstraintMatrix() );
    }

    TYPED_TEST(SurfacePressureConstraintTest, behaviorTests) {
        EXPECT_NO_THROW( this->behaviorTests() );
    }
}

