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

#include <SoftRobots/component/constraint/SurfacePressureConstraint.h>
using softrobots::constraint::SurfacePressureConstraint;

#include <sofa/linearalgebra/FullVector.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/helper/system/Locale.h>

#include <sofa/testing/BaseSimulationTest.h>

#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simpleapi/SimpleApi.h>

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;

using std::string;
using std::fabs;
using std::stof;

using sofa::core::objectmodel::ComponentState;

namespace softrobots {

    template <typename _DataTypes>
    struct SurfacePressureConstraintTest : public sofa::testing::BaseTest, SurfacePressureConstraint<_DataTypes>
    {

        using SurfacePressureConstraint<_DataTypes>::d_componentState;

        sofa::simulation::Node::SPtr m_root;                 ///< Root of the scene graph, created by the constructor an re-used in the tests

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

        void doSetUp() override
        {
            sofa::simpleapi::importPlugin(Sofa.Component);

            /// Load the scene
            string sceneName = "SurfacePressureConstraint.scn";

            string fileName  = string(SOFTROBOTS_TEST_DIR) + "/component/constraint/scenes/" + sceneName;

            m_root = sofa::core::objectmodel::SPtr_dynamic_cast<sofa::simulation::Node>( sofa::simulation::node::load(fileName.c_str()));

            /// Test if load has succeededls
            sofa::simulation::SceneLoaderXML scene;

            if(!m_root || !scene.loadSucceed)
                ADD_FAILURE() << "Error while loading the scene: " << sceneName << std::endl;
        }


        // Test the behavior of the algorithm
        void behaviorTests()
        {
            sofa::helper::system::TemporaryLocale locale(LC_NUMERIC, "C");

            sofa::simulation::node::initRoot(m_root.get());

            int nbTimeStep = 15;
            string deltaString;
            double tolerance = 1e-1;

            // Test normal behavior
            m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("valueType")->read("volumeGrowth");
            m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("value")->read("40");
            string initialVolume = m_root->getChild("bunny")->getChild("cavity")->getObject("SurfacePressureConstraint")->findData("initialCavityVolume")->getValueString();

            for(int i=0; i<nbTimeStep; i++)
                sofa::simulation::node::animate(m_root.get());

            m_root->getChild("bunny")->getChild("cavity")->getObject("VolumeFromTriangles")->findData("update")->read("1");
            deltaString = m_root->getChild("bunny")->getChild("cavity")->getObject("VolumeFromTriangles")->findData("volume")->getValueString();

            double error = fabs(stof(deltaString.c_str())-(stof(initialVolume.c_str())+40))/stof(deltaString.c_str());
            EXPECT_TRUE(error <= tolerance) << "Error is " << error << ". Expected error < " << tolerance ;
        }


    };

    using ::testing::Types;
    typedef Types<sofa::defaulttype::Vec3Types> DataTypes;

    TYPED_TEST_SUITE(SurfacePressureConstraintTest, DataTypes);

    TYPED_TEST(SurfacePressureConstraintTest, behaviorTests) {
        EXPECT_NO_THROW( this->behaviorTests() );
    }
}

