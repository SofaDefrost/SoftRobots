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

#include <sofa/testing/BaseTest.h>
#include <sofa/helper/BackTrace.h>
#include <sofa/component/statecontainer/MechanicalObject.h>

#include <sofa/linearalgebra/FullVector.h>
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

#include <sofa/component/topology/container/dynamic/TetrahedronSetTopologyContainer.h>
using sofa::component::topology::container::dynamic::TetrahedronSetTopologyContainer ;

using sofa::helper::WriteAccessor ;
using sofa::defaulttype::Vec3Types ;

#include <sofa/simulation/common/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <sofa/simulation/graph/DAGSimulation.h>
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::statecontainer::MechanicalObject ;

#include <SoftRobots/component/engine/VolumeFromTetrahedrons.h>
using softrobots::engine::VolumeFromTetrahedrons ;

#include <sofa/helper/system/FileRepository.h>
using sofa::helper::system::DataRepository;


namespace softrobots
{

template <typename _DataTypes>
struct VolumeFromTetrahedronsTest : public sofa::testing::BaseTest, VolumeFromTetrahedrons<_DataTypes>
{
    typedef VolumeFromTetrahedrons<_DataTypes> ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::VecCoord VecCoord;

    typedef BaseMeshTopology::Tetra Tetra;
    typedef BaseMeshTopology::Hexa  Hexa;

    typedef BaseMeshTopology::SeqTetrahedra VecTetra;
    typedef BaseMeshTopology::SeqHexahedra  VecHexa;

    ///////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using VolumeFromTetrahedrons<_DataTypes>::d_volume ;
    using VolumeFromTetrahedrons<_DataTypes>::d_tetras ;
    using VolumeFromTetrahedrons<_DataTypes>::d_hexas  ;
    ///////////////////////////////////////////////////////////////


    void normalTests(){
        Simulation* simu;
        setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

        Node::SPtr node = simu->createNewGraph("root");
        typename MechanicalObject<DataTypes>::SPtr mecaobject = New<MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;
        mecaobject->init() ;

        node->addObject(mecaobject) ;
        node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_TRUE( thisobject->findData("tetras") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("hexas") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("volume") != nullptr ) ;

        return ;
    }

    void simpleSceneTest(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'> "
                "   <MechanicalObject/>              "
                "   <VolumeFromTetrahedrons/>         "
                "</Node>                             " ;
        EXPECT_NO_THROW(SceneLoaderXML::loadFromMemory ( "test1", scene.c_str())) ;
    }

    double volumeComputationTest()
    {
        DataRepository.addFirstPath(SOFTROBOTS_TEST_DIR);

        Simulation* simu;
        setSimulation(simu = new sofa::simulation::graph::DAGSimulation());

        Node::SPtr node = simu->createNewGraph("root");

        typename TetrahedronSetTopologyContainer::SPtr    mesh       = New< TetrahedronSetTopologyContainer >() ;
        typename MechanicalObject<DataTypes>::SPtr        mecaobject = New< MechanicalObject<DataTypes> >() ;
        typename ThisClass::SPtr                          thisobject = New< ThisClass >() ;

        node->addObject(mesh) ;
        mesh->load("component/engine/mesh/single_tetra.msh");
        node->addObject(mecaobject) ;
        mecaobject->init();
        node->addObject(thisobject) ;

        EXPECT_NO_THROW(thisobject->init());

        return thisobject->getVolume();
    }
};

using ::testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_SUITE(VolumeFromTetrahedronsTest, DataTypes);


TYPED_TEST(VolumeFromTetrahedronsTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(VolumeFromTetrahedronsTest, SimpleScene) {
    ASSERT_NO_THROW(this->simpleSceneTest()) ;
}

TYPED_TEST(VolumeFromTetrahedronsTest, VolumeComputation) {
    ASSERT_DOUBLE_EQ(12.5,this->volumeComputationTest());
}
}
