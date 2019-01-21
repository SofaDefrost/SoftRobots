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

#include <SofaBaseLinearSolver/FullVector.h>

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <SoftRobots/component/controller/SerialPortBridgeGeneric.h>
using sofa::component::controller::SerialPortBridgeGeneric ;


namespace sofa
{

using helper::vector;
using helper::WriteAccessor;

template <typename _DataTypes>
struct SerialPortBridgeGenericTest : public Sofa_test<typename _DataTypes::value_type>, SerialPortBridgeGeneric
{
    typedef SerialPortBridgeGeneric ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename helper::vector<DataTypes> VecValue;

    ///////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using SerialPortBridgeGeneric::d_size ;
    using SerialPortBridgeGeneric::d_baudRate ;
    using SerialPortBridgeGeneric::d_precise ;
    using SerialPortBridgeGeneric::d_packetOut ;
    using SerialPortBridgeGeneric::d_port ;
    using SerialPortBridgeGeneric::d_header ;
    using SerialPortBridgeGeneric::d_splitPacket ;
    using SerialPortBridgeGeneric::m_packetOut ;
    ///////////////////////////////////////////////////////////////


    Simulation* m_simu;
    Node::SPtr m_node;

    void SetUp()
    {
        setSimulation(m_simu = new sofa::simulation::graph::DAGSimulation());
        m_node = m_simu->createNewGraph("root");
    }

    void normalTests(){

        typename ThisClass::SPtr thisobject = New<ThisClass>() ;
        m_node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        /// Some test to check that the parameters are still there.
        EXPECT_TRUE( thisobject->findData("size") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("precise") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("baudRate") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("sentData") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("port") != nullptr ) ;

        EXPECT_TRUE(thisobject->findData("size")->getValueString()=="0") ;
        EXPECT_TRUE(thisobject->findData("precise")->getValueString()=="0");

        return ;
    }


    void simpleSceneTest(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <SerialPortBridgeGeneric listening='true' input='@MO.position'/>         "
                "</Node>                             " ;
        EXPECT_NO_THROW(SceneLoaderXML::loadFromMemory ( "simpleSceneTest", scene.c_str(), scene.size())) ;
        return ;
    }


    void checkIndicesTest(){

        typename ThisClass::SPtr thisobject = New<ThisClass>() ;
        m_node->addObject(thisobject) ;

        thisobject->findData("size")->read("4");
        thisobject->onEndAnimationStep(0.0);
        EXPECT_EQ(thisobject->findData("size")->getValueString(),"0");

        thisobject->findData("sentData")->read("0 1 2 3");
        thisobject->onEndAnimationStep(0.0);
        EXPECT_EQ(thisobject->findData("size")->getValueString(),"4");

        thisobject->findData("size")->read("3");
        thisobject->findData("sentData")->read("0 1 2");
        thisobject->onEndAnimationStep(0.0);
        EXPECT_EQ(thisobject->findData("size")->getValueString(),"3");
    }



    void initTest(){
        d_size.setValue(4);
        d_precise.setValue(false);
        d_splitPacket.setValue(false);

        init();
        EXPECT_EQ(m_packetOut.size(),5);
        EXPECT_EQ(m_packetOut[0],d_header.getValue()[0]);
        for(int i=1; i<5; i++)
            EXPECT_EQ(m_packetOut[i],0);


        d_size.setValue(3);
        d_precise.setValue(true);
        d_splitPacket.setValue(false);

        init();
        EXPECT_EQ(m_packetOut.size(),7);
        EXPECT_EQ(m_packetOut[0],d_header.getValue()[0]);
        for(unsigned int i=1; i<7; i++)
            EXPECT_EQ(m_packetOut[i],0);

        d_size.setValue(3);
        d_precise.setValue(true);
        d_splitPacket.setValue(true);
        init();
        EXPECT_EQ(m_packetOut.size(),8);
        EXPECT_EQ(m_packetOut[0],d_header.getValue()[0]);
        for(unsigned int i=1; i<4; i++)
            EXPECT_EQ(m_packetOut[i],0);
        EXPECT_EQ(m_packetOut[4],d_header.getValue()[1]);
        for(unsigned int i=5; i<7; i++)
            EXPECT_EQ(m_packetOut[i],0);
    }


    void onEndAnimationStepTest(){

        WriteAccessor<Data<vector<unsigned char>>> packet = d_packetOut;

        d_size.setValue(4);
        d_precise.setValue(false);
        d_splitPacket.setValue(false);
        packet.clear();
        packet.resize(4);

        onEndAnimationStep(0.0);
        EXPECT_EQ(m_packetOut.size(),5);
        EXPECT_EQ(m_packetOut[0],d_header.getValue()[0]);
        for(int i=1; i<5; i++)
            EXPECT_EQ(m_packetOut[i],0);


        d_size.setValue(3);
        d_precise.setValue(true);
        d_splitPacket.setValue(false);
        packet.clear();
        packet.resize(3);

        onEndAnimationStep(0.0);
        EXPECT_EQ(m_packetOut.size(),7);
        EXPECT_EQ(m_packetOut[0],d_header.getValue()[0]);
        for(int i=1; i<7; i++)
            EXPECT_EQ(m_packetOut[i],0);
    }




};

using testing::Types;
typedef Types<defaulttype::Vec3d> DataTypes;

TYPED_TEST_CASE(SerialPortBridgeGenericTest, DataTypes);


TYPED_TEST(SerialPortBridgeGenericTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(SerialPortBridgeGenericTest, SimpleScene) {
    ASSERT_NO_THROW(this->simpleSceneTest()) ;
}

TYPED_TEST(SerialPortBridgeGenericTest, checkIndicesTest) {
    ASSERT_NO_THROW(this->checkIndicesTest()) ;
}

TYPED_TEST(SerialPortBridgeGenericTest, initTest) {
    ASSERT_NO_THROW(this->initTest()) ;
}

TYPED_TEST(SerialPortBridgeGenericTest, onEndAnimationStepTest) {
    ASSERT_NO_THROW(this->onEndAnimationStepTest()) ;
}

}
