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

#include <SofaBaseLinearSolver/FullVector.h>

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
#include <sofa/simulation/Node.h>
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <SoftRobots/component/controller/DataVariationLimiter.h>
using sofa::component::controller::DataVariationLimiter ;


namespace sofa
{

template <typename _DataTypes>
struct DataVariationLimiterTest : public sofa::testing::BaseTest, DataVariationLimiter<_DataTypes>
{
    typedef DataVariationLimiter<_DataTypes> ThisClass ;
    typedef _DataTypes DataTypes;
    typedef typename type::vector<DataTypes> VecValue;

    ///////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using DataVariationLimiter<DataTypes>::d_input ;
    using DataVariationLimiter<DataTypes>::d_output ;
    using DataVariationLimiter<DataTypes>::d_inputSize ;
    using DataVariationLimiter<DataTypes>::d_maxJump ;
    using DataVariationLimiter<DataTypes>::d_nbStep ;
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
        EXPECT_TRUE( thisobject->findData("input") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("output") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("size") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("maxJump") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("nbStep") != nullptr ) ;

        return ;
    }


    void simpleSceneTest(){
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject name='MO' position='0 0 0'/>              "
                "   <DataVariationLimiter listening='true' input='@MO.position'/>         "
                "</Node>                             " ;
        EXPECT_NO_THROW(SceneLoaderXML::loadFromMemory ( "simpleSceneTest", scene.c_str(), scene.size())) ;
        return ;
    }


    bool initTest(){

        VecValue v;
        v.resize(2);

        d_input.setValue(v);
        d_nbStep.setValue(0);  // nbStep < 1 not allowed
        d_maxJump.setValue(-1); // maxJump < 0 not allowed
        this->init();

        if(d_nbStep.getValue() < 1)
            return false;

        if(d_maxJump.getValue() < 0)
            return false;

        if(d_output.getValue().size() != d_input.getValue().size())
            return false;

        return true;
    }


    bool reinitTest(){

        this->init();

        d_nbStep.setValue(0);  // nbStep < 1 not allowed
        d_maxJump.setValue(-1); // maxJump < 0 not allowed
        this->reinit();

        if(d_nbStep.getValue() < 1)
            return false;

        if(d_maxJump.getValue() < 0)
            return false;

        return true;
    }

};

using ::testing::Types;
typedef Types<type::Vec3d> DataTypes;

TYPED_TEST_SUITE(DataVariationLimiterTest, DataTypes);


TYPED_TEST(DataVariationLimiterTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(DataVariationLimiterTest, SimpleScene) {
    ASSERT_NO_THROW(this->simpleSceneTest()) ;
}

TYPED_TEST(DataVariationLimiterTest, initTest) {
    ASSERT_TRUE(this->initTest()) ;
}

TYPED_TEST(DataVariationLimiterTest, reinitTest) {
    ASSERT_TRUE(this->reinitTest()) ;
}

}
