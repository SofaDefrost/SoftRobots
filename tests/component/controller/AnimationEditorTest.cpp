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
#include <sofa/simulation/DefaultAnimationLoop.h>
using sofa::simulation::DefaultAnimationLoop;

#include <SofaBaseLinearSolver/FullVector.h>
using sofa::core::topology::BaseMeshTopology ;
using sofa::core::objectmodel::Data ;

#include <SofaSimulationGraph/DAGSimulation.h>
using sofa::simulation::Simulation ;
using sofa::simulation::Node ;
using sofa::simulation::setSimulation ;
using sofa::core::objectmodel::New ;
using sofa::core::objectmodel::BaseData ;
using sofa::component::container::MechanicalObject ;

#include <SofaSimulationCommon/SceneLoaderXML.h>
using sofa::simulation::SceneLoaderXML ;

#include <SoftRobots/component/controller/AnimationEditor.h>
using sofa::component::controller::AnimationEditor ;

#include <sofa/core/behavior/MechanicalState.h>
using sofa::core::behavior::MechanicalState;


namespace sofa
{

template <typename DataTypes>
struct AnimationEditorTest : public Sofa_test<typename DataTypes::Real>, AnimationEditor<DataTypes>
{
    typedef AnimationEditor<DataTypes> ThisClass ;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;

    ///////////////////////////////////////////////////////////////////////
    // Bring parents members in the current lookup context.
    // more info at: https://gcc.gnu.org/onlinedocs/gcc/Name-lookup.html
    using AnimationEditor<DataTypes>::d_maxKeyFrame ;
    using AnimationEditor<DataTypes>::d_cursor ;
    using AnimationEditor<DataTypes>::d_filename ;
    using AnimationEditor<DataTypes>::m_keyFramesID ;
    using AnimationEditor<DataTypes>::m_maxKeyFrameID ;
    using AnimationEditor<DataTypes>::m_animation ;
    using AnimationEditor<DataTypes>::m_state ;
    using AnimationEditor<DataTypes>::moveCursor ;
    using AnimationEditor<DataTypes>::addKeyFrame ;
    using AnimationEditor<DataTypes>::deleteKeyFrame ;
    ///////////////////////////////////////////////////////////////////////


    Simulation* m_simu;
    Node::SPtr m_node;
    typename MechanicalObject<DataTypes>::SPtr m_mecaobject;


    void SetUp()
    {
        setSimulation(m_simu = new sofa::simulation::graph::DAGSimulation());

        m_node = m_simu->createNewGraph("root");
        m_mecaobject = New<MechanicalObject<DataTypes> >() ;
        m_mecaobject->init() ;
        m_node->addObject(m_mecaobject) ;
    }

    void normalTests()
    {
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;
        m_node->addObject(thisobject) ;

        thisobject->setName("myname") ;
        EXPECT_TRUE(thisobject->getName() == "myname") ;

        EXPECT_TRUE( thisobject->findData("maxKeyFrame") != nullptr ) ;
        EXPECT_TRUE( thisobject->findData("filename") != nullptr ) ;

        return ;
    }


    void simpleSceneTest()
    {
        string scene =
                "<?xml version='1.0'?>"
                "<Node 	name='Root' gravity='0 0 0' time='0' animate='0'   > "
                "   <MechanicalObject position='0 0 0'/>              "
                "   <AnimationEditor listening='true'/>         "
                "</Node>                             " ;
        EXPECT_NO_THROW(SceneLoaderXML::loadFromMemory ( "simpleSceneTest", scene.c_str(), scene.size())) ;
        return ;
    }


    bool initTest()
    {
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;

        m_mecaobject->findData("position")->read("0. 0. 0.");

        m_node->addObject(thisobject) ;
        thisobject->init();

        if(thisobject->m_keyFramesID.size() != 1)
            return false;

        if(thisobject->m_keyFramesID[0] != 0)
            return false;

        if(thisobject->m_animation.size() != 1)
            return false;

        thisobject->findData("maxKeyFrame")->read("-1"); // maxKeyFrame <= 0 not allowed
        thisobject->init();
        if(thisobject->d_maxKeyFrame.getValue() != 1)
            return false;

        thisobject->findData("maxKeyFrame")->read("0"); // maxKeyFrame <= 0 not allowed
        thisobject->init();
        if(thisobject->d_maxKeyFrame.getValue() != 1)
            return false;

        return true;
    }


    bool reinitTest()
    {
        typename ThisClass::SPtr thisobject = New<ThisClass >() ;

        m_mecaobject->findData("position")->read("0. 0. 0.");

        m_node->addObject(thisobject) ;
        thisobject->init();

        thisobject->findData("maxKeyFrame")->read("-1"); // maxKeyFrame <= 0 not allowed
        thisobject->reinit();
        if(thisobject->d_maxKeyFrame.getValue() != 1)
            return false;

        thisobject->findData("maxKeyFrame")->read("0"); // maxKeyFrame <= 0 not allowed
        thisobject->reinit();
        if(thisobject->d_maxKeyFrame.getValue() != 1)
            return false;

        return true;
    }


    void moveCursorTest()
    {
        d_cursor.setValue(0);
        moveCursor(20);
        EXPECT_EQ(d_cursor.getValue(),1);

        d_cursor.setValue(2);
        moveCursor(18);
        EXPECT_EQ(d_cursor.getValue(),1);

        d_cursor.setValue(0);
        moveCursor(18);
        EXPECT_EQ(d_cursor.getValue(),0);

        d_cursor.setValue(d_maxKeyFrame.getValue());
        moveCursor(20);
        EXPECT_EQ(d_cursor.getValue(),d_maxKeyFrame.getValue());

        m_keyFramesID.clear();
        m_keyFramesID.push_back(0);
        m_keyFramesID.push_back(53);
        m_keyFramesID.push_back(24);
        m_keyFramesID.push_back(100);
        m_maxKeyFrameID = 100;

        d_cursor.setValue(30);
        moveCursor(23); //PgDn
        EXPECT_EQ(d_cursor.getValue(),53);

        d_cursor.setValue(30);
        moveCursor(22); //PgUp
        EXPECT_EQ(d_cursor.getValue(),24);

        d_cursor.setValue(0);
        moveCursor(22); //PgUp
        EXPECT_EQ(d_cursor.getValue(),0);

        d_cursor.setValue(100);
        moveCursor(23); //PgDn
        EXPECT_EQ(d_cursor.getValue(),100);
    }


    bool addAndDeleteKeyFrameTest()
    {
        m_state = dynamic_cast<MechanicalState<DataTypes>*>(m_node->getMechanicalState());

        m_keyFramesID.clear();

        if(d_maxKeyFrame.getValue()<=0)
            d_maxKeyFrame.setValue(1);

        m_keyFramesID.push_back(0);
        m_animation.resize(1,m_state->read(core::ConstVecCoordId::position())->getValue());

        d_cursor.setValue(5);
        VecCoord newPosition;
        newPosition.resize(1);
        newPosition[0] = Coord(0.,12.5,0.);
        m_state->write(core::VecCoordId::position())->setValue(newPosition);
        addKeyFrame();
        if(m_keyFramesID.size() != 2) return false;
        if(m_keyFramesID[1] != 5) return false;
        if(m_animation.size() != 6) return false;
        EXPECT_EQ(m_animation[5],newPosition);

        deleteKeyFrame();
        if(m_keyFramesID.size() != 1) return false;
        if(m_animation.size() != 1) return false;

        return true;
    }


    void onBeginAnimationStepTest()
    {
        // Should be called one time at each time step
        typename ThisClass::SPtr thisobject = New<ThisClass >();
        typename DefaultAnimationLoop::SPtr animationLoop = New<DefaultAnimationLoop >();
        m_node->addObject(animationLoop) ;
        animationLoop->init();

        thisobject->setPlaying(true);
        thisobject->findData("listening")->read("1");
        thisobject->findData("frameTime")->read("1");
        m_node->addObject(thisobject) ;
        thisobject->init();

        m_simu->animate(m_node.get());
        double time = thisobject->getTime();
        double dt = m_node->getDt();

        EXPECT_EQ(time,dt);
    }


};

using testing::Types;
typedef Types<Vec3Types> DataTypes;

TYPED_TEST_CASE(AnimationEditorTest, DataTypes);


TYPED_TEST(AnimationEditorTest, NormalBehavior) {
    ASSERT_NO_THROW(this->normalTests()) ;
}

TYPED_TEST(AnimationEditorTest, SimpleScene) {
    ASSERT_NO_THROW(this->simpleSceneTest()) ;
}

TYPED_TEST(AnimationEditorTest, initTest) {
    ASSERT_TRUE(this->initTest()) ;
}

TYPED_TEST(AnimationEditorTest, reinitTest) {
    ASSERT_TRUE(this->reinitTest()) ;
}

TYPED_TEST(AnimationEditorTest, moveCursorTest) {
    ASSERT_NO_THROW(this->moveCursorTest()) ;
}

TYPED_TEST(AnimationEditorTest, addAndDeleteKeyFrameTest) {
    ASSERT_TRUE(this->addAndDeleteKeyFrameTest()) ;
}

TYPED_TEST(AnimationEditorTest, onBeginAnimationStepTest) {
    ASSERT_NO_THROW(this->onBeginAnimationStepTest()) ;
}

}
