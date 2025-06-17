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
#include <SoftRobots/component/initSoftRobots.h>
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/ComponentChange.h>
#include <sofa/Modules.h>

#include <sofa/helper/system/PluginManager.h>
using sofa::helper::system::PluginManager;

#include <fstream>

namespace softrobots
{
namespace constraint
{
    extern void registerCableConstraint(sofa::core::ObjectFactory* factory);
    extern void registerJointConstraint(sofa::core::ObjectFactory* factory);
    extern void registerPartialRigidificationConstraint(sofa::core::ObjectFactory* factory);
    extern void registerPositionConstraint(sofa::core::ObjectFactory* factory);
    extern void registerSurfacePressureConstraint(sofa::core::ObjectFactory* factory);
    extern void registerUnilateralPlaneConstraint(sofa::core::ObjectFactory* factory);
}

namespace controller
{
    extern void registerAnimationEditor(sofa::core::ObjectFactory* factory);
    extern void registerDataVariationLimiter(sofa::core::ObjectFactory* factory);
    extern void registerSerialPortBridgeGeneric(sofa::core::ObjectFactory* factory);

    #ifdef SOFTROBOTS_USES_COMMUNICATIONCONTROLLER
    extern void registerCommunicationController(sofa::core::ObjectFactory* factory);
    #endif

    #ifdef SOFTROBOTS_USES_ROBOTINOCONTROLLER
    extern void registerDataControllerRobot(sofa::core::ObjectFactory* factory);
    #endif

    #ifdef SOFTROBOTS_USES_GAMETRAKCONTROLLER
    extern void registerGameTrackController(sofa::core::ObjectFactory* factory);
    #endif

    #ifdef SOFTROBOTS_USES_INTERACTIVECONTROL
    extern void registerInteractiveControl(sofa::core::ObjectFactory* factory);
    #endif

    #ifdef SOFTROBOTS_USES_WITH_CAMERA
    extern void registerPointCloudProcessing(sofa::core::ObjectFactory* factory);
    #endif
}

namespace engine
{
    extern void registerCenterOfMass(sofa::core::ObjectFactory* factory);
}

namespace forcefield
{
    extern void registerPREquivalentStiffnessForceField(sofa::core::ObjectFactory* factory);
    extern void registerPartialRigidificationForceField(sofa::core::ObjectFactory* factory);
    extern void registerPipeForceField(sofa::core::ObjectFactory* factory);
}

extern "C" {
    SOFA_SOFTROBOTS_API void initExternalModule();
    SOFA_SOFTROBOTS_API const char* getModuleName();
    SOFA_SOFTROBOTS_API const char* getModuleVersion();
    SOFA_SOFTROBOTS_API const char* getModuleLicense();
    SOFA_SOFTROBOTS_API const char* getModuleDescription();
    SOFA_SOFTROBOTS_API const char* getModuleComponentList();
    SOFA_SOFTROBOTS_API void registerObjects(sofa::core::ObjectFactory* factory);
}

void init()
{    
    static bool first = true;
    if (!first)
    {
        return;
    }
    first = false;

    // moved components
    sofa::helper::lifecycle::movedComponents.insert({
        { "VolumeFromTriangles", sofa::helper::lifecycle::Moved("v25.12", "SoftRobots", Sofa.Component.Engine.Generate) },
        { "VolumeFromTetrahedrons", sofa::helper::lifecycle::Moved("v25.12", "SoftRobots", Sofa.Component.Engine.Generate) }
    });

    // make sure that this plugin is registered into the PluginManager
    sofa::helper::system::PluginManager::getInstance().registerPlugin(MODULE_NAME);

    // Automatically load the STLIB plugin if available.
    if( !PluginManager::getInstance().findPlugin("STLIB").empty() )
    {
        PluginManager::getInstance().loadPlugin("STLIB") ;
    }
}

void initExternalModule()
{
    init();
}

const char* getModuleName()
{
    return MODULE_NAME;
}

const char* getModuleVersion()
{
    return MODULE_VERSION;
}

const char* getModuleLicense()
{
    return "LGPL";
}

const char* getModuleDescription()
{
    return "The plugin allows to model soft robots";
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    constraint::registerCableConstraint(factory);
    constraint::registerJointConstraint(factory);
    constraint::registerPartialRigidificationConstraint(factory);
    constraint::registerPositionConstraint(factory);
    constraint::registerSurfacePressureConstraint(factory);
    constraint::registerUnilateralPlaneConstraint(factory);
    controller::registerAnimationEditor(factory);
    controller::registerDataVariationLimiter(factory);
    controller::registerSerialPortBridgeGeneric(factory);
#ifdef SOFTROBOTS_USES_COMMUNICATIONCONTROLLER
    controller::registerCommunicationController(factory);
#endif
#ifdef SOFTROBOTS_USES_ROBOTINOCONTROLLER
    controller::registerDataControllerRobot(factory);
#endif
#ifdef SOFTROBOTS_USES_GAMETRAKCONTROLLER
    controller::registerGameTrackController(factory);
#endif
#ifdef SOFTROBOTS_USES_INTERACTIVECONTROL
    controller::registerInteractiveControl(factory);
#endif
#ifdef SOFTROBOTS_USES_WITH_CAMERA
    controller::registerPointCloudProcessing(factory);
#endif
    engine::registerCenterOfMass(factory);
    forcefield::registerPREquivalentStiffnessForceField(factory);
    forcefield::registerPartialRigidificationForceField(factory);
    forcefield::registerPipeForceField(factory);
}

} // namespace SoftRobots
