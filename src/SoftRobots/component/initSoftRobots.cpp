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

#include <sofa/helper/system/PluginManager.h>
using sofa::helper::system::PluginManager;

#include <fstream>

namespace SoftRobots
{

extern "C" {
    SOFA_SOFTROBOTS_API void initExternalModule();
    SOFA_SOFTROBOTS_API const char* getModuleName();
    SOFA_SOFTROBOTS_API const char* getModuleVersion();
    SOFA_SOFTROBOTS_API const char* getModuleLicense();
    SOFA_SOFTROBOTS_API const char* getModuleDescription();
    SOFA_SOFTROBOTS_API const char* getModuleComponentList();
}

void initExternalModule()
{    
    static bool first = true;
    if (!first)
    {
        return;
    }
    first = false;

    // Automatically load the STLIB plugin if available.
    if( !PluginManager::getInstance().findPlugin("STLIB").empty() )
    {
        PluginManager::getInstance().loadPlugin("STLIB") ;
    }
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
    return "The plugin allows to control soft robots";
}

const char* getModuleComponentList()
{
    /// string containing the names of the classes provided by the plugin
    static std::string classes = sofa::core::ObjectFactory::getInstance()->listClassesFromTarget(MODULE_NAME);
    return classes.c_str();
}
} // namespace SoftRobots

SOFA_LINK_CLASS(AnimationEditor)
SOFA_LINK_CLASS(DataVariationLimiter)
SOFA_LINK_CLASS(PartialRigidificationConstraint)
SOFA_LINK_CLASS(PartialRigidificationForceField)
SOFA_LINK_CLASS(PREquivalentStiffnessForceField)
SOFA_LINK_CLASS(SurfacePressureConstraint)
SOFA_LINK_CLASS(SerialPortBridgeGeneric)
SOFA_LINK_CLASS(InteractiveControl)
SOFA_LINK_CLASS(UnilateralPlaneConstraint)
SOFA_LINK_CLASS(VolumeFromTriangles)
SOFA_LINK_CLASS(VolumeFromTetrahedrons)

