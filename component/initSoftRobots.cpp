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
#include <SoftRobots/initSoftRobots.h>
#include <sofa/core/ObjectFactory.h>

#include <sofa/helper/system/PluginManager.h>
using sofa::helper::system::PluginManager;
using sofa::helper::system::Plugin;

#include <sofa/helper/system/DynamicLibrary.h>
using sofa::helper::system::DynamicLibrary;

#include <sofa/helper/system/FileSystem.h>
using sofa::helper::system::FileSystem;

#include <sofa/helper/Utils.h>
using sofa::helper::Utils;

#ifdef SOFTROBOTS_PYTHON
#include <SofaPython/PythonEnvironment.h>
using sofa::simulation::PythonEnvironment;
#endif

#include <fstream>

namespace sofa
{

namespace component
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

    /// Automatically load the SoftRobots.Inverse plugin if available.
    if( !PluginManager::getInstance().findPlugin("SoftRobots.Inverse").empty() )
    {
        PluginManager::getInstance().loadPlugin("SoftRobots.Inverse") ;
    }

#ifdef SOFTROBOTS_PYTHON
    typedef std::map<std::string, Plugin > PluginMap;
    typedef PluginMap::iterator PluginIterator;

    PluginMap&  map = PluginManager::getInstance().getPluginMap();
    for( const auto& elem : map)
    {
        Plugin p = elem.second;
        if ( p.getModuleName() == getModuleName() )
        {
            std::string modulePath = elem.first;
            modulePath.resize( modulePath.find(getModuleName() + std::string(".") + DynamicLibrary::extension) );
            modulePath = FileSystem::getParentDirectory( modulePath );
            std::cout << "modulePath = " << modulePath << std::endl;

            // APPROACH 1: Read python path from .ini file and add it to python environment
//            const std::string etcDir = modulePath + "/etc";
//            const std::string moduleIniFilePath = etcDir + "/" + getModuleName() + ".ini";
//            std::map<std::string, std::string> iniFileValues = Utils::readBasicIniFile(moduleIniFilePath);
//            if (iniFileValues.find("PYTHON_DIR") != iniFileValues.end())
//            {
//                std::string iniFileValue = iniFileValues["PYTHON_DIR"];
//                if (!FileSystem::isAbsolute(iniFileValue))
//                    iniFileValue = etcDir + "/" + iniFileValue;
//                PythonEnvironment::addPythonModulePathsFromConfigFile(iniFileValue);
//            }

            // APPROACH 2: Read python config file to get python module path
            // see PythonEnvironment::addPythonModulePathsFromConfigFile
            std::string configFilePath = modulePath + "/etc/sofa/python.d/" + getModuleName();
            std::ifstream configFile(configFilePath.c_str());
            std::string line;
            while(std::getline(configFile, line))
            {
                if (!FileSystem::isAbsolute(line))
                {
                    line = modulePath + "/" + line;
                }
                PythonEnvironment::addPythonModulePath(line);
            }
        }
    }
#endif
}

const char* getModuleName()
{
    return "SoftRobots";
}

const char* getModuleVersion()
{
    return "1.0";
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
    static std::string classes = sofa::core::ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(SOFA_TARGET));
    return classes.c_str();
}

}

}

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

