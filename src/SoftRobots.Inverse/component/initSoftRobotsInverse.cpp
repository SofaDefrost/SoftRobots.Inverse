/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: Christian Duriez                                                   *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#include <sofa/core/ObjectFactory.h>
#include <sofa/helper/system/PluginManager.h>
using sofa::helper::system::PluginManager;

#include <SoftRobots.Inverse/component/initSoftRobotsInverse.h>


namespace softrobotsinverse
{

namespace constraint {
    extern void registerBarycentricCenterEffector(sofa::core::ObjectFactory* factory);
    extern void registerCableActuator(sofa::core::ObjectFactory* factory);
    extern void registerCableEffector(sofa::core::ObjectFactory* factory);
    extern void registerCableEquality(sofa::core::ObjectFactory* factory);
    extern void registerCableSensor(sofa::core::ObjectFactory* factory);
    extern void registerForcePointActuator(sofa::core::ObjectFactory* factory);
    extern void registerForceSurfaceActuator(sofa::core::ObjectFactory* factory);
    extern void registerJointActuator(sofa::core::ObjectFactory* factory);
    extern void registerPositionEffector(sofa::core::ObjectFactory* factory);
    extern void registerPositionEquality(sofa::core::ObjectFactory* factory);
    extern void registerSlidingActuator(sofa::core::ObjectFactory* factory);
    extern void registerSurfacePressureActuator(sofa::core::ObjectFactory* factory);
    extern void registerSurfacePressureEquality(sofa::core::ObjectFactory* factory);
    extern void registerSurfacePressureSensor(sofa::core::ObjectFactory* factory);
    extern void registerVolumeEffector(sofa::core::ObjectFactory* factory);
    extern void registerYoungModulusActuator(sofa::core::ObjectFactory* factory);
}

namespace solver {
    extern void registerQPInverseProblemSolver(sofa::core::ObjectFactory* factory);
}

//Here are just several convenient functions to help users know what the plugin contains

extern "C" {
    SOFA_SOFTROBOTS_INVERSE_API void initExternalModule();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleName();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleVersion();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleLicense();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleDescription();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleComponentList();
    SOFA_SOFTROBOTS_INVERSE_API void registerObjects(sofa::core::ObjectFactory* factory);
}


void init()
{
    static bool first = true;
    if (first)
    {
        first = false;
    }

    // make sure that this plugin is registered into the PluginManager
    PluginManager::getInstance().registerPlugin(MODULE_NAME);
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
    return "AGPL-v3";
}

const char* getModuleDescription()
{
    return "The plugin allows to control soft robots through inverse simulation.";
}

const char* getModuleComponentList()
{
    /// string containing the names of the classes provided by the plugin
    static std::string classes = sofa::core::ObjectFactory::getInstance()->listClassesFromTarget(sofa_tostring(MODULE_NAME));
    return classes.c_str();
}

void registerObjects(sofa::core::ObjectFactory* factory)
{
    constraint::registerBarycentricCenterEffector(factory);
    constraint::registerCableActuator(factory);
    constraint::registerCableEffector(factory);
    constraint::registerCableEquality(factory);
    constraint::registerCableSensor(factory);
    constraint::registerForcePointActuator(factory);
    constraint::registerForceSurfaceActuator(factory);
    constraint::registerJointActuator(factory);
    constraint::registerPositionEffector(factory);
    constraint::registerPositionEquality(factory);
    constraint::registerSlidingActuator(factory);
    constraint::registerSurfacePressureActuator(factory);
    constraint::registerSurfacePressureEquality(factory);
    constraint::registerSurfacePressureSensor(factory);
    constraint::registerVolumeEffector(factory);
    constraint::registerYoungModulusActuator(factory);
    solver::registerQPInverseProblemSolver(factory);
}

} // namespace softrobotsinverse


