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
#include <SoftRobots.Inverse/component/config.h>

#include <sofa/core/ObjectFactory.h>
#include <SoftRobots/component/initSoftRobots.h>


namespace softrobotsinverse
{

//Here are just several convenient functions to help users know what the plugin contains

extern "C" {
    SOFA_SOFTROBOTS_INVERSE_API void initExternalModule();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleName();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleVersion();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleLicense();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleDescription();
    SOFA_SOFTROBOTS_INVERSE_API const char* getModuleComponentList();
}

void initExternalModule()
{
    static bool first = true;
    if (first)
    {
        softrobots::init();
        first = false;
    }

}

const char* getModuleName()
{
    return MODULE_NAME;
}

const char* getModuleVersion()
{
    return "1.0";
}

const char* getModuleLicense()
{
    return "Private";
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

}

SOFA_LINK_CLASS(AnimationEditor)
SOFA_LINK_CLASS(BarycentricCenterEffector)
SOFA_LINK_CLASS(CableActuator)
SOFA_LINK_CLASS(CableEffector)
SOFA_LINK_CLASS(CableSensor)
SOFA_LINK_CLASS(CableEquality)
SOFA_LINK_CLASS(ForcePointActuator)
SOFA_LINK_CLASS(PartialRigidificationConstraint)
SOFA_LINK_CLASS(PartialRigidificationForceField)
SOFA_LINK_CLASS(PREquivalentStiffnessForceField)
SOFA_LINK_CLASS(PositionEffector)
SOFA_LINK_CLASS(QPInverseProblemSolver)
SOFA_LINK_CLASS(SurfacePressureConstraint)
SOFA_LINK_CLASS(SurfacePressureActuator)
SOFA_LINK_CLASS(SurfacePressureEquality)
SOFA_LINK_CLASS(YoungModulusActuator)
SOFA_LINK_CLASS(InteractiveControl)
SOFA_LINK_CLASS(UnilateralPlaneConstraint)
SOFA_LINK_CLASS(VolumeFromTriangles)
SOFA_LINK_CLASS(VolumeFromTetrahedrons)

