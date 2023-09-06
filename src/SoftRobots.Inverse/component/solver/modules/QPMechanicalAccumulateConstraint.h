/******************************************************************************
*                 SOFA, Simulation Open-Framework Architecture                *
*                    (c) 2006 INRIA, USTL, UJF, CNRS, MGH                     *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
*                       Plugin SoftRobots.Inverse                             *
*                                                                             *
* This plugin is distributed under the GNU AGPL v3 (Affero General            *
* Public License) license.                                                    *
*                                                                             *
* Authors: Christian Duriez, Eulalie Coevoet, Yinoussa Adagolodjo             *
*                                                                             *
* (c) 2023 INRIA                                                              *
*                                                                             *
* Contact information: https://project.inria.fr/softrobot/contact/            *
******************************************************************************/

#pragma once

#include <sofa/component/constraint/lagrangian/solver/ConstraintSolverImpl.h>
#include <SoftRobots/component/behavior/SoftRobotsBaseConstraint.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::solver::module
{

class SOFA_SOFTROBOTS_INVERSE_API QPMechanicalAccumulateConstraint : public sofa::simulation::BaseMechanicalVisitor
{
public:
    QPMechanicalAccumulateConstraint(const sofa::core::ConstraintParams* _cparams,
                                     sofa::core::MultiMatrixDerivId _res,
                                     bool _reverseOrder = false) ;

    /// Return true to reverse the order of traversal of child nodes
    virtual bool childOrderReversed(sofa::simulation::Node* node) ;

    virtual void bwdMechanicalMapping(sofa::simulation::Node* node, sofa::core::BaseMapping* map) ;

    /// Return a class name for this visitor
    /// Only used for debugging / profiling purposes
    virtual const char* getClassName() const ;

    virtual bool isThreadSafe() const ;

    // This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
    virtual bool stopAtMechanicalMapping(sofa::simulation::Node* node, sofa::core::BaseMapping* map) ;

#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors() ;
#endif

protected:
    sofa::core::MultiMatrixDerivId m_res;
    const sofa::core::ConstraintParams *m_cparams;
    bool m_reverseOrder;
};

} // namespace
