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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALACCUMULATECONSTRAINT_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALACCUMULATECONSTRAINT_CPP

#include <SoftRobots.Inverse/component/solver/modules/QPMechanicalAccumulateConstraint.h>
#include <sofa/core/BaseMapping.h>

#include <sofa/core/BaseMapping.h>

namespace softrobotsinverse::solver::module
{
using sofa::core::MultiMatrixDerivId ;
using sofa::core::ConstraintParams ;
using sofa::core::BaseMapping ;
using sofa::simulation::Node ;


QPMechanicalAccumulateConstraint::QPMechanicalAccumulateConstraint(const ConstraintParams* _cparams,
                                                                   MultiMatrixDerivId _res,
                                                                   bool _reverseOrder)
    : sofa::simulation::BaseMechanicalVisitor(_cparams)
    , m_res(_res)
    , m_cparams(_cparams)
    , m_reverseOrder(_reverseOrder)
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
}

/// Return true to reverse the order of traversal of child nodes
bool QPMechanicalAccumulateConstraint::childOrderReversed(Node*)
{
    return m_reverseOrder;
}

void QPMechanicalAccumulateConstraint::bwdMechanicalMapping(Node* node, BaseMapping* map)
{
    ctime_t t0 = begin(node, map);
    map->applyJT(m_cparams, m_res, m_res);
    end(node, map, t0);
}

/// Return a class name for this visitor
/// Only used for debugging / profiling purposes
const char* QPMechanicalAccumulateConstraint::getClassName() const
{
    return "QPMechanicalAccumulateConstraint";
}

bool QPMechanicalAccumulateConstraint::isThreadSafe() const
{
    return false;
}

// This visitor must go through all mechanical mappings, even if isMechanical flag is disabled
bool QPMechanicalAccumulateConstraint::stopAtMechanicalMapping(Node* node, BaseMapping* map)
{
    SOFA_UNUSED(node);
    SOFA_UNUSED(map);

    return false;
}

#ifdef SOFA_DUMP_VISITOR_INFO
void QPMechanicalAccumulateConstraint::setReadWriteVectors()
{
}
#endif

} // namespace

#endif

