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

#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALSETCONSTRAINT_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPMECHANICALSETCONSTRAINT_CPP

#include <SoftRobots.Inverse/component/solver/modules/QPMechanicalSetConstraint.h>

namespace softrobotsinverse::solver::module
{

using softrobots::behavior::SoftRobotsBaseConstraint ;
using sofa::core::behavior::BaseConstraint ;
using sofa::helper::system::thread::CTime ;
using sofa::type::vector;
using sofa::core::MultiMatrixDerivId ;
using sofa::core::ConstraintParams ;
using sofa::core::BaseMapping ;
using sofa::simulation::Node ;
using sofa::simulation::Visitor ;

QPMechanicalSetConstraint::QPMechanicalSetConstraint(const ConstraintParams* cparams,
                                                     MultiMatrixDerivId res,
                                                     unsigned int &constraintId,
                                                     QPInverseProblem* currentCP)
    : sofa::simulation::BaseMechanicalVisitor(cparams)
    , m_res(res)
    , m_constraintId(constraintId)
    , m_cparams(cparams)
    , m_currentCP(currentCP)
{
#ifdef SOFA_DUMP_VISITOR_INFO
    setReadWriteVectors();
#endif
}

Visitor::Result QPMechanicalSetConstraint::fwdConstraintSet(Node* node, sofa::core::behavior::BaseConstraintSet* c)
{
    ctime_t t0 = begin(node, c);

    unsigned int index = m_constraintId;
    QPInverseProblem::QPConstraintLists* qpCLists = m_currentCP->getQPConstraintLists();

    c->buildConstraintMatrix(m_cparams, m_res, m_constraintId);

    SoftRobotsBaseConstraint* ipc = dynamic_cast<SoftRobotsBaseConstraint*>(c);
    BaseConstraint* cc = dynamic_cast<BaseConstraint*>(c);

    if(ipc && ipc->m_constraintType == ipc->ACTUATOR)
    {
        unsigned int nbLines = m_constraintId - index;
        for(unsigned int i=0; i<nbLines; i++)
            (qpCLists->actuatorRowIds).push_back(index + i);

        (qpCLists->actuators).push_back(ipc);
    }

    if(ipc && ipc->m_constraintType == ipc->EFFECTOR)
    {
        unsigned int nbLines = m_constraintId - index;
        for(unsigned int i=0; i<nbLines; i++)
            (qpCLists->effectorRowIds).push_back(index + i);

        (qpCLists->effectors).push_back(ipc);
    }

    if(ipc && ipc->m_constraintType == ipc->SENSOR)
    {
        unsigned int nbLines = m_constraintId - index;
        for(unsigned int i=0; i<nbLines; i++)
            (qpCLists->sensorRowIds).push_back(index + i);

        (qpCLists->sensors).push_back(ipc);
    }

    if(ipc && ipc->m_constraintType == ipc->EQUALITY)
    {
        unsigned int nbLines = m_constraintId - index;
        for(unsigned int i=0; i<nbLines; i++)
            (qpCLists->equalityRowIds).push_back(index + i);
        (qpCLists->equality).push_back(ipc);

    }

    if(!ipc && cc)
    {
        unsigned int nbLines = m_constraintId - index;
        for(unsigned int i=0; i<nbLines; i++)
            (qpCLists->contactRowIds).push_back(index + i);

        (qpCLists->contacts).push_back(cc);
    }

    end(node, c, t0);
    return RESULT_CONTINUE;
}

const char* QPMechanicalSetConstraint::getClassName() const
{
    return "QPMechanicalSetConstraint";
}

bool QPMechanicalSetConstraint::isThreadSafe() const
{
    return false;
}

bool QPMechanicalSetConstraint::stopAtMechanicalMapping(Node* node, BaseMapping* map)
{
    SOFA_UNUSED(node);
    SOFA_UNUSED(map);

    return false;
}

#ifdef SOFA_DUMP_VISITOR_INFO
void QPMechanicalSetConstraint::setReadWriteVectors()
{
}
#endif

} // namespace

#endif
