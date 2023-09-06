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
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblem.h>

#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::solver::module
{

class SOFA_SOFTROBOTS_INVERSE_API QPMechanicalSetConstraint : public sofa::simulation::BaseMechanicalVisitor
{
public:
    QPMechanicalSetConstraint(const sofa::core::ConstraintParams* cparams,
                              sofa::core::MultiMatrixDerivId res,
                              unsigned int &contactId,
                              QPInverseProblem *currentCP) ;


    ////////////////////// Inherited from ConstraintSolverImpl ////////////////////////
    virtual Visitor::Result fwdConstraintSet(sofa::simulation::Node* node, sofa::core::behavior::BaseConstraintSet* c) ;
    virtual const char* getClassName() const ;
    virtual bool isThreadSafe() const ;
    virtual bool stopAtMechanicalMapping(sofa::simulation::Node* node, sofa::core::BaseMapping* map) ;
    /////////////////////////////////////////////////////////////////////////////////


#ifdef SOFA_DUMP_VISITOR_INFO
    void setReadWriteVectors() ;
#endif

protected:
    sofa::core::MultiMatrixDerivId m_res;
    unsigned int &m_constraintId;
    const sofa::core::ConstraintParams *m_cparams;

    QPInverseProblem* m_currentCP;

};

} // namespace
