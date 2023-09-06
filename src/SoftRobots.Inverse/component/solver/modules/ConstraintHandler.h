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

#include <sofa/type/vector.h>

#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblem.h>
#include <SoftRobots.Inverse/component/solver/modules/ContactHandler.h>
#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::solver::module {

using sofa::type::vector;

class SOFA_SOFTROBOTS_INVERSE_API ConstraintHandler
{
public:

    struct QPConstraintParams{

        vector<ContactHandler*>  contactStates; // State of contacts (size of #contacts points)
        vector<ContactHandler*>  allowedContactStates; // Possible states of contact
        ActiveContactHandler   activeContact;
        InactiveContactHandler inactiveContact;
        StickContactHandler    stickContact;
        SlidingContactHandler  slidingContact;

        unsigned int slidingDirId1; // The sliding force direction is along the convex combination of id1 and id2 directions
        unsigned int slidingDirId2; // The sliding force direction is along the convex combination of id1 and id2 directions

        unsigned int contactNbLines; // Number of variables for a contact (ex: 1 if frictionless)
        unsigned int nbContactPoints{0};

        vector<int>  constraintsId;  // In the final constraint matrix Af = [A ; Aeq], gives the id of the variables the constraint corresponds to. Size of Af.

        double mu{0}; // Friction coefficient
        bool allowSliding{false};

        double minContactForces{0};
        double maxContactForces{0};
        bool hasMinContactForces{false};
        bool hasMaxContactForces{false};
    };

    ConstraintHandler()
    {
        m_qpCParams = new QPConstraintParams;
    }
    virtual ~ConstraintHandler()
    {
        delete m_qpCParams;
    }

    /// Build constraints matrices A, Aeq, bu, bl, beq, u and l corresponding to the following system
    ///
    ///     min(1/2xQx + cx)
    ///
    ///     subject to:
    ///
    ///         bl<=Ax<=bu
    ///         Aeqx=beq
    ///         l<=x<=u


    /// Build constraints matrices A, bu, bl
    void buildInequalityConstraintMatrices(const vector<double> &result,
                                           QPInverseProblem::QPSystem* qpSystem,
                                           QPInverseProblem::QPConstraintLists* qpCLists);

    /// Build constraints matrices Aeq, beq
    void buildEqualityConstraintMatrices(const vector<double> &result,
                                         QPInverseProblem::QPSystem* qpSystem,
                                         QPInverseProblem::QPConstraintLists* qpCLists);

    /// Build constraints matrices u, l
    void getConstraintOnLambda(const vector<double> &result,
                               QPInverseProblem::QPSystem* qpSystem,
                               QPInverseProblem::QPConstraintLists* qpCLists);

    QPConstraintParams* getQPConstraintParams() {return m_qpCParams;}

    bool checkCListsConsistency(QPInverseProblem::QPConstraintLists* qpCLists);

    void checkAndUpdateActuatorConstraints(const vector<double> &result,
                                           QPInverseProblem::QPSystem* qpSystem,
                                           QPInverseProblem::QPConstraintLists* qpCLists);

    vector<bool> getInfeasibleConstraints(const vector<double> &result,
                                          QPInverseProblem::QPSystem* qpSystem,
                                          QPInverseProblem::QPConstraintLists* qpCLists);

    void initContactHandlers(const vector<double>&result,
                             QPInverseProblem::QPConstraintLists* qpCLists);
    void initContactHandlers();
    void initContactHandlerList();
    void addContactLimits(QPInverseProblem::QPSystem* qpSystem,
                               QPInverseProblem::QPConstraintLists* qpCLists);

protected:

    QPConstraintParams* m_qpCParams;
};

} //namespace

