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
#ifndef SOFTROBOTS_INVERSE_CONSTRAINTHANDLER_CPP
#define SOFTROBOTS_INVERSE_CONSTRAINTHANDLER_CPP

#include <SoftRobots.Inverse/component/solver/modules/ConstraintHandler.h>

namespace softrobotsinverse::solver::module {

using sofa::type::vector;
using softrobots::behavior::SoftRobotsBaseConstraint;
using sofa::helper::rabs;


void ConstraintHandler::buildInequalityConstraintMatrices(const vector<double> &result,
                                                          QPInverseProblem::QPSystem* qpSystem,
                                                          QPInverseProblem::QPConstraintLists* qpCLists)
{
    bool error = checkCListsConsistency(qpCLists);
    if(error)
        return;

    /// Problem:
    ///     minimize(1/2 lambdaT*Q*lambda + cT*lambda)
    ///     s.t     A*lambda <= bu
    ///
    ///  with A  = ( Waa  Wac)     constraint on actuator delta_a <= delta_a_max
    ///            (-Wca -Wcc)     Signorini constraint   delta_c >= 0
    ///
    ///       bu = (delta_a_max - delta_free_a)
    ///            (     delta_free_c       )

    qpSystem->A.clear();
    qpSystem->bu.clear();
    qpSystem->bl.clear();

    unsigned int nbActuatorRows   = qpCLists->actuatorRowIds.size();
    unsigned int nbEqualityRows   = qpCLists->equalityRowIds.size();

    for (unsigned int i=0; i<qpCLists->actuators.size(); i++)
        if((qpCLists->actuators[i]->hasDeltaMax() && qpCLists->actuators[i]->hasDeltaMin())
                || (qpCLists->actuators[i]->hasLambdaMax() && qpCLists->actuators[i]->hasLambdaMin())){
            qpSystem->hasBothSideInequalityConstraint = true;}

    if(m_qpCParams->hasMinContactForces || m_qpCParams->hasMaxContactForces)
        addContactLimits(qpSystem, qpCLists);
    if(m_qpCParams->hasMinContactForces && m_qpCParams->hasMaxContactForces)
        qpSystem->hasBothSideInequalityConstraint = true;

    int actuatorsId = 0;
    for (unsigned int i=0; i<qpSystem->dim;)
    {
        vector<double> row;
        if (i<nbActuatorRows)
        {
            SoftRobotsBaseConstraint* ac;
            ac = qpCLists->actuators[actuatorsId];
            actuatorsId++;
            int nbLines = ac->getNbLines();
            if(ac->hasDeltaMax())
            {
                for (int k=0; k<nbLines; k++)
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int j=0; j<qpSystem->dim; j++) // rows corresponding to (Waa Wac)
                    {
                        if (j<nbActuatorRows)                      row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)  row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                       row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }

                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);

                    qpSystem->bu.push_back(ac->getDeltaMax(k) - qpSystem->dFree[qpCLists->actuatorRowIds[i+k]]); // (delta_max - delta_free_a)

                    if(ac->hasDeltaMin()) // lambda_min <= A*lambda <= lambda_max
                        qpSystem->bl.push_back(ac->getDeltaMin(k) - qpSystem->dFree[qpCLists->actuatorRowIds[i+k]]); // (delta_min - delta_free_a)
                    else if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not -> Set -1e99 <= A*lambda <= lambda_max
                        qpSystem->bl.push_back(-1e99);
                }
            }
            else if(ac->hasDeltaMin())
            {
                if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not -> Set lambda_min <= A*lambda <= 1e99
                {
                    for (int k=0; k<nbLines; k++)
                    {
                        row.clear();
                        row.resize(qpSystem->dim, 0.);

                        for (unsigned int j=0; j<qpSystem->dim; j++) // rows corresponding to (Waa Wac)
                        {
                            if (j<nbActuatorRows)                        row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->actuatorRowIds[j]];
                            else if (j<nbActuatorRows+nbEqualityRows)    row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                            else                                         row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                        }

                        qpSystem->A.push_back(row);
                        m_qpCParams->constraintsId.push_back(i);
                        qpSystem->bl.push_back(ac->getDeltaMin(k) - qpSystem->dFree[qpCLists->actuatorRowIds[i+k]]); // (delta_min - delta_free_a)
                        qpSystem->bu.push_back(1e99);
                    }
                }
                else
                {
                    for (int k=0; k<nbLines; k++)
                    {
                        row.clear();
                        row.resize(qpSystem->dim, 0.);

                        for (unsigned int j=0; j<qpSystem->dim; j++)
                        {
                            if (j<nbActuatorRows)                        row[j] = -qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->actuatorRowIds[j]];
                            else if (j<nbActuatorRows+nbEqualityRows)    row[j] = -qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                            else                                         row[j] = -qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];

                        }

                        qpSystem->A.push_back(row);
                        m_qpCParams->constraintsId.push_back(i);
                        qpSystem->bu.push_back(-ac->getDeltaMin(k) + qpSystem->dFree[qpCLists->actuatorRowIds[i+k]]);
                    }
                }
            }
            i+=nbLines;
        }
        else if (i<nbActuatorRows+nbEqualityRows) {
            SoftRobotsBaseConstraint* ec;
            int equalityId = 0;
            ec = qpCLists->equality[equalityId];
            equalityId++;
            int nbLines = ec->getNbLines();
            i+=nbLines;
        }
        else
        {
            if(m_qpCParams->mu>0.0)
            {
                int contactId = (i-nbActuatorRows-nbEqualityRows)/m_qpCParams->contactNbLines;
                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->inactiveContact)// delta_n >= 0
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    // rows corresponding to (-Wca -Wcc)
                    for (unsigned int j=0; j<qpSystem->dim; j++)
                    {
                        if (j<nbActuatorRows)                     row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows) row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                      row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);

                    // ( delta_free_c )
                    qpSystem->bu.push_back(qpSystem->dFree[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]]);

                    if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not
                        qpSystem->bl.push_back(-1e99);
                }

                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->stickContact && m_qpCParams->allowSliding)// ||lambda_o|| + ||lambda_t|| <= lambda_n*m_qpCParams->m_mu
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    row[i]   = -m_qpCParams->mu;
                    row[i+1] = 1.;
                    row[i+2] = 1.;
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i+1);

                    row[i+1] = -1.;
                    row[i+2] = -1.;
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i+1);

                    row[i+1] = 1.;
                    row[i+2] = -1.;
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i+1);

                    row[i+1] = -1.;
                    row[i+2] = 1.;
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i+1);

                    for(int k=0; k<4; k++)
                    {
                        qpSystem->bu.push_back(0.);

                        if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not
                            qpSystem->bl.push_back(-1e99);
                    }
                }

                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->slidingContact)
                {
                    for (unsigned int k=1; k<m_qpCParams->contactNbLines; k++)
                    {
                        if(k==m_qpCParams->slidingDirId1 || k==m_qpCParams->slidingDirId2)
                        {
                            double sign = 1.;
                            if(result[i+k]>0)
                                sign = -1.;
                            else if(rabs(result[i+k])<1e-13 && qpSystem->previousResult.size()!=0 && qpSystem->previousResult[i+k]>0)
                                sign = -1.;

                            row.clear();
                            row.resize(qpSystem->dim, 0.);

                            for (unsigned int j=0; j<qpSystem->dim; j++)
                            {
                                if (j<nbActuatorRows)                      row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows+k]][qpCLists->actuatorRowIds[j]]*sign;
                                else if (j<nbActuatorRows+nbEqualityRows)  row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows+k]][qpCLists->equalityRowIds[j-nbActuatorRows]]*sign;
                                else                                       row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows+k]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]]*sign;
                            }
                            qpSystem->A.push_back(row);
                            m_qpCParams->constraintsId.push_back(i+k);

                            qpSystem->bu.push_back(qpSystem->dFree[qpCLists->contactRowIds[i-nbActuatorRows+k-nbEqualityRows]]*sign);

                            if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not
                                qpSystem->bl.push_back(-1e99);
                        }
                    }
                }
            }
            else
            {
                if(m_qpCParams->contactStates[i-nbActuatorRows]==&m_qpCParams->inactiveContact)// delta_c >= 0
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    // rows corresponding to (-Wca -Wcc)
                    for (unsigned int j=0; j<qpSystem->dim; j++)
                    {
                        if (j<nbActuatorRows)                       row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)   row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                        row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }
                    qpSystem->A.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);

                    // ( delta_free_c )
                    qpSystem->bu.push_back(qpSystem->dFree[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]]);

                    if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not
                        qpSystem->bl.push_back(-1e99);
                }
            }

            i+=m_qpCParams->contactNbLines;
        }
    }

}


void ConstraintHandler::addContactLimits(QPInverseProblem::QPSystem* qpSystem,
                                              QPInverseProblem::QPConstraintLists* qpCLists)
{
    vector<double> contactLimitRow;
    contactLimitRow.resize(qpSystem->dim, 0.);

    int nbContacts  = qpCLists->contactRowIds.size();
    int nbActuators = qpCLists->actuatorRowIds.size();

    double value = 1;
    if(!m_qpCParams->hasMaxContactForces)
        value = -1;

    int nbActiveContact = 0;
    for(int i=0; i<nbContacts; i+=m_qpCParams->contactNbLines)
    {
        int contactId = i/m_qpCParams->contactNbLines;
        if(m_qpCParams->contactStates[contactId]!=&m_qpCParams->inactiveContact)
        {
            contactLimitRow[nbActuators+i] = value; // normal direction
            nbActiveContact++;
        }
    }

    if(nbActiveContact<2)
        return;

    qpSystem->A.push_back(contactLimitRow);

    if (m_qpCParams->hasMaxContactForces){
        qpSystem->bu.push_back(m_qpCParams->maxContactForces);
        if (m_qpCParams->hasMinContactForces)
            qpSystem->bl.push_back(m_qpCParams->minContactForces);
        else if(qpSystem->hasBothSideInequalityConstraint)
            qpSystem->bl.push_back(-1e99);
    }
    else{
        qpSystem->bu.push_back(-m_qpCParams->minContactForces);
        if(qpSystem->hasBothSideInequalityConstraint) // One constraint has both side constraint and this one has not
            qpSystem->bl.push_back(-1e99);
    }

    m_qpCParams->constraintsId.push_back(0); // Corresponds to no optimization variable in particular, but at least specify that this constraint is not for pivot
}


void ConstraintHandler::buildEqualityConstraintMatrices(const vector<double> &result,
                                                        QPInverseProblem::QPSystem* qpSystem,
                                                        QPInverseProblem::QPConstraintLists* qpCLists)
{
    bool error = checkCListsConsistency(qpCLists);
    if(error)
        return;

    qpSystem->Aeq.clear();
    qpSystem->beq.clear();
    unsigned int nbActuatorRows   = qpCLists->actuatorRowIds.size();
    unsigned int nbEqualityRows   = qpCLists->equalityRowIds.size();

    int actuatorsId = 0; // Counter allowing multiple lines for actuator
    int equalityId = 0;
    for (unsigned int i=0; i<qpSystem->dim;)
    {
        vector<double> row;

        if (i<nbActuatorRows)
        {
            SoftRobotsBaseConstraint* ac;
            ac = qpCLists->actuators[actuatorsId];
            actuatorsId++;
            int nbLines = ac->getNbLines();
            if(ac->hasLambdaEqual())
            {
                for (int k=0; k<nbLines; k++)
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    row[i+k] = 1;
                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(ac->getLambdaEqual(k));
                }
            }

            if(ac->hasDeltaEqual())
            {
                for (int k=0; k<nbLines; k++)
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int j=0; j<qpSystem->dim; j++) // rows corresponding to (Waa Wac)
                    {
                        if (j<nbActuatorRows)                       row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)   row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                        row[j] = qpSystem->W[qpCLists->actuatorRowIds[i+k]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }

                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(ac->getDeltaEqual(k) - qpSystem->dFree[qpCLists->actuatorRowIds[i+k]]); // (delta_min - delta_free_a)
                }
            }
            i+=nbLines;
        }
        else if (i < nbActuatorRows+nbEqualityRows)
        {
            SoftRobotsBaseConstraint* ac;
            ac = qpCLists->equality[equalityId];
            equalityId++;

            int nbLines = ac->getNbLines();
            if(ac->hasLambdaEqual())
            {
                for (int k=0; k<nbLines; k++)
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    row[i+k] = 1;
                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(ac->getLambdaEqual(k));
                }
            }

            if(ac->hasDeltaEqual())
            {
                for (int k=0; k<nbLines; k++)
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int j=0; j<qpSystem->dim; j++) // rows corresponding to (Waa Wac)
                    {
                        if (j<nbActuatorRows)                      row[j] = qpSystem->W[qpCLists->equalityRowIds[i+k-nbActuatorRows]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)  row[j] = qpSystem->W[qpCLists->equalityRowIds[i+k-nbActuatorRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                       row[j] = qpSystem->W[qpCLists->equalityRowIds[i+k-nbActuatorRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }
                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(ac->getDeltaEqual(k) - qpSystem->dFree[qpCLists->equalityRowIds[i+k-nbActuatorRows]]); // beq = (delta_eq - delta_free_a)
                }
            }
            i+=nbLines;
        }
        else
        {
            if(m_qpCParams->mu>0.)
            {
                int contactId = (i-nbActuatorRows-nbEqualityRows)/m_qpCParams->contactNbLines;
                if(m_qpCParams->contactStates[contactId]!=&m_qpCParams->inactiveContact)// delta_n = 0
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int j=0; j<qpSystem->dim; j++)
                    {
                        if (j<nbActuatorRows)                      row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)  row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                       row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }
                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(qpSystem->dFree[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]]);
                }

                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->stickContact)// delta_o = delta_t = 0
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int k=1; k<3; k++)
                    {
                        for (unsigned int j=0; j<qpSystem->dim; j++)
                        {
                            if (j<nbActuatorRows)                      row[j] = -qpSystem->W[qpCLists->contactRowIds[i+k-nbActuatorRows-nbEqualityRows]][qpCLists->actuatorRowIds[j]];
                            else if (j<nbActuatorRows+nbEqualityRows)  row[j] = -qpSystem->W[qpCLists->contactRowIds[i+k-nbActuatorRows-nbEqualityRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                            else                                       row[j] = -qpSystem->W[qpCLists->contactRowIds[i+k-nbActuatorRows-nbEqualityRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                        }

                        qpSystem->Aeq.push_back(row);
                        m_qpCParams->constraintsId.push_back(i+k);
                        qpSystem->beq.push_back(qpSystem->dFree[qpCLists->contactRowIds[i+k-nbActuatorRows-nbEqualityRows]]);
                    }
                }

                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->slidingContact)
                {
                    // ||lambda_o|| + ||lambda_t|| = lambda_n*m_qpCParams->m_mu
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    row[i]   = -m_qpCParams->mu;
                    row[i+m_qpCParams->slidingDirId1] = 1;
                    row[i+m_qpCParams->slidingDirId2] = 1;

                    if(result[i+m_qpCParams->slidingDirId1]<0)
                        row[i+m_qpCParams->slidingDirId1] = -1;
                    else if(rabs(result[i+m_qpCParams->slidingDirId1])<1e-13 && qpSystem->previousResult.size()!=0 && qpSystem->previousResult[i+m_qpCParams->slidingDirId1]<0)
                        row[i+m_qpCParams->slidingDirId1] = -1;

                    if(result[i+m_qpCParams->slidingDirId2]<0)
                        row[i+m_qpCParams->slidingDirId2] = -1;
                    else if(rabs(result[i+m_qpCParams->slidingDirId2])<1e-13 && qpSystem->previousResult.size()!=0 && qpSystem->previousResult[i+m_qpCParams->slidingDirId2]<0)
                        row[i+m_qpCParams->slidingDirId2] = -1;

                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i+1); // TODO
                    qpSystem->beq.push_back(0.0);
                }
            }
            else
            {
                if(m_qpCParams->contactStates[i-nbActuatorRows]==&m_qpCParams->activeContact)// delta_c = 0
                {
                    row.clear();
                    row.resize(qpSystem->dim, 0.);

                    for (unsigned int j=0; j<qpSystem->dim; j++)
                    {
                        if (j<nbActuatorRows)                      row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->actuatorRowIds[j]];
                        else if (j<nbActuatorRows+nbEqualityRows)  row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->equalityRowIds[j-nbActuatorRows]];
                        else                                       row[j] = -qpSystem->W[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]][qpCLists->contactRowIds[j-nbActuatorRows-nbEqualityRows]];
                    }
                    qpSystem->Aeq.push_back(row);
                    m_qpCParams->constraintsId.push_back(i);
                    qpSystem->beq.push_back(qpSystem->dFree[qpCLists->contactRowIds[i-nbActuatorRows-nbEqualityRows]]);
                }
            }
            i+=m_qpCParams->contactNbLines;
        }
    }
}



void ConstraintHandler::getConstraintOnLambda(const vector<double> &result,
                                              QPInverseProblem::QPSystem* qpSystem,
                                              QPInverseProblem::QPConstraintLists* qpCLists)
{
    int dim = qpSystem->Q.size(); // Different than m_system->m_dim in case of friction with sliding contacts
    int nbActuatorRows = qpCLists->actuatorRowIds.size();
    int nbEqualityRows = qpCLists->equalityRowIds.size();

    qpSystem->u.resize(qpSystem->dim);
    qpSystem->l.resize(qpSystem->dim);

    int actuatorsId = 0;
    int equalityId = 0;
    for (int i=0; i<dim;)
    {
        if (i<nbActuatorRows)
        {
            SoftRobotsBaseConstraint* ac;
            ac = qpCLists->actuators[actuatorsId];
            actuatorsId++;

            int nbLines = ac->getNbLines();
            if(ac->hasLambdaMax()) // lambda_a <= lambda_max
            {
                for (int k=0; k<nbLines; k++)
                {
                    qpSystem->u[i+k] = ac->getLambdaMax(k);

                    if(ac->hasLambdaMin()) // lambda_min <= lambda_a <= lambda_max
                        qpSystem->l[i+k] = ac->getLambdaMin(k);
                    else
                        qpSystem->l[i+k] = -1e99;
                }
            }
            else if(ac->hasLambdaMin()) // lambda_min <= lambda_a
            {
                for (int k=0; k<nbLines; k++)
                {
                    qpSystem->u[i+k] = 1e99;
                    qpSystem->l[i+k] = ac->getLambdaMin(k);
                }
            }
            else
            {
                for (int k=0; k<nbLines; k++)
                {
                    qpSystem->u[i+k] = 1e99;
                    qpSystem->l[i+k] = -1e99;
                }
            }
            i+=nbLines;
        }
        else if (i < nbActuatorRows+nbEqualityRows)
        {
            SoftRobotsBaseConstraint* ac;
            ac = qpCLists->equality[equalityId];
            equalityId++;
            int nbLines = ac->getNbLines();
            if(ac->hasLambdaEqual()){
                for (int k=0; k<nbLines; k++)
                {
                    qpSystem->u[i+k] = ac->getLambdaEqual(k);
                    qpSystem->l[i+k] = ac->getLambdaEqual(k);
                }
            }else {
                for (int k=0; k<nbLines; k++)
                {
                    qpSystem->u[i+k] = 1e99;
                    qpSystem->l[i+k] = -1e99;
                }
            }
            i+=nbLines;
        }
        else
        {
            if(m_qpCParams->mu>0.0)
            {
                int contactId = (i-nbActuatorRows-nbEqualityRows)/m_qpCParams->contactNbLines;
                if(m_qpCParams->contactStates[contactId]!=&m_qpCParams->inactiveContact) // lambda_n >= 0
                {
                    qpSystem->u[i] = 1e99;
                    qpSystem->l[i] = 0.;
                }

                if(m_qpCParams->contactStates[contactId]==&m_qpCParams->inactiveContact) // lambda_c = 0
                {
                    qpSystem->u[i] = 0.;
                    qpSystem->l[i] = 0.;

                    qpSystem->u[i+1] = 0.;
                    qpSystem->l[i+1] = 0.;

                    qpSystem->u[i+2] = 0.;
                    qpSystem->l[i+2] = 0.;
                }

                else if(m_qpCParams->contactStates[contactId]==&m_qpCParams->slidingContact) // lambda_ti <=/>= 0 and lambda_ti <=/>= 0 other directions are zero
                {
                    for (unsigned int j=1; j<m_qpCParams->contactNbLines; j++)
                    {
                        if(j==m_qpCParams->slidingDirId1 || j==m_qpCParams->slidingDirId2)
                        {
                            if(result[i+j]<0)
                            {
                                qpSystem->u[i+j] = 0.;
                                qpSystem->l[i+j] = -1e99;
                            }
                            else if(rabs(result[i+j])<1e-13 && qpSystem->previousResult.size()!=0 && qpSystem->previousResult[i+j]<0)
                            {
                                qpSystem->u[i+j] = 0.;
                                qpSystem->l[i+j] = -1e99;
                            }
                            else
                            {
                                qpSystem->u[i+j] = 1e99;
                                qpSystem->l[i+j] = 0.;
                            }
                        }
                        else
                        {
                            qpSystem->u[i+j] = 0.;
                            qpSystem->l[i+j] = 0.;
                        }
                    }
                }

                else // Stick, no constraint on lambda_o and lambda_t other directions are zero
                {
                    for(unsigned int k=1; k<m_qpCParams->contactNbLines; k++)
                    {
                        if(k==1 || k==2)
                        {
                            qpSystem->u[i+k] = 1e99;
                            qpSystem->l[i+k] = -1e99;
                        }
                        else
                        {
                            qpSystem->u[i+k] = 0.;
                            qpSystem->l[i+k] = 0.;
                        }

                    }
                }
            }
            else
            {
                if(m_qpCParams->contactStates[i-nbActuatorRows-nbEqualityRows]==&m_qpCParams->activeContact) // lambda_c >= 0
                {
                    qpSystem->u[i] = 1e99;
                    qpSystem->l[i] = 0.;
                }
                else // lambda_c = 0
                {
                    qpSystem->u[i] = 0.;
                    qpSystem->l[i] = 0.;
                }
            }

            i+=m_qpCParams->contactNbLines;
        }
    }
}


bool ConstraintHandler::checkCListsConsistency(QPInverseProblem::QPConstraintLists* qpCLists)
{
    bool error = false;

    // Check contact lists. If contacts occured, should be true:
    // Case frictionless: m_qpCParams->contactStates.size()==qpCLists->contactRowIds.size()
    // Case friction: m_qpCParams->contactStates.size()==qpCLists->contactRowIds.size()/m_qpCParams->contactNbLines

    if(qpCLists->contactRowIds.size()>0) // If contacts occured
    {
        if(m_qpCParams->mu > 0 && m_qpCParams->contactStates.size()!=qpCLists->contactRowIds.size()/m_qpCParams->contactNbLines)
            error = true;

        if(m_qpCParams->mu <= 0 && m_qpCParams->contactStates.size()!=qpCLists->contactRowIds.size())
            error = true;
    }

    if(error)
        dmsg_error("ConstraintHandler") << "Problem with qpCLists consistency";

    return error;
}


void ConstraintHandler::checkAndUpdateActuatorConstraints(const vector<double> &result,
                                                          QPInverseProblem::QPSystem* qpSystem,
                                                          QPInverseProblem::QPConstraintLists* qpCLists)
{
    vector<bool> isInfeasible = getInfeasibleConstraints(result, qpSystem, qpCLists);

    for(unsigned int i=0; i<isInfeasible.size(); i++)
    {
        if(isInfeasible[i])
        {
            // If a constraint on actuator is infeasible we should:
            int index = m_qpCParams->constraintsId[i];

            // 1- fix the current actuator force by adding a constraint (because removing the variable from the problem is to complex...)
            qpSystem->l[index] = 0.;
            qpSystem->u[index] = 0.;

            // 2- set bl and bu to -inf and inf (because removing the lines from A is to complex 'cause of the constraintsId list needed for pivot)
            if(qpSystem->hasBothSideInequalityConstraint)
                qpSystem->bl[i] = -1e99;
            qpSystem->bu[i] = 1e99;

            msg_warning("QPInverseProblemSolver") << "An actuator constraint could not be satisfied. Released the constraint. Constraint id = " << i;

        }
    }
}

vector<bool> ConstraintHandler::getInfeasibleConstraints(const vector<double> &x,
                                                         QPInverseProblem::QPSystem* qpSystem,
                                                         QPInverseProblem::QPConstraintLists* qpCLists)
{
    vector<bool> infeasibleIds;
    int nbConstraints = qpSystem->A.size()+qpSystem->Aeq.size();
    infeasibleIds.resize(nbConstraints, false);

    for (unsigned int i=0; i<qpSystem->A.size(); i++)
    {
        if(m_qpCParams->constraintsId[i]<(int)qpCLists->actuators.size()) // Constraint on actuator are set first in the constraint matrix A, only these constraints can be released.
        {
            double Ax_i = 0;
            for(unsigned int j=0; j<qpSystem->dim; j++)
                Ax_i += qpSystem->A[i][j]*x[j];

            if(Ax_i>qpSystem->bu[i] && rabs(Ax_i-qpSystem->bu[i])>1e-5)
                infeasibleIds[i]=true;

            if(qpSystem->hasBothSideInequalityConstraint)
                if(Ax_i<qpSystem->bl[i] && rabs(Ax_i-qpSystem->bl[i])>1e-5)
                    infeasibleIds[i]=true;
        }
    }

    return infeasibleIds;
}

void ConstraintHandler::initContactHandlers(const vector<double>&result,
                                            QPInverseProblem::QPConstraintLists* qpCLists)
{
    int nbContactRow   = qpCLists->contactRowIds.size();
    int nbActuatorRow  = qpCLists->actuatorRowIds.size();

    if(m_qpCParams->mu>0.) // Case friction
    {
        for(int i=0; i<nbContactRow; i+=m_qpCParams->contactNbLines)
        {
            double lambda_n = result[nbActuatorRow+i];
            double lambda_t = result[nbActuatorRow+i+1];
            double lambda_o = result[nbActuatorRow+i+2];

            int contactId = i/m_qpCParams->contactNbLines;
            if(rabs(lambda_n)<=1e-14 && rabs(lambda_t)<=1e-14 && rabs(lambda_o)<=1e-14)
            {
                m_qpCParams->contactStates[contactId] = &m_qpCParams->inactiveContact;
            }
            else if(rabs(rabs(lambda_t)+rabs(lambda_o)-m_qpCParams->mu*rabs(lambda_n))<1e-14)
            {
                if(m_qpCParams->allowSliding)
                    m_qpCParams->contactStates[contactId] = &m_qpCParams->slidingContact;
                else
                    m_qpCParams->contactStates[contactId] = &m_qpCParams->stickContact;
            }
            else
            {
                m_qpCParams->contactStates[contactId] = &m_qpCParams->stickContact;
            }
        }
    }
    else
    {
        for(int i=0; i<nbContactRow; i++)
            if(result[nbActuatorRow+i]>1e-14)
                m_qpCParams->contactStates[i] = &m_qpCParams->activeContact;
    }
}


void ConstraintHandler::initContactHandlers()
{
    m_qpCParams->contactStates.clear();
    m_qpCParams->contactStates.resize(m_qpCParams->nbContactPoints, &m_qpCParams->inactiveContact);
}


void ConstraintHandler::initContactHandlerList()
{
    m_qpCParams->allowedContactStates.clear();
    if(m_qpCParams->mu>0.)
    {
        m_qpCParams->allowedContactStates.resize(3);
        m_qpCParams->allowedContactStates[0] = &m_qpCParams->inactiveContact;
        m_qpCParams->allowedContactStates[1] = &m_qpCParams->stickContact;
        m_qpCParams->allowedContactStates[2] = &m_qpCParams->slidingContact;
    }
    else
    {
        m_qpCParams->allowedContactStates.resize(2);
        m_qpCParams->allowedContactStates[0] = &m_qpCParams->activeContact;
        m_qpCParams->allowedContactStates[1] = &m_qpCParams->inactiveContact;
    }

    for(unsigned int i=0; i<m_qpCParams->allowedContactStates.size(); i++)
        m_qpCParams->allowedContactStates[i]->setAllowSliding(m_qpCParams->allowSliding);
}

} // namespace

#endif
