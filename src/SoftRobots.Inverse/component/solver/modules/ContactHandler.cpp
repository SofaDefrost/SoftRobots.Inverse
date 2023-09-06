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
#ifndef SOFTROBOTS_INVERSE_CONTACTHANDLER_CPP
#define SOFTROBOTS_INVERSE_CONTACTHANDLER_CPP

#include "ContactHandler.h"
#include <sofa/helper/config.h>
#include <sofa/helper/rmath.h>
#include <iostream>

namespace softrobotsinverse::solver::module {

using sofa::helper::rabs;
using sofa::type::vector;

/******************************* ACTIVECONTACTHANDLER********************************************************/

bool ActiveContactHandler::hasReachedBoundary(const double &lambda, const double &delta)
{
    SOFA_UNUSED(delta);

    if(rabs(lambda) <= m_epsilon)
        return true;

    return false;
}

ContactHandler* ActiveContactHandler::getNewContactHandler(const vector<ContactHandler*>& handlerPtrList,
                                                           const double& lambda,
                                                           const double& delta )
{
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(delta);

    return handlerPtrList[1];
}


/******************************* INACTIVECONTACTHANDLER*******************************************************/

bool InactiveContactHandler::hasReachedBoundary(const double &lambda, const double &delta)
{
    SOFA_UNUSED(lambda);

    if(rabs(delta) <= m_epsilon)
        return true;

    return false;
}

ContactHandler* InactiveContactHandler::getNewContactHandler(const vector<ContactHandler*>& handlerPtrList,
                                                             const double& lambda,
                                                             const double& delta )
{
    SOFA_UNUSED(delta);
    SOFA_UNUSED(lambda);

    return handlerPtrList[0]; // ActiveContactHandler
}

bool InactiveContactHandler::hasReachedBoundary(const vector<double> &lambda, const vector<double> &delta, const double &mu, int &candidateId)
{
    SOFA_UNUSED(lambda);
    SOFA_UNUSED(mu);

    candidateId=0; // Blocking constraint is lambda_n

    if(rabs(delta[0]) <= m_epsilon)
        return true;

    return false;
}

ContactHandler* InactiveContactHandler::getNewContactHandler(const vector<ContactHandler*>& handlerPtrList,
                                                             const vector<double>& lambda,
                                                             const vector<double>& delta ,
                                                             const double& mu)
{
    SOFA_UNUSED(delta);

    if(m_allowSliding)
        if(rabs(rabs(lambda[1])+rabs(lambda[2])-mu*lambda[0])<=m_epsilon)
            return handlerPtrList[2]; // SlidingContactHandler

    return handlerPtrList[1]; // StickContactHandler
}





/******************************* STICKCONTACTHANDLER*******************************************************/

bool StickContactHandler::hasReachedBoundary(const vector<double> &lambda, const vector<double> &delta, const double &mu, int& candidateId)
{
    SOFA_UNUSED(delta);

    if(m_allowSliding)
    {
        if(rabs(rabs(lambda[1])+rabs(lambda[2])-mu*lambda[0]) <= m_epsilon)
        {
            candidateId=3; // Blocking constraint are lambda_t and lambda_o
            return true;
        }
    }

    if(rabs(lambda[0])<=m_epsilon/* && rabs(lambda[1])<=m_epsilon && rabs(lambda[2])<=m_epsilon*/)
    {
        candidateId=0; // Blocking constraint is lambda_n
        return true;
    }

    return false;
}

ContactHandler* StickContactHandler::getNewContactHandler(const vector<ContactHandler*>& handlerPtrList,
                                                          const vector<double>& lambda,
                                                          const vector<double>& delta ,
                                                          const double& mu)
{
    SOFA_UNUSED(delta);
    SOFA_UNUSED(mu);

    if(m_allowSliding)
        if(rabs(rabs(lambda[1])+rabs(lambda[2])-mu*lambda[0]) <= m_epsilon)
            return handlerPtrList[2]; // SlidingContactHandler

    return handlerPtrList[0]; // InactiveContactHandler
}



/******************************* SLIDINGCONTACTHANDLER*******************************************************/

bool SlidingContactHandler::hasReachedBoundary(const vector<double> &lambda, const vector<double> &delta, const double &mu, int &candidateId)
{
    SOFA_UNUSED(mu);

    if(rabs(lambda[0])<=m_epsilon)
    {
        candidateId=0; // Blocking constraint is lambda_n
        return true;
    }

    if(rabs(delta[1])<=m_epsilon && rabs(delta[2])<=m_epsilon)
    {
        candidateId=3; // Blocking constraint are lambda_t and lambda_o
        return true;
    }

    return false;
}

ContactHandler* SlidingContactHandler::getNewContactHandler(const vector<ContactHandler*>& handlerPtrList,
                                                             const vector<double>& lambda,
                                                             const vector<double>& delta ,
                                                             const double& mu)
{
    SOFA_UNUSED(mu);
    SOFA_UNUSED(delta);

    if(rabs(lambda[0])<=m_epsilon)
        return handlerPtrList[0]; // InactiveContactHandler

    return handlerPtrList[1]; // StickContactHandler
}


} // namespace

#endif
