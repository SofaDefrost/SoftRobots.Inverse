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
#include <SoftRobots.Inverse/component/config.h>

namespace softrobotsinverse::solver::module {

class SOFA_SOFTROBOTS_INVERSE_API ContactHandler
{
public:
    ContactHandler(){}
    virtual ~ContactHandler(){}

public:

    /// Return if the contact has reached is boundary, different implementation depending on contact state.
    /// Input: contact force, contact violation, radius=friction_coef*contact_normal_force
    /// Output: true if the contact has reached is boundary, else false
    virtual bool hasReachedBoundary(const sofa::type::vector<double>& lambda,
                                    const sofa::type::vector<double>& delta ,
                                    const double& mu ,
                                    int& candidateId)
    {
        SOFA_UNUSED(mu);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);
        SOFA_UNUSED(candidateId);

        return false;
    }

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta )
    {
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return false;
    }

    /// Find new contact state after pivot and return a pointer to the corresponding ContactHandler
    /// Input: Pointers on different ContactHandler {active, inactive} in case of frictionless or {inactive, stick, sliding} in case of friction
    ///        contact force, contact violation, radius=friction_coef*contact_normal_force
    /// Output: Pointer on new ContactHandler corresponding to new state after pivoting
    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const sofa::type::vector<double>& lambda,
                                                 const sofa::type::vector<double>& delta ,
                                                 const double& mu )
    {
        SOFA_UNUSED(mu);
        SOFA_UNUSED(ptrList);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return nullptr;
    }

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta )
    {
        SOFA_UNUSED(ptrList);
        SOFA_UNUSED(lambda);
        SOFA_UNUSED(delta);

        return nullptr;
    }

    void setAllowSliding(bool allowSliding) {m_allowSliding=allowSliding;}
    virtual std::string getStateString()=0;

protected:
    double m_epsilon{1e-14};
    double m_allowSliding{false};

};


class ActiveContactHandler : public ContactHandler
{
public:
    ActiveContactHandler(){}
    virtual ~ActiveContactHandler(){}

public:

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta ) override;

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta ) override;

    std::string getStateString() override {return "Active";}
};


class InactiveContactHandler : public ContactHandler
{
public:
    InactiveContactHandler(){}
    virtual ~InactiveContactHandler(){}

public:
    virtual bool hasReachedBoundary(const sofa::type::vector<double>& lambda,
                                    const sofa::type::vector<double>& delta ,
                                    const double& mu ,
                                    int& candidateId) override;

    virtual bool hasReachedBoundary(const double& lambda,
                                    const double& delta) override;

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const sofa::type::vector<double>& lambda,
                                                 const sofa::type::vector<double>& delta ,
                                                 const double& mu) override;

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const double& lambda,
                                                 const double& delta ) override;

    std::string getStateString() override {return "Inactive";}
};


class StickContactHandler : public ContactHandler
{
public:
    StickContactHandler(){}
    virtual ~StickContactHandler(){}

public:
    virtual bool hasReachedBoundary(const sofa::type::vector<double>& lambda,
                                    const sofa::type::vector<double>& delta ,
                                    const double& mu,
                                    int& candidateId) override;

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const sofa::type::vector<double>& lambda,
                                                 const sofa::type::vector<double>& delta ,
                                                 const double& mu) override;

    std::string getStateString() override {return "Stick";}
};


class SlidingContactHandler : public ContactHandler
{
public:
    SlidingContactHandler(){}
    virtual ~SlidingContactHandler(){}

public:
    virtual bool hasReachedBoundary(const sofa::type::vector<double>& lambda,
                                    const sofa::type::vector<double>& delta ,
                                    const double& mu,
                                    int& candidateId) override;

    virtual ContactHandler* getNewContactHandler(const sofa::type::vector<ContactHandler*>& ptrList,
                                                 const sofa::type::vector<double>& lambda,
                                                 const sofa::type::vector<double>& delta ,
                                                 const double& mu) override;

    std::string getStateString() override {return "Sliding";}
};

}

