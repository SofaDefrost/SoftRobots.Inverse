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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMIMPL_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPINVERSEPROBLEMIMPL_CPP


#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>
#include <SoftRobots.Inverse/component/solver/modules/NLCPSolver.h>

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/component/collision/response/contact/CollisionResponse.h>
#include <iomanip>
#include <sstream>

namespace softrobotsinverse::solver::module
{

using softrobots::behavior::SoftRobotsBaseConstraint;
using sofa::type::vector;
using std::cout ;
using std::endl ;
using sofa::helper::rabs;
using std::string;
using std::istringstream;

using sofa::linearalgebra::FullVector;
using sofa::linearalgebra::LPtrFullMatrix;

using sofa::component::collision::response::contact::CollisionResponse;
using sofa::core::objectmodel::BaseContext;
using sofa::core::behavior::BaseConstraint;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::LDLT;

using sofa::helper::AdvancedTimer;


QPInverseProblemImpl::QPInverseProblemImpl()
    :    QPInverseProblem(),
    m_lcpQpSolver{nullptr}
{
    m_currentSequence.clear();
    m_previousSequence.clear();
    m_sequence.clear();

    m_constraintHandler = new ConstraintHandler();
    m_qpCParams = m_constraintHandler->getQPConstraintParams();
}


QPInverseProblemImpl::~QPInverseProblemImpl()
{
    delete m_constraintHandler;
}

void QPInverseProblemImpl::init(){

    m_step=0;
    unsigned int nbActuators = m_qpCLists->actuatorRowIds.size();
    int actuatorsId = 0;
    unsigned int actuatorLine = 0;

    m_qpSystem->lambda.clear();
    m_qpSystem->lambda.resize(m_qpSystem->dim);
    for (unsigned int k=0; k<m_qpSystem->dim; k++)
    {
        if(k<nbActuators) // actuators
        {
            SoftRobotsBaseConstraint* ac;
            ac = m_qpCLists->actuators[actuatorsId];
            if(ac->hasLambdaInit())
                m_qpSystem->lambda[k] = ac->getLambdaInit(actuatorLine);
            actuatorLine++;
            if(actuatorLine == ac->getNbLines())
            {
                actuatorLine = 0;
                actuatorsId++;
            }
        }
    }
}

/// Build system
void QPInverseProblemImpl:: computeEnergyWeight(double& weight)
{
    weight = 0.0;
    unsigned int nbActuators = m_qpCLists->actuatorRowIds.size();
    unsigned int nbEquality = m_qpCLists->equalityRowIds.size();

    vector<unsigned int> acIds;
    acIds.resize(m_qpSystem->dim);
    for (unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        if(i<nbActuators)                  acIds[i] = m_qpCLists->actuatorRowIds[i];  // actuators first
        else if(i<nbActuators+nbEquality)  acIds[i] = m_qpCLists->equalityRowIds[i-nbActuators]; //second
        else                               acIds[i] = m_qpCLists->contactRowIds[i-nbActuators-nbEquality]; // contacts last
    }

    double normQ = 0; // uniform norm
    for (unsigned int k=0; k<m_qpSystem->dim; k++)
    {
        double sum = 0;
        for(unsigned int j=0; j<m_qpSystem->dim; j++)
            sum += rabs(m_qpSystem->Q[k][j]);
        if (sum>normQ) normQ = sum;
    }

    // If not m_actuatorsOnly:
    // W_energy = (Waa Wac)
    //            (Wca Wcc)
    unsigned int dim = (m_actuatorsOnly)? nbActuators : m_qpSystem->dim;

    double normW = 0; // uniform norm
    for (unsigned int k=0; k<dim; k++)
    {
        double sum = 0;
        for(unsigned int j=0; j<dim; j++)
            sum += rabs(m_qpSystem->W[acIds[k]][acIds[j]]);
        if (sum>normW) normW = sum;
    }

    if(normW > 1e-14)
        weight = normQ/normW;
}


void QPInverseProblemImpl::buildQPMatrices()
{
    unsigned int nbActuators   = m_qpCLists->actuatorRowIds.size();
    unsigned int nbEffectors   = m_qpCLists->effectorRowIds.size();
    unsigned int nbEquality   = m_qpCLists->equalityRowIds.size();

    vector<unsigned int> acIds;
    acIds.resize(m_qpSystem->dim);
    for (unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        if(i<nbActuators)                 acIds[i] = m_qpCLists->actuatorRowIds[i]; // actuators first
        else if(i<nbActuators+nbEquality) acIds[i] = m_qpCLists->equalityRowIds[i-nbActuators]; // equality second
        else                              acIds[i] = m_qpCLists->contactRowIds[i-nbActuators-nbEquality ]; // contacts last
    }

    m_qpSystem->Q.clear();
    m_qpSystem->c.clear();

    m_qpSystem->Q.resize(m_qpSystem->dim);
    for(unsigned int j=0; j<m_qpSystem->dim; j++)
        m_qpSystem->Q[j].resize(m_qpSystem->dim);

    for (unsigned int k=0; k<m_qpSystem->dim; k++)
    {
        for(unsigned int j=0; j<m_qpSystem->dim; j++)
        {
            double sum = 0;
            for(unsigned int i=0; i<nbEffectors; i++)
            {
                unsigned int rowId  = m_qpCLists->effectorRowIds[i];
                unsigned int col1Id = acIds[j];
                unsigned int col2Id = acIds[k];
                sum += m_qpSystem->W[rowId][col1Id]* m_qpSystem->W[rowId][col2Id];
            }
            m_qpSystem->Q[k][j] = sum;
        }
    }

    m_qpSystem->c.resize(m_qpSystem->dim);
    for(unsigned int j=0; j<m_qpSystem->dim; j++)
    {
        m_qpSystem->c[j] = 0;
        for(unsigned int i=0; i<nbEffectors; i++)
            m_qpSystem->c[j] += m_qpSystem->W[m_qpCLists->effectorRowIds[i]][acIds[j]]*m_qpSystem->dFree[m_qpCLists->effectorRowIds[i]];
    }

    // Add energy term to Q+=eps*||Q||/||Waa||*Waa, eps is set by user
    double weight = 0.;
    computeEnergyWeight(weight); // compute ||Q||/||Waa||
    int actuatorsId = 0;
    int equalityId = 0;
    unsigned int actuatorsNbLines = 0;
    unsigned int equalitysNbLines = 0;

    // If not m_actuatorsOnly:
    // W_energy = (Waa Wac)
    //            (Wca Wcc)
    // Q+=eps*||Q||/||W_energy||*W_energy
    unsigned int dim = (m_actuatorsOnly)? nbActuators : m_qpSystem->dim;

    for (unsigned int k=0; k<dim; k++)
    {
        if(k<nbActuators) // actuators
        {
            SoftRobotsBaseConstraint* ac;
            ac = m_qpCLists->actuators[actuatorsId];
            actuatorsNbLines++;
            if(actuatorsNbLines == ac->getNbLines())
            {
                actuatorsNbLines = 0;
                actuatorsId++;
            }
            for(unsigned int j=0; j<dim; j++)
            {
                if(ac->hasEpsilon() && j==k) // energy of a specific actuator
                    m_qpSystem->Q[k][j] += ac->getEpsilon()*weight*m_qpSystem->W[acIds[k]][acIds[j]];
                else
                    m_qpSystem->Q[k][j] += m_epsilon*weight*m_qpSystem->W[acIds[k]][acIds[j]];
            }
        }
        else if (k<nbActuators+nbEquality)
        {
            SoftRobotsBaseConstraint* ac;
            ac = m_qpCLists->equality[equalityId];
            equalitysNbLines++;
            if(equalitysNbLines == ac->getNbLines())
            {
                equalitysNbLines = 0;
                equalityId++;
            }
            for(unsigned int j=0; j<dim; j++)
            {
                if(ac->hasEpsilon() && j==k) // energy of a specific actuator
                    m_qpSystem->Q[k][j] += ac->getEpsilon()*weight*m_qpSystem->W[acIds[k]][acIds[j]];
                else
                    m_qpSystem->Q[k][j] += m_epsilon*weight*m_qpSystem->W[acIds[k]][acIds[j]];
            }
        }
        else // contacts
        {
            for(unsigned int j=0; j<dim; j++)
                m_qpSystem->Q[k][j] += m_epsilon*weight*m_qpSystem->W[acIds[k]][acIds[j]];
        }
    }

}



/// Solve system

void QPInverseProblemImpl::solve(double& objective, int& iterations)
{
    int nbContactRows   = m_qpCLists->contactRowIds.size();
    int nbActuatorRows  = m_qpCLists->actuatorRowIds.size();
    int nbEffectorRows  = m_qpCLists->effectorRowIds.size();
    int nbEqualityRows  = m_qpCLists->equalityRowIds.size();

    m_qpSystem->dim = nbContactRows + nbActuatorRows + nbEqualityRows;
    m_qpSystem->W = getW();
    m_qpSystem->dFree = getDfree();
    m_qpCParams->mu = m_mu;
    m_qpCParams->allowSliding = m_allowSliding;

    vector<double> result; // size of dim
    vector<double> dual; // size of nb constraints

    objective  = 0;
    iterations = 0; // from contact pivot algorithm

    if(nbContactRows>0)
    {
        if(m_mu>0.)
        {
            // TODO: implement choice of user and projection of the tangential force to determine dir1 and dir2
            m_qpCParams->slidingDirId1 = 1;
            m_qpCParams->slidingDirId2 = 2;
            m_qpCParams->contactNbLines = 3;
            m_qpCParams->nbContactPoints = m_qpCLists->contactRowIds.size()/m_qpCParams->contactNbLines;
        }
        else
        {
            m_qpCParams->contactNbLines = 1;
            m_qpCParams->nbContactPoints = m_qpCLists->contactRowIds.size();
        }
        solveWithContact(result, objective, iterations);
    }
    else if(nbActuatorRows + nbEqualityRows > 0 )
    {
        buildQPMatrices();
        m_constraintHandler->buildInequalityConstraintMatrices(result, m_qpSystem, m_qpCLists);
        m_constraintHandler->buildEqualityConstraintMatrices(result, m_qpSystem, m_qpCLists);
        m_constraintHandler->getConstraintOnLambda(result, m_qpSystem, m_qpCLists);

        AdvancedTimer::stepBegin("QP resolution");
        solveInverseProblem(objective, result, dual);
        AdvancedTimer::stepEnd("QP resolution");

        updateLambda(result);
        if(!isFeasible(result))
            msg_warning("QPInverseProblemImpl") << "Solution not feasible.";
    }

    for(int i=0; i<nbEffectorRows; i++)
        objective += m_qpSystem->dFree[m_qpCLists->effectorRowIds[i]]*m_qpSystem->dFree[m_qpCLists->effectorRowIds[i]];

    storeResults(m_qpSystem->lambda);
}


void QPInverseProblemImpl::solveWithContact(vector<double>& result, double& objective, int& iteration)
{
    int nbActuatorRows  = m_qpCLists->actuatorRowIds.size();

    m_constraintHandler->initContactHandlerList();
    m_constraintHandler->initContactHandlers();

    AdvancedTimer::stepBegin("LCP resolution");
    solveContacts(result);
    AdvancedTimer::stepEnd("LCP resolution");
    updateLambda(result);
    m_qpSystem->previousResult = result;

    if(nbActuatorRows>0)
    {
        m_constraintHandler->initContactHandlers(result, m_qpCLists);

        iteration = 0;
        objective = 0.;

        buildQPMatrices();

        // TODO check for null rows and remove them from the OASES system

        AdvancedTimer::stepBegin("QPs resolution");
        bool stopFlag = false;
        while(stopFlag == false && iteration<=m_maxNbPivot)
        {
            m_iteration = iteration;

            m_qpCParams->constraintsId.clear();
            m_constraintHandler->buildInequalityConstraintMatrices(result, m_qpSystem, m_qpCLists);
            m_constraintHandler->buildEqualityConstraintMatrices(result, m_qpSystem, m_qpCLists);
            m_constraintHandler->getConstraintOnLambda(result, m_qpSystem, m_qpCLists);

            vector<double> dual;
            solveInverseProblem(objective, result, dual);

            stopFlag = checkAndUpdatePivot(result, dual);
            iteration++;

            if(stopFlag == true || iteration == m_maxNbPivot)
                updateLambda(result);
        }
        AdvancedTimer::stepEnd("QPs resolution");

        m_qpCParams->contactStates.clear();
        m_sequence.clear();
        m_currentSequence.clear();
        m_previousSequence.clear();
    }
}


void QPInverseProblemImpl::solveContacts(vector<double>& res)
{
    unsigned int nbActuatorRows   = m_qpCLists->actuatorRowIds.size();
    unsigned int nbContactRows    = m_qpCLists->contactRowIds.size();

    FullVector<double> q;
    q.resize(nbContactRows);
    LPtrFullMatrix<double> M;
    M.resize(nbContactRows,nbContactRows);

    if (m_step == 0) {
        init();
        m_step++;
    }

    for(unsigned int i=0; i<nbContactRows; i++)
    {
        q[i]=m_qpSystem->dFree[m_qpCLists->contactRowIds[i]];
        if(m_qpSystem->lambda.size()!=0)
            for(unsigned int j=0; j<nbActuatorRows; j++)
                q[i]+=m_qpSystem->W[m_qpCLists->contactRowIds[i]][m_qpCLists->actuatorRowIds[j]]*m_qpSystem->lambda[j];

        for(unsigned int j=0; j<nbContactRows; j++)
            M[i][j]=m_qpSystem->W[m_qpCLists->contactRowIds[i]][m_qpCLists->contactRowIds[j]];
    }

    FullVector<double> x;
    res.clear();
    res.resize(m_qpSystem->dim, 0.);

    if(m_mu>0.)
    {
        x.resize(nbContactRows);

        // Warm start
        if(m_qpSystem->lambda.size()>=nbActuatorRows+nbContactRows)
        {
            for(unsigned int i=0; i<nbContactRows; i++)
                x[i] = m_qpSystem->lambda[nbActuatorRows+i];
        }

        NLCPSolver* nlcpSolver = new NLCPSolver();
        nlcpSolver->setAllowSliding(m_allowSliding);
        nlcpSolver->solve(nbContactRows, q.ptr(), M.lptr(), x.ptr(), m_mu, m_tolerance, m_maxIteration, true);
        delete nlcpSolver;

        for (unsigned int i=0; i<nbContactRows; i++)
            res[i+nbActuatorRows] = x[i];
    }
    else
    {
        x.clear();
        x.resize(nbContactRows);
        // will use overrided solve depending on its solver implementation
        m_lcpQpSolver->solve(nbContactRows, q.ptr(), M.lptr(), x.ptr());

        for (unsigned int i=0; i<nbContactRows; i++)
            res[i+nbActuatorRows]=x[i];
    }

    if(m_qpSystem->lambda.size()>=nbActuatorRows)
        for (unsigned int i=0; i<nbActuatorRows; i++)
            res[i] = m_qpSystem->lambda[i];
}

/// Utils for pivot algorithm

bool QPInverseProblemImpl::checkAndUpdatePivot(const vector<double>&result,
                                               const vector<double>&dual)
{
    if(dual.size()==0)
        return true;

    int nbContactRow   = m_qpCLists->contactRowIds.size();
    int nbActuatorRow  = m_qpCLists->actuatorRowIds.size();

    vector<bool> isCandidate;
    isCandidate.resize(nbContactRow, false);

    int nbPivot = 0;

    // Identify those inequality constraints that reach their boundary
    if(m_mu>0.0)
    {
        for(int i=0; i<nbContactRow; i+=m_qpCParams->contactNbLines) // For each contact point
        {
            vector<double> delta  = {getDelta(result, m_qpCLists->contactRowIds[i]),
                                     getDelta(result, m_qpCLists->contactRowIds[i+m_qpCParams->slidingDirId1]),
                                     getDelta(result, m_qpCLists->contactRowIds[i+m_qpCParams->slidingDirId2])};
            vector<double> lambda = {result[nbActuatorRow+i],
                                     result[nbActuatorRow+i+m_qpCParams->slidingDirId1],
                                     result[nbActuatorRow+i+m_qpCParams->slidingDirId2]};

            for(unsigned int k=0;k<m_qpCParams->contactNbLines; k++)
                isCandidate[i+k]   = false;

            int candidateId = 0;
            if(m_qpCParams->contactStates[i/m_qpCParams->contactNbLines]->hasReachedBoundary(lambda, delta, m_mu, candidateId))
            {
                if(m_allowSliding)
                {
                    if(candidateId>2) // In case sliding <-> stick we need to look at the dual variable of both lambda_t and lambda_o
                    {
                        isCandidate[i+m_qpCParams->slidingDirId1] = true;
                        isCandidate[i+m_qpCParams->slidingDirId2] = true;
                    }
                    else
                        isCandidate[i+candidateId] = true;
                }
                else
                    isCandidate[i] = true;
            }

            if(isCandidate[i] || isCandidate[i+1] || isCandidate[i+2])
                nbPivot++;
        }
    }
    else
    {
        for(int i=0; i<nbContactRow; i++)
        {
            double delta = getDelta(result, m_qpCLists->contactRowIds[i]);
            double lambda = result[nbActuatorRow+i];

            isCandidate[i] = m_qpCParams->contactStates[i]->hasReachedBoundary(lambda, delta);
            if(isCandidate[i])
                nbPivot++;
        }
    }

    // Pivot constraints
    if(nbPivot>0)
    {
        // Select best candidate for pivot
        double max = 0.;
        bool doPivot = false;
        int bestCandidate = 0;

        for(unsigned int i=0; i<dual.size(); i++) // Size of [A; Aeq]
        {
            if(m_qpCParams->constraintsId[i]>=nbActuatorRow) // Row in [A; Aeq] corresponding to contact constraint
            {
                int contactConstraintId = m_qpCParams->constraintsId[i]-nbActuatorRow;
                if(isCandidate[contactConstraintId] && (dual[i]>max || doPivot == false)) // Find the most blocking lambda_c by looking at its dual
                {
                    if(!isCycling(contactConstraintId))
                    {
                        max = dual[i];
                        bestCandidate = contactConstraintId;
                        doPivot = true;
                    }
                    /*else
                        msg_warning("checkAndUpdatePivot") << "is cycling at iteration " << m_iteration;*/
                }
            }
        }

        // Update
        // Pivot best candidate
        if(doPivot)
        {
            if(m_mu>0)
            {
                int candidate = (bestCandidate - (bestCandidate%m_qpCParams->contactNbLines))/m_qpCParams->contactNbLines; // Contact id

                vector<double> delta  = {getDelta(result, m_qpCLists->contactRowIds[candidate*m_qpCParams->contactNbLines]),
                                         getDelta(result, m_qpCLists->contactRowIds[candidate*m_qpCParams->contactNbLines+1]),
                                         getDelta(result, m_qpCLists->contactRowIds[candidate*m_qpCParams->contactNbLines+2])};
                vector<double> lambda = {result[nbActuatorRow+candidate*m_qpCParams->contactNbLines],
                                         result[nbActuatorRow+candidate*m_qpCParams->contactNbLines+1],
                                         result[nbActuatorRow+candidate*m_qpCParams->contactNbLines+2]};

                m_qpCParams->contactStates[candidate] = m_qpCParams->contactStates[candidate]->getNewContactHandler(m_qpCParams->allowedContactStates, lambda, delta, m_mu);
            }
            else
            {
                double delta = getDelta(result, m_qpCLists->contactRowIds[bestCandidate]);
                double lambda = result[nbActuatorRow+bestCandidate];
                m_qpCParams->contactStates[bestCandidate] = m_qpCParams->contactStates[bestCandidate]->getNewContactHandler(m_qpCParams->allowedContactStates, lambda, delta);
            }

            if(isIn(m_currentSequence,bestCandidate))
            {
                m_previousSequence = m_sequence;
                m_sequence = m_currentSequence;
                m_currentSequence.clear();
            }
            m_currentSequence.push_back(bestCandidate);

            return false;
        }
    }

    return true;
}


bool QPInverseProblemImpl::isCycling(const int pivot)
{
    // Sequence starts if pivot already in currentSequence
    if(isIn(m_currentSequence, pivot))
        if(m_currentSequence.size()!=0 && m_currentSequence[m_currentSequence.size()-1]==pivot)
            return true;

    if(m_sequence.size()!=0 && m_previousSequence.size()!=0 && m_previousSequence[0]==pivot && m_sequence[0]==pivot)
        return true;

    vector<int> currentSequence = m_currentSequence;
    currentSequence.push_back(pivot);
    if(currentSequence==m_sequence || currentSequence==m_previousSequence)
        return true;

    return false;
}


bool QPInverseProblemImpl::isIn(const vector<int> list,
                                const int elem)
{
    for(unsigned int i=0; i<list.size(); i++)
        if (list[i]==elem)
            return true;

    return false;
}


double QPInverseProblemImpl::getDelta(const vector<double>& result,
                                      const int& index)
{
    unsigned int nbActuators  = m_qpCLists->actuatorRowIds.size();
    unsigned int nbEquality = m_qpCLists->equalityRowIds.size();

    vector<unsigned int> acIds;
    acIds.resize(m_qpSystem->dim);

    for (unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        if (i<nbActuators)                acIds[i] = m_qpCLists->actuatorRowIds[i];  // actuators first
        else if(i<nbActuators+nbEquality) acIds[i] = m_qpCLists->equalityRowIds[i-nbActuators]; // equality second
        else                              acIds[i] = m_qpCLists->contactRowIds[i-nbActuators-nbEquality]; // contacts last
    }

    double delta = m_qpSystem->dFree[index];
    for(unsigned int i=0; i<m_qpSystem->dim; i++)
        delta += m_qpSystem->W[index][acIds[i]]*result[i];

    return delta;
}



/// Others

void QPInverseProblemImpl::updateLambda(const vector<double>& lambda)
{
    if(lambda.size() != m_qpSystem->dim)
        msg_error("[QPInverseProblemImpl]") <<"Problem with lambda size";

    m_qpSystem->lambda.clear();
    m_qpSystem->lambda.resize(m_qpSystem->dim);
    for(unsigned int i=0; i<m_qpSystem->dim; i++)
        m_qpSystem->lambda[i] = lambda[i];
}


bool QPInverseProblemImpl::isFeasible(const vector<double>& x)
{
    for (unsigned int i=0; i<m_qpSystem->A.size(); i++)
    {
        double Ax_i = 0;
        for(unsigned int j=0; j<m_qpSystem->dim; j++)
            Ax_i += m_qpSystem->A[i][j]*x[j];

        if(Ax_i>m_qpSystem->bu[i] && rabs(Ax_i-m_qpSystem->bu[i])>1e-5)
            return false;

        if(m_qpSystem->hasBothSideInequalityConstraint)
            if(Ax_i<m_qpSystem->bl[i] && rabs(Ax_i-m_qpSystem->bl[i])>1e-5)
                return false;
    }

    for (unsigned int i=0; i<m_qpSystem->Aeq.size(); i++)
    {
        double Ax_i = 0;
        for(unsigned int j=0; j<m_qpSystem->dim; j++)
            Ax_i += m_qpSystem->Aeq[i][j]*x[j];

        if(rabs(Ax_i-m_qpSystem->beq[i])>1e-5)
            return false;
    }

    return true;
}


string QPInverseProblemImpl::getContactsState()
{
    string states;
    for(unsigned int i=0; i<m_qpCParams->contactStates.size(); i++)
        states += m_qpCParams->contactStates[i]->getStateString() + " ";
    return states;
}

} // namespace

#endif

