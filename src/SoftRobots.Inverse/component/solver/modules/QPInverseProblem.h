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
#include "Eigen/Core"


namespace softrobotsinverse::solver::module
{

using softrobots::behavior::SoftRobotsBaseConstraint;
using sofa::core::behavior::BaseConstraint;
using sofa::type::vector;

class SOFA_SOFTROBOTS_INVERSE_API QPInverseProblem : public sofa::component::constraint::lagrangian::solver::ConstraintProblem
{

  public:

    struct QPSystem{

        vector< vector<double> > Q; /// QP problem matrix
        vector< double > c;  /// QP problem vector

        vector< vector<double> > A; /// Inequality constraint matrix
        vector< vector<double> > Aeq; /// Equality constraint matrix
        vector< double > bl; /// Inequality constraint vector A*x >= bl
        vector< double > bu; /// Inequality constraint vector A*x <= bu
        vector< double > beq; /// Equality constraint vector Aeq*x = beq

        vector< double > l; /// Inequality constraint vector x >= l
        vector< double > u; /// Inequality constraint vector x <= u

        bool  hasBothSideInequalityConstraint; // If not, only bu will be filled for all constraint

        vector< double > delta; /// d = W*lambda + dfree
        vector< double > lambda;
        vector<double>   previousResult;

        unsigned int dim{0}; // QP problem dimension

        double* dFree;
        double** W;
    };

    struct QPConstraintLists{

        vector<SoftRobotsBaseConstraint*> actuators;
        vector<SoftRobotsBaseConstraint*> effectors;
        vector<SoftRobotsBaseConstraint*> sensors;
        vector<BaseConstraint*>           contacts;
        vector<SoftRobotsBaseConstraint*> equality;

        vector< unsigned int > effectorRowIds;
        vector< unsigned int > actuatorRowIds;
        vector< unsigned int > sensorRowIds;
        vector< unsigned int > contactRowIds;
        vector< unsigned int > equalityRowIds;
    };


    QPInverseProblem();
    virtual ~QPInverseProblem();


    ////////////////////// Inherit from ConstraintSolverImpl ////////////////////////
    virtual void clear(int nbConstraints);
    virtual void solveTimed(double tol, int maxIt, double timeout);
    /////////////////////////////////////////////////////////////////////////////////

    void solve(double& objective, int& iterations);

    void setTime(const double& time) {m_time = time;}
    void setEpsilon(const double& epsilon) {m_epsilon = epsilon;}
    void setEnergyActuatorsOnly(const bool& actuatorsOnly) {m_actuatorsOnly = actuatorsOnly;}
    void setTolerance(const double& tolerance) {m_tolerance = tolerance;}
    void setMaxIterations(const int& maxIt) {m_maxIteration = maxIt;}
    void setFrictionCoeff(const double& mu) {m_mu = mu;}
    void allowSliding(const bool& allowSliding) {m_allowSliding = allowSliding;}

    void clearProblem();

    void displayResult();

    void displayQPSystem();

    void displayQNormVariation(int& countdownFilterStartPerturb);

    void saveMatricesToFile(const bool append=false);

    QPSystem* getQPSystem() {return m_qpSystem;}
    QPConstraintLists* getQPConstraintLists() {return m_qpCLists;}

    void sendResults();

protected:

    QPSystem* m_qpSystem;
    QPConstraintLists* m_qpCLists;

    double    m_epsilon;
    bool      m_actuatorsOnly;
    double    m_tolerance;
    int       m_maxIteration;
    int       m_maxNbPivot{100}; //TODO: set by user?
    double    m_mu{0.0};
    bool      m_allowSliding;

    double m_largestQNormVariation;
    double m_QNorm;

    int m_resolutionId;

    double   m_time{0.};

    void storeResults(const sofa::type::vector<double> &x);

public:
    Eigen::MatrixXd getQPMatriceQ(){
        Eigen::MatrixXd Q;
        Q.resize(m_qpSystem->dim,m_qpSystem->dim);

        for (unsigned int i=0; i<m_qpSystem->dim; i++)
            for (unsigned int j=0; j<m_qpSystem->dim; j++)
                Q(i,j) = m_qpSystem->Q[i][j];

        return Q;
    }

    /// QP problem vector
    Eigen::VectorXd getQPVectorC(){
        Eigen::VectorXd c; c.resize(m_qpSystem->dim);

        for (unsigned int i=0; i<m_qpSystem->dim; i++)
            c(i) = m_qpSystem->c[i];
        return c;
    }

    /// Inequality constraint matrix
    Eigen::MatrixXd getQPMatrixA(){
        unsigned int ASize = m_qpSystem->A.size();
        Eigen::MatrixXd A; A.resize(ASize,m_qpSystem->dim);

        for (unsigned int i=0; i<ASize; i++)
            for (unsigned int j=0; j<m_qpSystem->dim; j++)
                A(i,j) = m_qpSystem->A[i][j];
        return A;
    }

    /// Equality constraint matrix
    Eigen::MatrixXd getQPMatrixAeq(){
        unsigned int AeqSize = m_qpSystem->Aeq.size();
        Eigen::MatrixXd Aeq; Aeq.resize(AeqSize,m_qpSystem->dim);

        for (unsigned int i=0; i<AeqSize; i++)
            for (unsigned int j=0; j<m_qpSystem->dim; j++)
                Aeq(i,j) = m_qpSystem->Aeq[i][j];
        return Aeq;
    }

    /// Inequality constraint vector A*x >= bl
    Eigen::VectorXd getQPVectorbl(){
        Eigen::VectorXd bl; bl.resize(m_qpSystem->A.size());

        for(unsigned int i=0; i<m_qpSystem->A.size(); i++){
            if (m_qpSystem->hasBothSideInequalityConstraint) bl(i)=m_qpSystem->bl[i];
            else bl(i) = -1e99;
        }
        return bl;
    }

    /// Inequality constraint vector A*x <= bu
    Eigen::VectorXd getQPVectorbu(){
        Eigen::VectorXd bu; bu.resize(m_qpSystem->A.size());

        for(unsigned int i=0; i<m_qpSystem->A.size(); i++)
            bu(i)=m_qpSystem->bu[i];
        return bu;
    }

    /// Equality constraint vector Aeq*x = beq
    Eigen::VectorXd getQPVectorbeq(){
        Eigen::VectorXd beq; beq.resize(m_qpSystem->Aeq.size());

        for(unsigned int i=0; i<m_qpSystem->Aeq.size(); i++)
            beq(i)=m_qpSystem->beq[i];
        return beq;
    }

    /// Inequality constraint vector x >= l
    Eigen::VectorXd getQPVectorl(){
        Eigen::VectorXd l; l.resize(m_qpSystem->dim);

        for(unsigned int i=0; i<m_qpSystem->dim; i++)
            l(i)=m_qpSystem->l[i];
        return l;
    }

    /// Inequality constraint vector x <= u
    Eigen::VectorXd getQPvectoru(){
        Eigen::VectorXd u; u.resize(m_qpSystem->dim);

        for(unsigned int i=0; i<m_qpSystem->dim; i++)
            u(i)=m_qpSystem->u[i];
        return u;
    }

};

} // namespace
