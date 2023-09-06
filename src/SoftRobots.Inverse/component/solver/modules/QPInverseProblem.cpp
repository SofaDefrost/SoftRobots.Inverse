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
#ifndef SOFA_COMPONENT_CONSTRAINTSET_QPInverseProblem_CPP
#define SOFA_COMPONENT_CONSTRAINTSET_QPInverseProblem_CPP

#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblem.h>
#include <SoftRobots.Inverse/component/solver/modules/QPInverseProblemImpl.h>
#include "libcholesky/QuadraticToLeastSquare.h"

#include <sofa/helper/LCPcalc.h>
#include <qpOASES.hpp>
#include <fstream>

#include <iomanip>

namespace softrobotsinverse::solver::module
{

using softrobots::behavior::SoftRobotsBaseConstraint;
using sofa::type::vector;
using std::endl;
using sofa::helper::rabs;
using std::string;
using std::ostringstream;
using std::ceil;

using sofa::linearalgebra::FullVector;
using sofa::linearalgebra::LPtrFullMatrix;


using qpOASES::QProblem;
using qpOASES::Options;
using qpOASES::real_t;
using qpOASES::int_t;

using sofa::helper::logging::Message;
using sofa::core::objectmodel::Base;


QPInverseProblem::QPInverseProblem()
    : m_epsilon(1e-3)
    , m_tolerance(1e-5)
    , m_maxIteration(200)
    , m_largestQNormVariation(0.0)
    , m_QNorm(0.0)
    , m_resolutionId(0)
{
    m_qpSystem = new QPSystem;
    m_qpCLists = new QPConstraintLists;

    m_qpSystem->hasBothSideInequalityConstraint = false;
}

QPInverseProblem::~QPInverseProblem()
{
    delete m_qpCLists;
    delete m_qpSystem;
}


void QPInverseProblem::clear(int nbC)
{
    ConstraintProblem::clear(nbC);
}


void QPInverseProblem::solveTimed(double tol, int maxIt, double timeout)
{
    SOFA_UNUSED(tol);
    SOFA_UNUSED(maxIt);
    SOFA_UNUSED(timeout);
}


void QPInverseProblem::clearProblem()
{
    m_qpCLists->actuators.clear();
    m_qpCLists->equality.clear();
    m_qpCLists->effectors.clear();
    m_qpCLists->sensors.clear();
    m_qpCLists->contacts.clear();

    m_qpCLists->actuatorRowIds.clear();
    m_qpCLists->equalityRowIds.clear();
    m_qpCLists->effectorRowIds.clear();
    m_qpCLists->sensorRowIds.clear();
    m_qpCLists->contactRowIds.clear();

    m_qpSystem->A.clear();
    m_qpSystem->Aeq.clear();
    m_qpSystem->bl.clear();
    m_qpSystem->bu.clear();
    m_qpSystem->beq.clear();

    m_qpSystem->Q.clear();
    m_qpSystem->c.clear();
}


void QPInverseProblem::storeResults(const vector<double> &x)
{
    double *lambda = getF();
    double **w = getW();
    double *dfree = getDfree();

    unsigned int nbActuatorRows = m_qpCLists->actuatorRowIds.size();
    unsigned int nbEffectorRows = m_qpCLists->effectorRowIds.size();
    unsigned int nbSensorRows   = m_qpCLists->sensorRowIds.size();
    unsigned int nbContactRows  = m_qpCLists->contactRowIds.size();
    unsigned int nbEqualityRows     = m_qpCLists->equalityRowIds.size();
    unsigned int nbRows = nbEffectorRows + nbActuatorRows + nbContactRows + nbSensorRows + nbEqualityRows;

    m_qpSystem->delta.resize(nbRows);

    for(unsigned int i=0; i<nbEffectorRows; i++)
        lambda[m_qpCLists->effectorRowIds[i]] = 0;

    for(unsigned int i=0; i<nbActuatorRows; i++)
        lambda[m_qpCLists->actuatorRowIds[i]] = x[i];

    for(unsigned int i=0; i<nbEqualityRows; i++)
        lambda[m_qpCLists->equalityRowIds[i]] = x[i+nbActuatorRows];

    for(unsigned int i=0; i<nbSensorRows; i++)
        lambda[m_qpCLists->sensorRowIds[i]] = 0;

    for(unsigned int i=0; i<nbContactRows; i++)
        lambda[m_qpCLists->contactRowIds[i]] = x[i+nbActuatorRows+nbEqualityRows];


    for(unsigned int i=0; i<nbRows; i++)
    {
        m_qpSystem->delta[i] = dfree[i];
        for(unsigned int j=0; j<nbRows; j++)
            m_qpSystem->delta[i] += lambda[j]*w[i][j];
    }

    sendResults();
}


void QPInverseProblem::sendResults()
{
    double *lambda = getF();

    int line = 0;
    for(unsigned int i=0; i<m_qpCLists->actuators.size(); i++) // For each actuator component
    {
        vector<double> localLambda, localDelta;
        int nbLines = m_qpCLists->actuators[i]->getNbLines();

        localLambda.resize(nbLines);
        localDelta.resize(nbLines);

        for(int j=0; j<nbLines; j++)
        {
            localLambda[j] = lambda[m_qpCLists->actuatorRowIds[line]];
            localDelta[j]  = m_qpSystem->delta[m_qpCLists->actuatorRowIds[line]];
            line++;
        }

        m_qpCLists->actuators[i]->storeResults(localLambda,localDelta);

        // Allows actuators to change value of force if needed (from the call to storeResults)
        for(int j=0; j<nbLines; j++)
            lambda[m_qpCLists->actuatorRowIds[line-nbLines+j]] = localLambda[j];
    }

    line = 0;
    for(unsigned int i=0; i<m_qpCLists->equality.size(); i++) // For each equality component
    {
        vector<double> localLambda, localDelta;
        int nbLines = m_qpCLists->equality[i]->getNbLines();

        localLambda.resize(nbLines);
        localDelta.resize(nbLines);

        for(int j=0; j<nbLines; j++)
        {
            localLambda[j] = lambda[m_qpCLists->equalityRowIds[line]];
            localDelta[j]  = m_qpSystem->delta[m_qpCLists->equalityRowIds[line]];
            line++;
        }

        m_qpCLists->equality[i]->storeResults(localLambda,localDelta);

        // Allows equality to change value of force if needed (from the call to storeResults)
        for(int j=0; j<nbLines; j++)
            lambda[m_qpCLists->equalityRowIds[line-nbLines+j]] = localLambda[j];
    }


    line = 0;
    for(unsigned int i=0; i<m_qpCLists->effectors.size(); i++) // For each effector component
    {
        vector<double> localDelta;
        int nbLines = m_qpCLists->effectors[i]->getNbLines();

        localDelta.resize(nbLines);

        for(int j=0; j<nbLines; j++)
            localDelta[j]=m_qpSystem->delta[m_qpCLists->effectorRowIds[line++]];

        m_qpCLists->effectors[i]->storeResults(localDelta);
    }


    line = 0;
    for(unsigned int i=0; i<m_qpCLists->sensors.size(); i++) // For each sensor component
    {
        vector<double> localDelta;
        int nbLines = m_qpCLists->sensors[i]->getNbLines();

        localDelta.resize(nbLines);

        for(int j=0; j<nbLines; j++)
            localDelta[j]=m_qpSystem->delta[m_qpCLists->sensorRowIds[line++]];

        m_qpCLists->sensors[i]->storeResults(localDelta);
    }
}


void QPInverseProblem::displayResult()
{
    int dim = getDimension();
    double* lambda = getF();

    ostringstream stream;

    stream << std::setiosflags(std::ios::right);
    stream << " delta = [";
    for(unsigned int i=0; i<m_qpSystem->delta.size(); i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->delta[i];

        if (i!=m_qpSystem->delta.size()-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " lambda = [";
    for(int i=0; i<dim; i++)
    {
        stream << "\t" << std::setw(12) << lambda[i];

        if (i!=dim-1)
            stream << ", ";
    }
    stream << "      ];\n\n";
    stream << std::resetiosflags(std::ios::right);

    msg_info("QPInverseProblem") << stream.str();
}


void QPInverseProblem::displayQPSystem()
{
    ostringstream stream;

    unsigned int dim = m_qpCLists->actuatorRowIds.size() + m_qpCLists->effectorRowIds.size() + m_qpCLists->contactRowIds.size() + m_qpCLists->equalityRowIds.size();
    stream << std::setiosflags(std::ios::right);
    stream << "System: \n";

    stream << " W      = [";
    for(unsigned int i=0; i<dim; i++)
    {
        stream << "[";
        if(i!=0)
            stream << "\t";
        for(unsigned int j=0; j<dim; j++)
        {
            stream << "\t"<< std::setw(12) <<m_qpSystem->W[i][j];

            if (j!=dim-1)
                stream << ", ";
            else stream << "],";
        }
        if(i!=dim-1)
            stream << "\n";
    }
    stream << "           ];\n\n";

    dim = m_qpSystem->Q.size();
    stream << std::setiosflags(std::ios::right);
    stream << " Q      = [";
    for(unsigned int i=0; i<dim; i++)
    {
        if(i!=0)
            stream << "\t";
        for(unsigned int j=0; j<dim; j++)
        {
            stream << "\t"<< std::setw(12) <<m_qpSystem->Q[i][j];

            if (j!=dim-1)
                stream << ", ";
        }
        if(i!=dim-1)
            stream << "\n";
    }
    stream << "           ];\n\n";

    stream << " A      = [";
    for(unsigned int i=0; i<m_qpSystem->A.size(); i++)
    {
        if(i!=0)
            stream << "\t";
        for(unsigned int j=0; j<dim; j++)
        {
            stream << "\t"<< std::setw(12) <<m_qpSystem->A[i][j];

            if (j!=dim-1)
                stream << ", ";
        }
        if(i!=dim-1)
            stream << "\n";
    }
    stream << "           ];\n\n";

    stream << " Aeq      = [";
    for(unsigned int i=0; i<m_qpSystem->Aeq.size(); i++)
    {
        if(i!=0)
            stream << "\t";
        for(unsigned int j=0; j<dim; j++)
        {
            stream << "\t"<< std::setw(12) <<m_qpSystem->Aeq[i][j];

            if (j!=dim-1)
                stream << ", ";
        }
        if(i!=dim-1)
            stream << "\n";
    }
    stream << "           ];\n\n";


    stream << " c      = [";
    for(unsigned int i=0; i<dim; i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->c[i];

        if (i!=dim-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " bu      = [";
    for(unsigned int i=0; i<m_qpSystem->bu.size(); i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->bu[i];

        if (i!=m_qpSystem->bu.size()-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " bl      = [";
    for(unsigned int i=0; i<m_qpSystem->bl.size(); i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->bl[i];

        if (i!=m_qpSystem->bl.size()-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " beq      = [";
    for(unsigned int i=0; i<m_qpSystem->beq.size(); i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->beq[i];

        if (i!=m_qpSystem->beq.size()-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " l      = [";
    for(unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->l[i];

        if (i!=m_qpSystem->dim-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " u      = [";
    for(unsigned int i=0; i<m_qpSystem->dim; i++)
    {
        stream << "\t" << std::setw(12) << m_qpSystem->u[i];

        if (i!=m_qpSystem->dim-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << " dfree  = [";
    dim = getDimension();
    double* dfree = getDfree();
    for(unsigned int i=0; i<dim; i++)
    {
        stream << "\t" << std::setw(12) << dfree[i];

        if (i!=dim-1)
            stream << ", ";
    }
    stream << "      ];\n\n";

    stream << std::resetiosflags(std::ios::right);

    msg_info("QPInverseProblem") << stream.str();
}


void QPInverseProblem::saveMatricesToFile(const bool append)
{
    std::ofstream file;
    char filename[100];
    sprintf(filename, "QPInverseProblemSolverMatrices.txt");
    if(append)
    {
        if(m_time>0.)
            file.open(filename, std::ios_base::app);
        else
            file.open(filename);
        file << "####################" << endl;
        file << "### Time = " << m_time << endl;
        file << "####################" << endl << endl;
    }
    else
        file.open(filename);

    unsigned int dim = m_qpSystem->Q.size();
    unsigned int dimSys = m_qpSystem->dim;

    //Save Q matrix in matlab/scilab format
    file << " Q      = [";
    for(unsigned int i=0; i<dim; i++)
    {
        file << "[   ";
        for(unsigned int j=0; j<dim-1; j++)
            file << "\t"<<m_qpSystem->Q[i][j] << " , " ;
        file << "\t "<<m_qpSystem->Q[i][dim-1] <<"  ], " << endl;
    }
    file << " ];" << endl << endl;

    //Save c vector in matlab/scilab format
    file << " c      = [";
    for(unsigned int i=0; i<dim-1; i++)
        file << "\t" << m_qpSystem->c[i]<< " , " ;
    file << "\t "<< m_qpSystem->c[dim-1]<< "      ];" << endl << endl;

    //Save A matrix in matlab/scilab format
    file << " A      = [";
    for(unsigned int i=0; i<m_qpSystem->A.size(); i++)
    {
        file << "[   ";
        for(unsigned int j=0; j<dim-1; j++)
            file << "\t"<<m_qpSystem->A[i][j]<< " , " ;
        file << "\t "<<m_qpSystem->A[i][dim-1] <<"  ], " << endl;
    }
    file << "           ];" << endl << endl;

    //Save Aeq matrix in matlab/scilab format
    file << " Aeq      = [";
    unsigned int szAeq = m_qpSystem->Aeq.size();
    for(unsigned int i=0; i<szAeq; i++)
    {
        file << "[   ";
        for(unsigned int j=0; j<dim-1; j++)
            file << "\t"<<m_qpSystem->Aeq[i][j]<< " , " ;
        file << "\t "<<m_qpSystem->Aeq[i][dim-1] <<"  ], " << endl;

    }
    file << "           ];" << endl << endl;

    //Save bl vector in matlab/scilab format
    file << " bl      = [";
    unsigned int blSz = m_qpSystem->bl.size();
    for(unsigned int i=0; i < blSz; i++)
    {
        if (i>(blSz -1)) file << "\t" << m_qpSystem->bl[i]<< " , " ;
        else file << "\t "<< m_qpSystem->bl[i];
    }
    file << "           ];" << endl << endl;

    //Save bu vector in matlab/scilab format
    file << " bu      = [";
    unsigned int buSz = m_qpSystem->bu.size();
    for(unsigned int i=0; i<buSz; i++)
    {
        if(i<buSz-1) file << "\t" << m_qpSystem->bu[i]<< " , " ;
        else file << "\t "<<m_qpSystem->bu[i];
    }
    file << "           ];" << endl << endl;

    //Save beq vector in matlab/scilab format
    file << " beq      = [";
    unsigned int beqSz = m_qpSystem->beq.size();
    for(unsigned int i=0; i<beqSz; i++)
    {
        if(i<beqSz-1) file << "\t" << m_qpSystem->beq[i]<< " , " ;
        else file << "\t "<< m_qpSystem->beq[i];
    }
    file << "           ];" << endl << endl;

    //Save l vector in matlab/scilab format
    file << " l      = [";
    for(unsigned int i=0; i<dimSys; i++)
    {
        if (i < dimSys-1) file << "\t" << m_qpSystem->l[i]<< " , " ;
        else file << "\t "<< m_qpSystem->l[i];
    }
    file << "           ];" << endl << endl;

    //Save u vector in matlab/scilab format
    file << " u      = [";
    for(unsigned int i=0; i<dimSys; i++)
    {
        if(i<dimSys-1) file << "\t" << m_qpSystem->u[i]<< " , " ;
        else file << "\t "<< m_qpSystem->u[i];
    }
    file << "           ];" << endl << endl;

    //Save lambda vector in matlab/scilab format
    file << " lambda      = [";
    for(unsigned int i=0; i<dim; i++)
    {
        if(i<dim-1) file << "\t" << m_qpSystem->lambda[i]<< " , " ;
        else file << "\t "<< m_qpSystem->lambda[i];
    }
    file << "           ];" << endl << endl;

    file.close();
}


void QPInverseProblem::displayQNormVariation(int& countdownFilterStartPerturb)
{
    int dim = m_qpSystem->Q.size();

    double prevNorm = m_QNorm;
    double norm = 0.0;

    //infinity-norm of a square matrix
    for (int i=0; i<dim; i++)
    {
        double columnAbsSum = 0.0;
        for (int j=0; j<dim; j++)
        {
            columnAbsSum += sofa::helper::rabs(m_qpSystem->Q[j][i]);
        }
        if (columnAbsSum > norm)
        {
            norm = columnAbsSum;
        }
    }
    m_QNorm = norm;

    ostringstream stream;
    stream << " Q infinity norm : " << norm << "\n";
    stream << " Previous infinity norm : " << prevNorm << "\n";

    if(countdownFilterStartPerturb==0)
    {
        double varNorm = rabs((ceil((((norm*1000)-(prevNorm*1000))*10000)/(prevNorm*1000)))/100);
        if (varNorm > m_largestQNormVariation)
        {
            m_largestQNormVariation = varNorm;
        }

        stream << " Relative variation of infinity norm through one step: " << varNorm << " %\n";
        stream << " Largest relative variation of infinity norm through one step: " << m_largestQNormVariation << " %\n\n";
    }
    else
    {
        countdownFilterStartPerturb = countdownFilterStartPerturb-1;
    }
}

} // namespace

#endif
