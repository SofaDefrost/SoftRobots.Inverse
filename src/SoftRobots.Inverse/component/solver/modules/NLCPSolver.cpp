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

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/helper/logging/Messaging.h>
#include <sofa/helper/rmath.h>
#include <sofa/type/Mat.h>

#include <SoftRobots.Inverse/component/solver/modules/NLCPSolver.h>

namespace softrobotsinverse::solver::module
{

using namespace std;
using namespace sofa::helper::system::thread;
using sofa::helper::AdvancedTimer;
using sofa::type::Mat;

int NLCPSolver::solve(int dim, double *dfree, double**W, double *result, double mu, double tol, int nbIterationMax,
                      bool useInitialF, bool verbose, double minW, double maxF, sofa::type::vector<double>* residuals,
                      sofa::type::vector<double>* violations)

{
    SOFA_UNUSED(minW);
    SOFA_UNUSED(maxF);

    if (dim % 3)
    {
        msg_warning("NLCPSolver") << "The parameter 'dim' should be dividable by three.";
        return 0;
    }
    const int nbContacts = dim/3;

    // iterators
    int it,cIt,i;

    // put the vector force to zero
    if (!useInitialF)
        memset(result, 0, dim*sizeof(double));

    // previous value of the force and the displacment
    double f_prev[3];
    double d_prev[3];

    // allocation of the inverted system 3x3
    NLCPSolverMatrix33 **W33;
    W33 = (NLCPSolverMatrix33 **) malloc (dim*sizeof(NLCPSolverMatrix33));
    for (cIt=0; cIt<nbContacts; cIt++)
        W33[cIt] = new NLCPSolverMatrix33();


    //////////////
    // Beginning of iterative computations
    //////////////
    double error = 0.;
    double dn, dt, ds, fn, ft, fs;

    for (it=0; it<nbIterationMax; it++)
    {
        error = 0.;
        for (cIt=0; cIt<nbContacts; cIt++)
        {
            // index of contact
            int cIndex = cIt;

            // put the previous value of the contact force in a buffer and put the current value to 0
            f_prev[0] = result[3*cIndex];
            f_prev[1] = result[3*cIndex+1];
            f_prev[2] = result[3*cIndex+2];
            sofa::helper::set3Dof(result,cIndex,0.0,0.0,0.0); // f[3*cIndex] = 0.0; f[3*cIndex+1] = 0.0; f[3*cIndex+2] = 0.0;

            // computation of actual d due to contribution of other contacts
            dn=dfree[3*cIndex];
            dt=dfree[3*cIndex+1];
            ds=dfree[3*cIndex+2];

            for (i=0; i<dim; i++)
            {
                dn += W[3*cIndex  ][i]*result[i];
                dt += W[3*cIndex+1][i]*result[i];
                ds += W[3*cIndex+2][i]*result[i];
            }

            d_prev[0] = dn + W[3*cIndex  ][3*cIndex  ]*f_prev[0]+W[3*cIndex  ][3*cIndex+1]*f_prev[1]+W[3*cIndex  ][3*cIndex+2]*f_prev[2];
            d_prev[1] = dt + W[3*cIndex+1][3*cIndex  ]*f_prev[0]+W[3*cIndex+1][3*cIndex+1]*f_prev[1]+W[3*cIndex+1][3*cIndex+2]*f_prev[2];
            d_prev[2] = ds + W[3*cIndex+2][3*cIndex  ]*f_prev[0]+W[3*cIndex+2][3*cIndex+1]*f_prev[1]+W[3*cIndex+2][3*cIndex+2]*f_prev[2];

            if(W33[cIndex]->m_stored==false)
            {
                W33[cIndex]->storeW(W[3*cIndex][3*cIndex],W[3*cIndex][3*cIndex+1],W[3*cIndex][3*cIndex+2],
                        W[3*cIndex+1][3*cIndex+1], W[3*cIndex+1][3*cIndex+2],W[3*cIndex+2][3*cIndex+2]);
            }

            fn=f_prev[0];
            ft=f_prev[1];
            fs=f_prev[2];
            W33[cIndex]->GSState(mu,dn,dt,ds,fn,ft,fs,m_allowSliding);

            error += sofa::helper::absError(dn,dt,ds,d_prev[0],d_prev[1],d_prev[2]);

            sofa::helper::set3Dof(result,cIndex,fn,ft,fs);
        }

        if (residuals) residuals->push_back(error);

        if (violations)
        {
            double sum_d = 0;
            for (int c=0;  c<nbContacts ; c++)
            {
                dn = dfree[3*c];

                for (int i=0; i<dim; i++)
                    dn += W[3*c][i]*result[i];

                if (dn < 0)
                    sum_d += -dn;
            }
            violations->push_back(sum_d);
        }

        if (error < tol*(nbContacts+1))
        {
            for (int i = 0; i < nbContacts; i++)
                delete W33[i];
            free(W33);
            if (verbose)
                msg_info("NLCPSolver") << "Convergence after" << it <<" iteration(s) with tolerance : "<< tol <<" and error : "<< error <<" with dim : "<<dim;
            AdvancedTimer::valSet("GS iterations", it+1);
            return 1;
        }
    }
    AdvancedTimer::valSet("GS iterations", it);

    for (int i = 0; i < nbContacts; i++)
        delete W33[i];
    free(W33);

    if (verbose)
        msg_warning("NLCPSolver") <<"No convergence in  nlcp_gaussseidel function : error ="<<error <<" after"<< it<<" iterations";

    return 0;
}


void NLCPSolverMatrix33::GSState(double &mu, double &dn, double &dt, double &ds, double &fn, double &ft, double &fs, bool allowSliding)
{
    double d[3];

    // evaluation of the current normal position
    d[0] = m_w[0]*fn + m_w[1]*ft + m_w[2]*fs + dn;
    // evaluation of the new contact force
    fn -= d[0]/m_w[0];

    if (fn < 0)
    {
        fn=0;
        ft=0;
        fs=0;
        return;
    }

    // evaluation of the current tangent positions
    d[1] = m_w[1]*fn + m_w[3]*ft + m_w[4]*fs + dt;
    d[2] = m_w[2]*fn + m_w[4]*ft + m_w[5]*fs + ds;

    // evaluation of the new fricton forces
    ft -= 2*d[1]/(m_w[3]+m_w[5]);
    fs -= 2*d[2]/(m_w[3]+m_w[5]);

    if(allowSliding)
    {
        double normFt;
        normFt=rabs(ft)+rabs(fs);
        if (normFt > mu*fn)
        {
            ft *= mu*fn/normFt;
            fs *= mu*fn/normFt;
        }
    }

    dn += m_w[0]*fn + m_w[1]*ft + m_w[2]*fs;
    dt += m_w[1]*fn + m_w[3]*ft + m_w[4]*fs;
    ds += m_w[2]*fn + m_w[4]*ft + m_w[5]*fs;

}


void NLCPSolverMatrix33::storeW(double &w11, double &w12, double &w13, double &w22, double &w23, double &w33)
{
    m_w[0]=w11; m_w[1]=w12; m_w[2] = w13; m_w[3]=w22; m_w[4]=w23; m_w[5]=w33;
    m_stored=true;
}


} // namespace
