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

namespace softrobotsinverse::python3::doc
{
static auto QPInverseProblemSolver = R"(
        An Interface to write your own solver.

        Typical usage example:

        import Sofa.Constraint
        import Sofa.SoftRobotsInverse


        class MyQPInverseProblemSolver(Sofa.SoftRobotsInverse.QPInverseProblemSolver):

            def __init__(self, *args, **kwargs):
                Sofa.SoftRobotsInverse.QPInverseProblemSolver.__init__(self, *args, **kwargs)

            def solveSystem(self):
                W = self.W()
                dfree = self.dfree()
                forces = self.lambda_force()  # pointer on lambda
                forces = my_resolution()
                return True
         )";
}
