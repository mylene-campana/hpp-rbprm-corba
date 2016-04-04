// Copyright (c) 2012 CNRS
// Author: Florent Lamiraux
//
// This file is part of hpp-manipulation-corba.
// hpp-manipulation-corba is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-corba is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-corba.  If not, see
// <http://www.gnu.org/licenses/>.

#include <hpp/corbaserver/server.hh>
#include <hpp/corbaserver/rbprm/server.hh>
#include <hpp/core/problem-solver.hh>
#include <hpp/rbprm/planner/steering-dynamic.hh>
#include <hpp/rbprm/planner/steering-method-parabola.hh>
#include <hpp/rbprm/planner/dynamic-planner.hh>
#include <hpp/rbprm/planner/prm-planner.hh>

typedef hpp::rbprm::Server RbprmServer;
typedef hpp::corbaServer::Server CorbaServer;

int main (int argc, char* argv [])
{
    hpp::core::ProblemSolverPtr_t problemSolver (hpp::core::ProblemSolver::create());
    problemSolver->addSteeringMethodType("SteeringDynamic", boost::bind(                                                   static_cast<hpp::rbprm::SteeringDynamicPtr_t (*)(const hpp::core::ProblemPtr_t&)>                                                     (&hpp::rbprm::SteeringDynamic::create), _1 ));
    problemSolver->addSteeringMethodType("SteeringParabola", boost::bind(                                                   static_cast<hpp::rbprm::SteeringMethodParabolaPtr_t (*)(const hpp::core::ProblemPtr_t&)>                                                     (&hpp::rbprm::SteeringMethodParabola::create), _1 ));


    problemSolver->addPathPlannerType("RRTdynamic",hpp::rbprm::DynamicPlanner::createWithRoadmap);
    problemSolver->addPathPlannerType("PRMplanner",hpp::rbprm::PrmPlanner::createWithRoadmap);


    CorbaServer corbaServer (problemSolver, argc,
               const_cast<const char**> (argv), true);
    RbprmServer rbprmServer (argc, argv, true, "rbprmChild");
    rbprmServer.setProblemSolver (problemSolver);

    corbaServer.startCorbaServer ();
    rbprmServer.startCorbaServer ("hpp", "corbaserver",
                "rbprm");
    corbaServer.processRequest(true);
}
