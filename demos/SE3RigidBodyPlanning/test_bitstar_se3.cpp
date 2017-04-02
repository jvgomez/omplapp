/*********************************************************************
* Rice University Software Distribution License
*
* Copyright (c) 2010, Rice University
* All Rights Reserved.
*
* For a full description see the file named LICENSE.
*
*********************************************************************/

/* Author: Ioan Sucan */

#include <omplapp/apps/SE3RigidBodyPlanning.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <omplapp/config.h>


using namespace ompl;

int main()
{
	double time = 0.1;
	bool halton = true;
	unsigned iterations = 50;
	unsigned success = 0;

    ompl::msg::setLogLevel(ompl::msg::LOG_WARN);
	// Run  times the regular algorithm
    for (unsigned i = 0; i < iterations; ++i) {
		std::cout << "RUN " << i << "\t";
		// plan in SE3
		app::SE3RigidBodyPlanning setup;

		// load the robot and the environment
		std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_robot.dae";
		std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/3D/cubicles_env.dae";
		setup.setRobotMesh(robot_fname.c_str());
		setup.setEnvironmentMesh(env_fname.c_str());

		// define start state
		base::ScopedState<base::SE3StateSpace> start(setup.getSpaceInformation());
		start->setXYZ(-4.96, -40.62, 70.75);
		start->rotation().setAxisAngle(1, 0, 0, 0);
		//start->setY(-40.62);
		//start->setYaw(0.0);

		// define goal state
		base::ScopedState<base::SE3StateSpace> goal(start);
		goal->setXYZ(200., -40.62, 70.75);
		goal->rotation().setAxisAngle(1, 0, 0, 0);
		
		base::RealVectorBounds bounds(3);
        bounds.setHigh(0,319.62);
        bounds.setHigh(1,531.87);
        bounds.setHigh(2,101.);
        bounds.setLow(0,-508.88);
        bounds.setLow(1,-230.13);
        bounds.setLow(2,-123.75);
        setup.getStateSpace()->as<base::SE3StateSpace>()->setBounds(bounds);

		// set the start & goal states
		setup.setStartAndGoalStates(start, goal);

		// setting collision checking resolution to 1% of the space extent
		setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

		// make sure the planners run until the time limit, and get the best possible solution
		setup.getProblemDefinition()->setOptimizationObjective(
			std::make_shared<base::PathLengthOptimizationObjective>(setup.getSpaceInformation()));

		// run with RRT*
		setup.setPlanner(std::make_shared<geometric::BITstar>(setup.getSpaceInformation()));
		setup.getPlanner()->as<geometric::BITstar>()->setUseHalton(halton);
		
		setup.setup();

		setup.solve(time);
		if (setup.haveExactSolutionPath()) {
			double length = setup.getSolutionPath().length();
			std::cout << setup.getLastPlanComputationTime() << "\t" << length << std::endl;
			++success;
		} else {
			std::cout << time << "\t" << -1 << std::endl;
		}
	}
	
	std::cout << "Success rate:\t" << 100. * success/iterations << std::endl; 


    /*for (double time = 1.0 ; time < 10.1 ; time = time + 1.0)
    {
        setup.clear();
        double length = -1.0;
        // try to solve the problem
        if (setup.solve(time) && setup.haveExactSolutionPath())
            length = setup.getSolutionPath().length();
        res << "time = "  << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;
    }*/

    
    /*ompl::base::PlannerData data(setup.getSpaceInformation());
    setup.getPlanner()->getPlannerData(data);
    std::cout << data.numVertices() << std::endl;
    for (unsigned i = 0; i < data.numVertices(); ++i) {
		const base::State *s = data.getVertex(i).getState();
		std::cout << s->as<base::SE2StateSpace::StateType>()->getX() << "\t"
		          << s->as<base::SE2StateSpace::StateType>()->getY() << "\t"
		          << s->as<base::SE2StateSpace::StateType>()->getYaw() << std::endl;
	}*/


    return 0;
}
