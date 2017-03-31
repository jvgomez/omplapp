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

#include <omplapp/apps/SE2RigidBodyPlanning.h>
#include <ompl/geometric/planners/bitstar/BITstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/tools/multiplan/OptimizePlan.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <omplapp/config.h>


using namespace ompl;

int main()
{
    // plan in SE2
    app::SE2RigidBodyPlanning setup;

    // load the robot and the environment
    std::string robot_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/car2_planar_robot.dae";
    std::string env_fname = std::string(OMPLAPP_RESOURCE_DIR) + "/2D/Maze_planar_env.dae";
    setup.setRobotMesh(robot_fname.c_str());
    setup.setEnvironmentMesh(env_fname.c_str());

    // define start state
    base::ScopedState<base::SE2StateSpace> start(setup.getSpaceInformation());
    start->setX(0.01);
    start->setY(-0.15);
    start->setYaw(0.0);

    // define goal state
    base::ScopedState<base::SE2StateSpace> goal(start);
    goal->setX(41.01);
    goal->setY(-0.15);
    goal->setYaw(0.802851455917);

    // set the start & goal states
    setup.setStartAndGoalStates(start, goal);

    // setting collision checking resolution to 1% of the space extent
    setup.getSpaceInformation()->setStateValidityCheckingResolution(0.01);

    // make sure the planners run until the time limit, and get the best possible solution
    setup.getProblemDefinition()->setOptimizationObjective(
        std::make_shared<base::PathLengthOptimizationObjective>(setup.getSpaceInformation()));

    setup.setup();

    std::stringstream res;

    // run with RRT*
    setup.setPlanner(std::make_shared<geometric::BITstar>(setup.getSpaceInformation()));
    //setup.setPlanner(std::make_shared<geometric::RRTstar>(setup.getSpaceInformation()));
    setup.solve(1);

    if (setup.haveExactSolutionPath()) {
        double length = setup.getSolutionPath().length();
        res << "time = "  << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;
    }
    /*for (double time = 1.0 ; time < 10.1 ; time = time + 1.0)
    {
        setup.clear();
        double length = -1.0;
        // try to solve the problem
        if (setup.solve(time) && setup.haveExactSolutionPath())
            length = setup.getSolutionPath().length();
        res << "time = "  << setup.getLastPlanComputationTime() << " \t length = " << length << std::endl;
    }*/

    std::cout << res.str();
    
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
