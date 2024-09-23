#include "CollisionChecking.h"
#include "RTP.h"

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/PlannerStatus.h>
#include <ompl/base/PlannerTerminationCondition.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <iostream>
#include <vector>
#include <fstream>

namespace ob = ompl::base;

// State validity checker using the isValidPoint function
bool isPointStateValid(const ompl::base::State *state, const std::vector<Rectangle> &obstacles)
{
    const auto *point = state->as<ompl::base::RealVectorStateSpace::StateType>();
    double x = point->values[0];
    double y = point->values[1];
    return isValidPoint(x, y, obstacles);
}

// State validity checker using the isValidSquare function
bool isBoxStateValid(const ompl::base::State *state, const std::vector<Rectangle> &obstacles, double hbound, double lbound)
{
    const auto *se2State = state->as<ompl::base::SE2StateSpace::StateType>();
    double x = se2State->getX();
    double y = se2State->getY();
    double theta = se2State->getYaw();
    double sideLength = 1.0; // Set the side length as needed
    return isValidSquare(x, y, theta, sideLength, obstacles, hbound, lbound);
}

void planPoint(const std::vector<Rectangle> &obstacles)
{
    // Construct the 2D state space
    auto space(std::make_shared<ompl::base::RealVectorStateSpace>(2));

    // Set the bounds for the 2D plane
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0);   // x lower bound
    bounds.setHigh(0, 20.0); // x upper bound
    bounds.setLow(1, 0.0);   // y lower bound
    bounds.setHigh(1, 20.0); // y upper bound
    space->setBounds(bounds);

    // Construct an instance of space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // Set state validity checking for this space
    si->setStateValidityChecker([&obstacles](const ompl::base::State *state)
                                { return isPointStateValid(state, obstacles); });

    // Create a start state
    ompl::base::ScopedState<> start(space);
    start[0] = 5.5;
    start[1] = 6.0;
    // start.random();

    // Create a goal state
    ompl::base::ScopedState<> goal(space);
    goal[0] = 17.5;
    goal[1] = 12.5;
    // goal.random();

    // Create a problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RTP>(si));

    // Set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // Perform setup steps for the planner
    planner->setup();

    // Print the settings for this space
    si->printSettings(std::cout);

    // Print the problem settings
    pdef->print(std::cout);

    // Define a planner termination condition for a time limit of 1 second
    // ompl::base::PlannerTerminationCondition ptc(1.0);

    // Attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = planner->ob::Planner::solve(30.0);

    if (solved)
    {
        // Get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // Print the path to screen
        path->print(std::cout);

        // Cast the path to PathGeometric and save to a txt file
        auto *geoPath = path->as<ompl::geometric::PathGeometric>();
        if (geoPath)
        {
            std::ofstream outFile("point_robot_solution.txt");
            outFile << "R2" << std::endl;
            geoPath->printAsMatrix(outFile); // Dump the path to a file
            outFile.close();
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void planBox(const std::vector<Rectangle> &obstacles)
{
    // Construct the SE(2) state space (2D position + orientation)
    auto space(std::make_shared<ompl::base::SE2StateSpace>());

    // Set the bounds for the 2D plane and orientation
    ompl::base::RealVectorBounds bounds(2);
    bounds.setLow(0, 0.0);   // x lower bound
    bounds.setHigh(0, 20.0); // x upper bound
    bounds.setLow(1, 0.0);   // y lower bound
    bounds.setHigh(1, 20.0); // y upper bound
    bounds.setLow(2, -M_PI); // theta lower bound
    bounds.setHigh(2, M_PI); // theta upper bound
    space->setBounds(bounds);

    // Retrieve the environment bounds
    double lbound = bounds.low[0];  // x lower bound
    double hbound = bounds.high[1]; // y upper bound

    // Construct an instance of space information from this state space
    auto si(std::make_shared<ompl::base::SpaceInformation>(space));

    // Set state validity checking for this space
    si->setStateValidityChecker([&obstacles, hbound, lbound](const ompl::base::State *state)
                                { return isBoxStateValid(state, obstacles, hbound, lbound); });

    // Create a random start state
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::SE2StateSpace::StateType>()->setX(5.4);
    start->as<ompl::base::SE2StateSpace::StateType>()->setY(6.0);
    start->as<ompl::base::SE2StateSpace::StateType>()->setYaw(0.0);
    // start.random();

    // Create a random goal state
    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::SE2StateSpace::StateType>()->setX(17.5);
    goal->as<ompl::base::SE2StateSpace::StateType>()->setY(12.5);
    goal->as<ompl::base::SE2StateSpace::StateType>()->setYaw(M_PI/4);
    // goal.random();

    // Create a problem instance
    auto pdef(std::make_shared<ompl::base::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create a planner for the defined space
    auto planner(std::make_shared<ompl::geometric::RTP>(si));

    // Set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // Perform setup steps for the planner
    planner->setup();

    // Print the settings for this space
    si->printSettings(std::cout);

    // Print the problem settings
    pdef->print(std::cout);

    // Attempt to solve the problem within one second of planning time
    ompl::base::PlannerStatus solved = planner->ob::Planner::solve(60.0);

    if (solved)
    {
        // Get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        ompl::base::PathPtr path = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // Print the path to screen
        path->print(std::cout);

        // Cast the path to PathGeometric and save to a txt file
        auto *geoPath = path->as<ompl::geometric::PathGeometric>();
        if (geoPath)
        {
            std::ofstream outFile("box_robot_solution.txt");
            outFile << "SE2 1.0" << std::endl;
            geoPath->printAsMatrix(outFile); // Dump the path to a file
            outFile.close();
        }
    }
    else
    {
        std::cout << "No solution found" << std::endl;
    }
}

void makeEnvironment1(std::vector<Rectangle> &obstacles)
{
    // Define a simple environment with some rectangular obstacles
    obstacles.clear(); // Clear any existing obstacles

    // Boundary walls
    obstacles.push_back({0.0, 0.0, 20.0, 1.0});
    obstacles.push_back({0.0, 19.0, 20.0, 1.0});
    obstacles.push_back({0.0, 0.0, 1.0, 20.0});
    obstacles.push_back({19.0, 0.0, 1.0, 20.0});

    // Inner obstacles forming the maze
    obstacles.push_back({3.0, 3.0, 5.0, 1.5});
    obstacles.push_back({3.0, 3.0, 1.5, 14.0});
    obstacles.push_back({6.5, 0.0, 1.5, 16.0});
    obstacles.push_back({10.5, 4.0, 1.5, 16.0});
    obstacles.push_back({14.5, 4.0, 1.5, 12.0});
    obstacles.push_back({15.0, 14.5, 5.0, 1.5});
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    // Define a more complex or different environment
    obstacles.clear(); // Clear any existing obstacles

    obstacles.push_back({0.0, 0.0, 20.0, 1.0});
    obstacles.push_back({0.0, 19.0, 20.0, 1.0});
    obstacles.push_back({0.0, 0.0, 1.0, 20.0});
    obstacles.push_back({19.0, 0.0, 1.0, 20.0});

    obstacles.push_back({6.0, 0.0, 1.5, 9.0});
    obstacles.push_back({6.0, 10.5, 1.5, 9.5});
    obstacles.push_back({12.0, 0.0, 1.5, 9.0});
    obstacles.push_back({12.0, 10.5, 1.5, 9.5});
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Maze with false paths" << std::endl;
        std::cout << " (2) Flappy Bird Environment" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
    case 1:
        makeEnvironment1(obstacles);
        break;
    case 2:
        makeEnvironment2(obstacles);
        break;
    default:
        std::cerr << "Invalid Environment Number!" << std::endl;
        break;
    }

    switch (robot)
    {
    case 1:
        planPoint(obstacles);
        break;
    case 2:
        planBox(obstacles);
        break;
    default:
        std::cerr << "Invalid Robot Type!" << std::endl;
        break;
    }

    return 0;
}