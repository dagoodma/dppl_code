/*
 * Example showing how to build a simple path planner that solves for an efficient
 * sequence of waypoints. This is the same interface used in the QGC implementation.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdio.h>

#include <dpp/basic/basic.h>
#include <dpp/planner/WaypointSequencePlanner.h>

#define TURN_RADIUS             10.0 // [m]
#define INITIAL_HEADING_ANGLE   0.0 // [rad]

// Waypoint scenario to solve. First point is initial point.
const dpp::WaypointList waypoints {
    {0, 0},
    {8.1*TURN_RADIUS, -10.3*TURN_RADIUS},
    {5.1*TURN_RADIUS, 0.0},
    {-2.0*TURN_RADIUS, -3.3*TURN_RADIUS}
};

/**
 * This example code solves the waypoint pattern defined above with a turn radius
 * of 10 meters, and an initial heading of 0 radians (north). The initial position
 * is the first node in the loaded .gml graph.
 */
int main(int argc, char *argv[]) {

    // Build path planner and find a solution
    dpp::WaypointSequencePlanner p;
    p.initialHeading(INITIAL_HEADING_ANGLE);
    p.turnRadius(TURN_RADIUS);
    p.addWaypoints(waypoints);

    p.planWaypointSequence(); // synonymous with solve()

    // Get and print re-ordered list
    std::vector<int> newSequenceList = p.newWaypointSequenceList();
    double cost = p.cost();

    std::cout << "Solved " << p.waypointCount() << " point tour with cost " << cost << "." << std::endl;
    std::cout << "New waypoint order: { ";
    bool first = true;
    for (const auto& i : newSequenceList) {
        if (!first) std::cout << ", ";
        std::cout << i;
        first = false;
    }
    std::cout << " }" << std::endl;

    return 0;
}