/*
 * Simplified implementation of DubinsVehiclePathPlanner that keeps track of 
 * the original sequence id (index) of added waypoints. Does not require OGDF
 * objects. Only standard containers.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */



#include <memory>
#include <cxxopts.h>
#include <stacktrace.h>

#include <solveCppAsDtsp.h>

#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/WaypointSequencePlanner.h>


namespace dpp {

/**
 * Adds the waypoints from the list to the path planner. 
 * @param list  Waypoints to add.
 * @remark Previously added waypoints are cleared.
 */
void WaypointSequencePlanner::addWaypoints(const WaypointList list) {
    DPP_ASSERT(list.size() > 0);
    m_G.clear();

    int i;
    for (const auto& waypoint : list) {
        i = addWaypoint(waypoint);
    }
}

/**
 * Adds the waypoint to the planner.
 * @param waypoint  Waypoint to add.
 * @return Sequence id (index) of new waypoint.
 * @remark Waypoint coordinate-pair must be unique.
 */
int WaypointSequencePlanner::addWaypoint(const Waypoint waypoint) {
    DPP_ASSERT(!_containsWaypoint(waypoint));
    // Create the new node with sequence id i
    int i = waypointCount() + 1;
    ogdf::node u = m_G.newNode();
    m_GA.x(u) = waypoint.x;
    m_GA.x(u) = waypoint.x;
    m_GA.idNode(u) = i;

    // Configure the transform list for the waypoint
    m_sequenceTransformList[u].oldIndex = i;
    m_sequenceTransformList[u].newIndex = DPP_SEQUENCE_ID_NOT_SET;

    m_originalNodeSequenceList.reserve(i);
    m_originalNodeSequenceList[i] = u;

    return i;
}

/**
 * Solve for the waypoint sequence that minimizes path length for the Dubins
 * vehicle.
 * @remark Retrieve the new sequence id through newWaypointSequenceId()
 */
void WaypointSequencePlanner::planWaypointSequence(void) {
    DPP_ASSERT(waypointCount() > 1);

    DubinsVehiclePathPlanner::solve();

    // Find new sequence ids
    int newIndex = 1;
    for ( tourIter = m_Edges.begin(); tourIter != m_Edges.end(); tourIter++ ) {
        ogdf::edge e = *tourIter;

        node u = e->source();
        node v = e->target();

        int newIndex
    }
}

/**
 * Get the new sequence id from solution.
 * @param oldIndex  Original sequence id of added waypoint.
 * @return New index of waypoint from solution.
 * @remark The planner must have a solution (see haveSolution() and planWaypointSequence())
 */
int WaypointSequencePlanner::newWaypointSequenceId(int oldIndex) {
    DPP_ASSERT(m_haveSolution);
    DPP_ASSERT(oldIndex > 0 && oldIndex <= waypointCount());

    ogdf::node u = m_originalNodeSequenceList[oldIndex];
    return m_sequenceTransformList[u].newIndex;
}


/**
 * Checks if a waypoint with the given coordinate pair was added.
 * @param x     X-coordinate of waypoint.
 * @param y     Y-coordinate of waypoint.
 * @return True if waypoint exists at x-y position.
 */
bool WaypointSequencePlanner::containsWaypoint(double x, double y) {
    node u;
    forall_nodes(u, m_G) {
        if (m_GA.x(u) == x && m_GA.y(u) == y) {
            return true;
        }
    }
    return false;
}