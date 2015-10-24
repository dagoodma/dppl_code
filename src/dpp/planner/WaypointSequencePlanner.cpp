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

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Adding list of waypoints: " << std::endl;
    for (const auto& waypoint : list) {
        i = addWaypoint(waypoint);
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    (" << waypoint.x << ", " << waypoint.y << ")" << std::endl;
    }
}

/**
 * Adds the waypoint to the planner.
 * @param waypoint  Waypoint to add.
 * @return Sequence id (index) of new waypoint.
 * @remark Waypoint coordinate-pair must be unique.
 */
int WaypointSequencePlanner::addWaypoint(const Waypoint waypoint) {
    DPP_ASSERT(!containsWaypoint(waypoint));
    // Create the new node with sequence id i
    int i = waypointCount() + 1;
    ogdf::node u = m_G.newNode();
    m_GA.x(u) = waypoint.x;
    m_GA.y(u) = waypoint.y;
    m_GA.idNode(u) = i;

    // Configure the transform list for the waypoint
    m_sequenceTransformList[u].oldIndex = i;
    m_sequenceTransformList[u].newIndex = DPP_SEQUENCE_ID_NOT_SET;

    m_originalNodeSequenceList.push_back(u);
/*
    m_originalNodeSequenceList.reserve(i);
    m_originalNodeSequenceList[i] = u;
*/
    return i;
}

/**
 * Solve for the waypoint sequence that minimizes path length for the Dubins
 * vehicle, then builds the waypoint sequence transform list for lookup.
 * @remark Retrieve the new sequence id through newWaypointSequenceId()
 * @return true if the planner succeeded
 */
bool WaypointSequencePlanner::planWaypointSequence(void) {
    DPP_ASSERT(waypointCount() > 1);
    bool result = DubinsVehiclePathPlanner::solve();

    if (!result) return result;

    // Find new sequence ids for transform list
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Building sequence transform list:" << std::endl;
    int i = 1;
    ogdf::ListIterator<ogdf::node> tourIter;
    for ( tourIter = m_Tour.begin(); tourIter != m_Tour.end(); tourIter++ ) {
        ogdf::node u = *tourIter;

        // Skip origin for returning tours
        if (u == m_G.firstNode() && tourIter.succ() == m_Tour.end())
            break;

        m_sequenceTransformList[u].newIndex = i++;

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Node " << m_GA.idNode(u) << " -> "
            << m_sequenceTransformList[u].newIndex << ", originally "
            << m_sequenceTransformList[u].oldIndex << std::endl;
    }

    return result;
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
        Logger::logDebug(DPP_LOGGER_VERBOSE_3) << "Comparing (" << x << ", " << y << ") with"
            << " node " << m_GA.idNode(u) << " (" << m_GA.x(u) << ", "
            << m_GA.y(u) << ")." << std::endl;

        if (m_GA.x(u) == x && m_GA.y(u) == y) {
            return true;
        }
    }
    return false;
}

} // namespace dpp