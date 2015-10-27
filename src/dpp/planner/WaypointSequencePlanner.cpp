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
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/WaypointSequencePlanner.h>


namespace dpp {

/**
 * Adds the waypoints to the path planner from the list.  The first waypoint
 * in the list will be the initial position.
 * @param list  Waypoints to add.
 * @remark Previously added waypoints are cleared.
 */
void WaypointSequencePlanner::addWaypoints(const WaypointList list) {
    DPP_ASSERT(list.size() > 0);
    m_G.clear();
    m_originalNodeList.clear();
    m_newNodeList.clear();

    int i;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Adding list of waypoints: " << std::endl;
    for (const auto& waypoint : list) {
        i = addWaypoint(waypoint);
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    (" << waypoint.x << ", " << waypoint.y << ")" << std::endl;
    }

    // Some debug output
    #ifdef DPP_DEBUG

    // print the old list
    i =0;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Old node list: " << std::endl;
    for (const auto& u : m_originalNodeList) {
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    listi=" << i++ << " node " << u
        << " has oldIndex=" << m_sequenceTransformList[u].oldIndex << " newIndex="
        << m_sequenceTransformList[u].newIndex << std::endl;
    }

    // print the new list
    i =0;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "New node list: " << std::endl;
    for (const auto& u : m_newNodeList) {
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    listi=" << i++ << " node " << u
        << " has oldIndex=" << m_sequenceTransformList[u].oldIndex << " newIndex="
        << m_sequenceTransformList[u].newIndex << std::endl;
    }

    #endif // DPP_DEBUG
}

/**
 * Adds the waypoint to the planner. The first waypoint added will be the initial position.
 * @param waypoint  Waypoint to add.
 * @return Sequence id (index) of new waypoint.
 * @remark Waypoint coordinate-pair must be unique.
 */
int WaypointSequencePlanner::addWaypoint(const Waypoint waypoint) {
    DPP_ASSERT(!containsWaypoint(waypoint));
    // Create the new node with sequence id i
    int i = waypointCount();
    ogdf::node u = m_G.newNode();
    m_GA.x(u) = waypoint.x;
    m_GA.y(u) = waypoint.y;
    // FIXME should we reindex from 0 for onode ids?
    //       reading the tour sees: Out of range node id n in tour file.
    //       without this fix.
    m_GA.idNode(u) = i + 1; // nodes are indexed from 1

    // Configure the transform list for the waypoint
    m_sequenceTransformList[u].oldIndex = i;
    m_sequenceTransformList[u].newIndex = DPP_SEQUENCE_ID_NOT_SET;

    m_originalNodeList.push_back(u);
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    -> pushed node " << u << " index=" << i
        << " into list with " << m_originalNodeList.size() << std::endl;
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
    m_newNodeList.clear();

    bool result = DubinsVehiclePathPlanner::solve();
    if (!result) return result;

    // Find new sequence ids for transform list
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Building sequence transform list:" << std::endl;
    int i = 0;
    ogdf::ListIterator<ogdf::node> tourIter;
    for ( tourIter = m_Tour.begin(); tourIter != m_Tour.end(); tourIter++ ) {
        ogdf::node u = *tourIter;

        // Skip adding the origin to the end
        if (u == m_G.firstNode() && tourIter.succ() == m_Tour.end())
            break;

        m_sequenceTransformList[u].newIndex = i++;
        m_newNodeList.push_back(u);

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Node " << u << " id=" << m_GA.idNode(u) << ", orig="
            << m_sequenceTransformList[u].oldIndex << " changed to new="
            << m_sequenceTransformList[u].newIndex << std::endl;
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
    DPP_ASSERT(oldIndex >= 0 && oldIndex <= waypointCount());

    // Update: added initial point as #0 in sequence
    //oldIndex--; // reindex from 0, FIXME is this okay for QGC?

    ogdf::node u = m_originalNodeList[oldIndex];
    int newIndex = m_sequenceTransformList[u].newIndex;

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Getting oldIndex= " << oldIndex << " associated with node "
        << u << " id=" << m_GA.idNode(u) << " has newIndex=" << newIndex << " for oldIndex="
        << m_sequenceTransformList[u].oldIndex << std::endl;
    return newIndex;
}

/**
 * Returns the new, ordered waypoint sequence list with indices as the original sequence 
 * number. This should be used to reorder your original list to match the solution.
 */
std::vector<int> WaypointSequencePlanner::newWaypointSequenceList(void) {
    std::vector<int> oldIndexList;

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Building new waypoint sequence list: " << std::endl;
    for (const auto& u : m_newNodeList) {
        int oldIndex = m_sequenceTransformList[u].oldIndex;
        oldIndexList.push_back(oldIndex);

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    " << oldIndex << std::endl;
    }

    return oldIndexList;
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