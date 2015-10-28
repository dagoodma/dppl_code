/*
 * Simplified implementation of DubinsSensorPathPlanner. Does not require OGDF
 * objects. Only standard containers and the ones defined here.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/CoverageWaypointPlanner.h>

namespace dpp {


void CoverageWaypointPlanner::addPolygonVertex(Vertex v) {
	ogdf::DPoint p(v.x, v.y);
	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Adding point (" << p.m_x << ", "
		<< p.m_y << ") to polygon." << std::endl;
	if (!m_polygon.containsPoint(p)) {
		m_polygon.pushBack(p);
    }
}

/**
 * Find waypoints that efficiently cover the designated area.
 * @return true if the planner succeeded
 * @remark Call waypointList() afterwards to retreive the solution.
 */
bool CoverageWaypointPlanner::planCoverageWaypoints(void) {
	DPP_ASSERT(vertexCount() >= 3);
	Logger::logDebug() << "Here!";

	if(!solveAsDtsp()) {
		return false;
	}

	// Build waypoint list from tour
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Building waypoint list:" << std::endl;
    ogdf::ListIterator<ogdf::node> tourIter;
    for ( tourIter = m_Tour.begin(); tourIter != m_Tour.end(); tourIter++ ) {
        ogdf::node u = *tourIter;

        // Skip adding the origin to the end of the list
        if (u == m_G.firstNode() && tourIter.succ() == m_Tour.end())
            continue;

        Waypoint w = {m_GA.x(u), m_GA.y(u)};
        m_waypointList.push_back(w);

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Node " << u << " id=" << m_GA.idNode(u) << ", x="
            << w.x << ", y=" << w.y << std::endl;
    }

    return true;
}

} // namespace dpp