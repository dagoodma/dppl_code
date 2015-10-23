/*
 * Implementation of dpp::FieldTracksCPP class for solving CPP problems
 * by decomposing the field into convex polygons, adding minimum altitude
 * tracks for each polygon, and solving for the optimal visiting order
 * with LKH by converting this GTSP into an ATSP.
 *
 * Derived from the Xin Yu's dissertation, 2015.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <cmath>
#include <limits>
#include <cstddef>
#include <iostream>

#include <dpp/basic/basic.h>
#include <dpp/planalg/FieldTracksCpp.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Path.h>

using ogdf::DPoint;

namespace dpp {

/**
 * Grids the polygon with squares of sensor width, and then adds the grid tile
 * centroids as nodes to the graph.
 * @note This function is used in converting the CPP problem into a DTSP.
 * @param G     Graph to add nodes to.
 * @param GA    Attributes of the graph.
 * @param poly  Polygon to grid.
 * @param e     Sensor width in meters for sizing sides of grid tiles.
 */
int addNodesFromPolygonGrid(Graph &G, GraphAttributes &GA, DPolygon &poly, double e) {

    // Find a bounding rectangle
    // FIXME use GraphAttributes::boundingBox()
    double min_x = std::numeric_limits<double>::max(),
           max_x = - std::numeric_limits<double>::max(),
           min_y = std::numeric_limits<double>::max(),
           max_y = - std::numeric_limits<double>::max();

    ListIterator<ogdf::DPoint> iter;
    for ( iter = poly.begin(); iter != poly.end(); iter++ ) {
        ogdf::DPoint p = *iter;
        if (p.m_x < min_x) min_x = p.m_x;
        if (p.m_x > max_x) max_x = p.m_x;
        if (p.m_y < min_y) min_y = p.m_y;
        if (p.m_y > max_y) max_y = p.m_y;
    }

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found bounding rectangle for polygon: "
        << "min_x=" << min_x << ", max_x=" << max_x << ", min_y=" << min_y << ", max_y="
        << max_y << std::endl; 

    // Add grid tile centroids to the graph if they are inside the polygon
    int i = 2;
    for (double x=min_x; x <= max_x; x += e) {
        for (double y=min_y; y <= max_y; y += e) {
            DPoint p(x,y);
            if (poly.containsPoint(p)) {
                node u = G.newNode();
                GA.x(u) = p.m_x;
                GA.y(u) = p.m_y;
                GA.idNode(u) = i++;
            }
        }
    }

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Built graph from tiled polygon." << std::endl
        << printGraph(G, GA);


    return i - 2; // subtract origin and extra i++
}

int FieldTracksCpp::run(void) {
    Logger::logWarn() << "Field Tracks algorithm not implemented!" << std::endl;

    return SUCCESS;
}

} // namespace dpp