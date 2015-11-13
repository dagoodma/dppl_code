/*
 * Implementation of dpp::Field class for representing a coverage field. Used by
 * CPP algorithms for polygonal decomposition and field track generation.
 * 
 * findMinimumWidth() algorithm derived from the Xin Yu's dissertation, 2015.
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
#include <algorithm>

#include <dpp/basic/Field.h>
#include <dpp/basic/Logger.h>

namespace dpp {

/**
 * Constructor for building polygon from a list of points.
 */
Field::Field(PolyVertexList vertices, double coverageWidth) {
	m_coverageWidth = coverageWidth;
	//m_poly.clear();
    PolyVertexIterator iter;
    for ( iter = vertices.begin(); iter != vertices.end(); iter++ ) {
    	//DPP_ASSERT(!m_poly.containsPoint(*iter)); // breaks for non-convex
    	m_poly.pushBack(*iter);
    }
    m_poly.unify(); // delete duplicates
    computeBoundingBox();
}

/**
 * Computes the bounding box by finding the minimum and maximum of both
 * Y and X coordinates of all vertices of field's defining polygon.
 */
void Field::computeBoundingBox(void) {
	DPP_ASSERT(m_poly.size() > 0);
	m_minXVertex = m_poly.begin();
	m_minYVertex = m_poly.begin();
	m_maxXVertex = m_poly.begin();
	m_maxYVertex = m_poly.begin();

    PolyVertexIterator iter;
    for ( iter = m_poly.begin(); iter != m_poly.end(); iter++ ) {
        if ((*iter).m_x < (*m_minXVertex).m_x) {
            m_minXVertex = iter;
        }
        if ((*iter).m_y < (*m_minYVertex).m_y) {
            m_minYVertex = iter;
        }
        if ((*iter).m_x > (*m_maxXVertex).m_x) {
            m_maxXVertex = iter;
        }
        if ((*iter).m_y > (*m_maxYVertex).m_y) {
            m_maxYVertex = iter;
        }
    }

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found bounding rectangle for polygon: "
        << "min_x=" << (*m_minXVertex).m_x << ", max_x=" << (*m_maxXVertex).m_x
        << ", min_y=" << (*m_minYVertex).m_y << ", max_y=" << (*m_maxYVertex).m_y
        << std::endl; 
}

/**
 * Test if polygon is defined with points counter-clockwise.
 */
bool Field::isCcw(void) {
    return m_poly.counterclock(); // FIXME check for ccw
}


/**
 * Test if a polygon is convex.
 * @see http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
 * FIXME look into using DSegment.det(), or Eigen cross product
 */
bool Field::isConvex(void) {
	DPP_ASSERT(m_poly.size() > 2);

    if (m_poly.size() <= 3)
        return true;

    bool sign = false, first = true,
        result = true;

    PolyVertexIterator listIter;
    for ( listIter = m_poly.begin(); listIter != m_poly.end()
        && listIter.succ() != m_poly.end(); listIter++ ) {
        DSegment u = m_poly.segment(listIter);
        DSegment v = m_poly.segment(listIter.succ());
        double zcross = v.dx()*u.dy() - v.dy()*u.dx();
        /*
        std::cout << "seg1: (" << u.start().m_x << ", " << u.start().m_y << ") -> ("
            << u.end().m_x << ", " << u.end().m_y << ")" << std::endl;
        std::cout << "seg2: (" << v.start().m_x << ", " << v.start().m_y << ") -> ("
            << v.end().m_x << ", " << v.end().m_y << ")" << std::endl;
        std::cout << "zcross: " << zcross << std::endl;
        */
        if (first) {
            sign = zcross > 0;
            first = false;
        }
        else {
            if (sign != (zcross > 0)) {
                result = false;
                break;
            }
        }
    }
    if (!sign != m_poly.counterclock()) {
        Logger::logWarn() << "Polygon " << ((m_poly.counterclock())? "CCW" : "CW")
            << " was expected, but sign did not match" << std::endl;
    }
    return result;
}

/**
 * Finds the minimum width of a convex polygon.
 * @param[out] width The width, or minimum altitude of the polygon.
 * @param[out] angle The optimal coverage orientation angle for generating track lines.
 * @return Success (=0) or error code.
 * @see Algorithm 1, page 20 of Xin Yu dissertation 2015
 */
int Field::findMinimumWidth(double &width, double &angle) {
	DPP_ASSERT(m_poly.size() > 2);
    DPP_ASSERT(isConvex());
    // TODO delete middle vertices of any collinear sequence of three vertices
    //      see DPolygon::normalize()
    PolyVertexIterator va = m_minYVertex; //findVertexWithMinY();
    PolyVertexIterator vb = m_maxYVertex; //findVertexWithMaxY();

    /*
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Finding min width of polygon: " << std::endl;
    PolyVertexIterator listIter;
    for ( listIter = m_poly.begin(); listIter != m_poly.end(); listIter++ ) {
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "    " << *listIter << std::endl;
    }
    */
    double rotAngle = 0;
    angle = 0;
    width = std::numeric_limits<double>::infinity();

    Vector2d caliper_a = Vector2d(1,0); // unit vector on positive x-axis
    Vector2d caliper_b = Vector2d(-1,0); // unit vector on negative x-axis

    /*
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Va: (" << (*va).m_x << ", " << (*va).m_y << ")" << std::endl
        << "Vb: (" << (*vb).m_x << ", " << (*vb).m_y << ")" << std::endl
        << "Cal_a: (" << caliper_a[0] << ", " << caliper_a[1] << "); " << std::endl
        << "Cal_b: (" << caliper_b[0] << ", " << caliper_b[1] << "); " << std::endl;
    */ 

    while (rotAngle < M_PI) {
        Vector2d ea = segmentToVector(m_poly.segment(va));
        Vector2d eb = segmentToVector(m_poly.segment(vb));

        double aa =  angleBetween(caliper_a, ea);
        double ab =  angleBetween(caliper_b, eb);
        /*
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "----" << std::endl << "Ea: (" << ea[0] << ", " << ea[1] << "); " << std::endl
            << "Eb: (" << eb[0] << ", " << eb[1] << "); " << std::endl
            << "Angle_a=" << radToDeg(aa) << ", Angle_b=" << radToDeg(ab) << std::endl;
        */
        double altitude = 0;
        if (aa < ab) {
            // Rotate calipers by angle aa
            Eigen::Rotation2D<double> t(aa);
            caliper_a = t * caliper_a;
            caliper_b = t * caliper_b;
            altitude = distanceToCaliper(*vb, *va, caliper_a);
            //altitude = (*vb).distance(*va); // caliper a is at va //getDistanceFromVector(*vb, caliper_a);

            va = m_poly.cyclicSucc(va);
            rotAngle += aa;
            /*
            Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Cal_a: (" << caliper_a[0] << ", " << caliper_a[1] << "); " << std::endl
                << "Cal_b: (" << caliper_b[0] << ", " << caliper_b[1] << "); " << std::endl
                << "Va: (" << (*va).m_x << ", " << (*va).m_y << ")" << std::endl
                << "Rotang: " << radToDeg(rotAngle) << std::endl
                << "Alt: " << altitude << std::endl;
                */
        }
        else {
            // Rotate calipers by angle ab
            Eigen::Rotation2D<double> t(ab);
            caliper_a = t * caliper_a;
            caliper_b = t * caliper_b;

            altitude = distanceToCaliper(*va, *vb, caliper_b);
            //altitude = (*va).distance(*vb); //getDistanceFromVector(*va, caliper_b);
            vb = m_poly.cyclicSucc(vb);
            rotAngle += ab;
            /*
            Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Cal_a: (" << caliper_a[0] << ", " << caliper_a[1] << "); " << std::endl
                << "Cal_b: (" << caliper_b[0] << ", " << caliper_b[1] << "); " << std::endl
                << "Vb: (" << (*vb).m_x << ", " << (*vb).m_y << ")" << std::endl
                << "Rotang: " << radToDeg(rotAngle) << std::endl
                << "Alt: " << altitude << std::endl;
                */
        }

        if (altitude < width) {
            width = altitude;
            angle = rotAngle;
            /*
            Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "*** Altitude="<< altitude << ", minwidth="<<width<<", angle="
                << radToDeg(rotAngle) << std::endl;
                */
        }
    } // while()
    angle = myMod(angle, M_PI);

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found polygon minimum width=" << width
    	<< ", and orientation angle=" << radToDeg(angle) << " deg" << std::endl;

    return SUCCESS;
}


/**
 * Grids the polygon with squares of sensor width e, and then adds the grid tile
 * centroids as nodes to the graph.
 * @note This function is used in converting the CPP problem into a DTSP.
 * @param G     Graph to add nodes to.
 * @param GA    Attributes of the graph.
 */
int Field::addNodesFromGrid(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
	DPP_ASSERT(m_poly.size() > 2);

    // Get bounding rectangle
    double min_x = (*m_minXVertex).m_x,
           max_x = (*m_maxXVertex).m_x,
           min_y = (*m_minYVertex).m_y,
           max_y = (*m_maxYVertex).m_y;

    // Add grid tile centroids to the graph if they are inside the polygon
    int i = 2;
    for (double x=min_x; x <= max_x; x += m_coverageWidth) {
        for (double y=min_y; y <= max_y; y += m_coverageWidth) {
            DPoint p(x,y);
            if (m_poly.containsPoint(p)) {
                ogdf::node u = G.newNode();
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

} // namespace dpp