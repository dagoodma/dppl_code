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
bool Field::isCcw(void) const {
    return m_poly.counterclock(); // FIXME check for ccw
}


/**
 * Test if a polygon is convex.
 * @see http://stackoverflow.com/questions/471962/how-do-determine-if-a-polygon-is-complex-convex-nonconvex
 * FIXME look into using DSegment.det(), or Eigen cross product
 */
bool Field::isConvex(void) const {
	DPP_ASSERT(m_poly.size() > 2);

    if (m_poly.size() <= 3)
        return true;

    bool sign = false, first = true,
        result = true;

    PolyVertexConstIterator listIter;
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
    bool polySign = !((DPolygon)m_poly).counterclock(); // sign = 1 means CW
    if (sign != polySign) {
        Logger::logWarn() << "Polygon " << ((!polySign)? "CCW" : "CW")
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
int Field::findMinimumWidth(double &width, double &angle) const {
	DPP_ASSERT(m_poly.size() > 2);
    DPP_ASSERT(isConvex());
    // TODO delete middle vertices of any collinear sequence of three vertices
    //      see DPolygon::normalize()
    PolyVertexConstIterator va(m_minYVertex); //findVertexWithMinY();
    PolyVertexConstIterator vb(m_maxYVertex); //findVertexWithMaxY();

    /*
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Finding min width of polygon: " << std::endl
    	<< this << std::endl;
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
 * Generates a list of parallel field tracks at the given angle.
 * @remark Uses field's coverageWidth to space the tracks apart.
 * @return Number of field tracks generated.
 */
int Field::generateFieldTracks(FieldTrackList &tracks) const {
	//DPP_ASSERT(0 <= angle && angle < 2*M_PI);
	tracks.clear();

	// FIXME remove this comment // This is only used here for debugging
	double polygonWidth;
	double coverageAngle;
	findMinimumWidth(polygonWidth, coverageAngle); 
	/*
	if (!DIsEqual(angle, optimalCoverageAngle)) {
		Logger::logWarn(DPP_LOGGER_VERBOSE_1)
			<< "Field tracks are at non-optimal coverage angle "
			<< radToDeg(optimalCoverageAngle) << " degrees." << std::endl;
	}
	*/

	// Find the sweep-line segment and angle
	// FIXME remove requirement to start with existing segment
	DSegment sweepSegment;
	if (!findPolySegmentWithAngle(coverageAngle, polygon(),
		sweepSegment, false)) {
    	throw std::runtime_error("Failed to find sweep segment.");
	}
	//double sweepAngle = wrapAngle(angleOfSegment(sweepSegment) + M_PI/2);
	double sweepAngle = wrapAngle(angleOfSegment(sweepSegment) + M_PI/2);
	FieldTrackSweepLine sweepLine(sweepSegment);
	sweepLine.translatePolar(m_coverageWidth/2, sweepAngle); // shift the initial line
	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Initial sweep line: " << sweepLine
		<< std::endl;

	// Generate tracks to cover field
	int nTurn = ceil(polygonWidth/m_coverageWidth);
	Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Generating " << nTurn << " field tracks "
		<< " at " << radToDeg(coverageAngle) << " degrees." << std::endl;
	bool finished = false; // FIXME ensure no infinte looping
	while (!finished) {
		FieldTrack t;
		if (sweepLine.intersectingTrack(this, t)) {
			tracks.pushBack(t);
			sweepLine.translatePolar(m_coverageWidth, sweepAngle);
		}
		else {
			finished = true;
		}
	}

	if (tracks.size() != nTurn) {
		Logger::logWarn() << "Expected to find " << nTurn << " field tracks, but got "
			<< tracks.size() << std::endl;
	}

	Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Generated " << tracks.size()
		<< " field tracks." << std::endl;

	return tracks.size();
}

/**
 * Finds the first segment with the given angle in the polygon.
 * @param[in] angle of segment to search for from [0, 2*pi)
 * @param[in] poly  polygon to search over for segment
 * @param[out] s    segment to hold result if found
 * @param[in] dir    whether direction of segment should matter
 * @remark If dir is false, the search angle and segment angle will be shifted to [0, pi)
 * @returns true or false if the segment was found
 */
bool findPolySegmentWithAngle(double angle, const DPolygon *poly, DSegment &seg, bool dir) {
    DPP_ASSERT(0 <= angle && angle < 2*M_PI);
    DPP_ASSERT(poly->size() > 0);
    bool result = false;

    PolyVertexConstIterator iter;
    for ( iter = poly->begin(); iter != poly->end(); iter++ ) {
        DSegment s = poly->segment(iter);
        double sAngle = angleOfSegment(s);
        if (!dir && angle >= M_PI) 
            angle = angle - M_PI;
        if (!dir && sAngle >= M_PI) 
            sAngle = sAngle - M_PI;

        if (DIsEqual(angle, sAngle)) {
            result = true;
            seg = s;
            break; // find the first segment
        }
    }
    return result;
}


/**
 * Grids the polygon with squares of sensor width e, and then adds the grid tile
 * centroids as nodes to the graph.
 * @note This function is used in converting the CPP problem into a DTSP.
 * @param G     Graph to add nodes to.
 * @param GA    Attributes of the graph.
 */
int Field::addNodesFromGrid(ogdf::Graph &G, ogdf::GraphAttributes &GA) const {
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

/**
 * Create a field track by finding two intersection points with the field.
 * @param[in] f field to find intersections with
 * @param[out] t
 */
bool FieldTrackSweepLine::intersectingTrack(const Field *field, FieldTrack &track) const {
	ogdf::List<DPoint> inter; // intersection points
	int result = false;

	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Finding field track with sweep-line "
		<< *this << " on polygon: " << std::endl << *field << std::endl;

	// Check for intersection with all segments
    PolyVertexConstIterator iter;
    bool onTopOf = false; // whether the sweep line lies on an edge segment
    for ( iter = field->polygon()->begin(); iter != field->polygon()->end(); iter++ ) {
    	DSegment s = field->polygon()->segment(iter);
    	DPoint ip;
    	if (intersection(s, ip)) {
    		inter.pushBack(ip);
    		Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found intersection point " << ip
    			<< " with segment: " << s.start() << " -> " << s.end() << std::endl;
    	}
    	// Check if on top of
        Line2d line(s);
        if (line == *this) {
        	onTopOf = true;
        }
    }

    // Handle intersection points
    if (onTopOf) {// We're at a vertex of the polygon
		Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Collinear with edge segment." << std::endl;
    }
    else if (inter.size() == 1) {
    	// We're at a vertex of the polygon
		Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Only one intersection point found." << std::endl;
    }
    else if (inter.size() == 2) {
    	// Found a cross-section of the polygon
    	DSegment s(*(inter.get(0)), *(inter.get(1)));
    	track = FieldTrack(s);
    	result = true;
    }
    else if (inter.size() > 2) {
    	// Polygon is non-convex and or complex
    	throw std::domain_error("Expected polygon to be convex.");
    }
    else {
    	// We're collinear with an edge, or not touching the polygon at all
		Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "No intersection points found." << std::endl;
    }
    return result;
}

} // namespace dpp