/*
 * Implementation of dpp::FieldTrack class for representing a track on a field for
 * vehicle traversal. The track is a line that contains 2 nodes on a graph as endpoints.
 * 
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <dpp/basic/FieldTrack.h>

namespace dpp {

using ogdf::node;
using ogdf::edge;

ogdf::node addNodeToGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA, DPoint p) {
	//int newNodeId = G.maxNodeIndex() + 1; // origin id=1 should already be in the graph
	int newNodeId = (G.maxNodeIndex() < 0)? 1
		: G.maxNodeIndex() + 2; // indexing starts from 1 or else MATLAB and maybe QGC will crash

	node u = G.newNode();
	GA.x(u) = p.m_x;
	GA.y(u) = p.m_y;
	GA.idNode(u) = newNodeId;

	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Added node " << newNodeId
		<< ": " << p << " to the graph." << std::endl;

	return u;
}


int addNodesFromFieldTrack(const FieldTrack &t, ogdf::Graph &G,
	ogdf::GraphAttributes &GA, double distance, ogdf::List<ogdf::node> &Tour,
 	ogdf::NodeArray<double> &Headings) {
	int n = 0;

	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Adding nodes to cover field track: "
		<< t << std::endl;

	double angle = (!t.reverse())? t.angle()
		: wrapAngle(t.angle() + M_PI);

	// Add the start node
	//DPoint p = (!t.reverse())? t.start() : t.end();
	DPoint p = t.start();
	Tour.pushBack(addNodeToGraph(G, GA, p));
	Headings(Tour.back()) = angleToHeading(angle);
	n++;

	// Add nodes along the track, distance apart
	if (t.length() > distance) {
		p += polarToCartesian(angle, distance);
		while(t.contains(p) && p != t.end()) {
			Tour.pushBack(addNodeToGraph(G, GA, p));
			Headings(Tour.back()) = angleToHeading(angle);
			n++;

			p += polarToCartesian(angle, distance);
		}

	}

	// Add the end node
	if (t.length() >= distance) {
		//p = (!t.reverse())? t.start() : t.end();
		Tour.pushBack(addNodeToGraph(G, GA, t.end()));
		Headings(Tour.back()) = angleToHeading(angle);
		n++;
	}

	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Added " << n
		<< " nodes to cover field track." << std::endl;

	return n;
}


/**
 * Decomposes the list into nodes placed distance apart, and adds them to the
 * graph. Builds a tour with headings.
 * @param[in] G graph to add nodes to
 * @param[in] GA graph attributes
 * @param[in] distance between nodes along each track
 * @param[out] Tour ordered list to save the nodes
 * @param[out] Headings to save node heading along track
 * @return the number of nodes added to the graph
 * @remark FIXME does not check if graph contains identical nodes
 */
 int FieldTrackList::addNodesToGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA, 
 	double distance, ogdf::List<ogdf::node> &Tour,
 	ogdf::NodeArray<double> &Headings) const {
 	int n = 0;
 	// Clear the tour and add the origin node
 	Tour.clear();
 	Tour.pushBack(G.firstNode());
 	//Headings.init(G);

 	// Iterate over the field tracks and create nodes
 	FieldTrackListConstIterator iter;
 	for (iter = begin(); iter != end(); iter++) {
 		const FieldTrack t = *iter;
 		n += addNodesFromFieldTrack(t, G, GA, distance, Tour, Headings);
 	}
 	return n;
}

} // namespace dpp