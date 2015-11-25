/*
 * Implementation of dpp::BoustrophdeonCpp class for solving CPP problems
 * by generating field tracks that cover a convex field, and decomposing
 * the tracks into waypoints by joining the tracks end-to-end.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <dpp/basic/Logger.h>
#include <dpp/basic/Path.h>

#include <dpp/planalg/BoustrophedonCpp.h>

namespace dpp {

/**
 * Order field tracks in the track list according to the Boustrophedon CPP
 * algorithm. Track direction (reverse or not) will be set.
 * @remark This sets the tracks to reversed in an alternating order.
 * @param[in] tracks field track list to reorder
 * @param[in] startAtLast whether to use the last track as the first
 * @param[in] reverseStartTrack whether to start in the reverse direction
 * 	on the first track (or last depending).
 */
void setFieldTrackOrder(FieldTrackList &tracks, bool startAtLast,
	bool reverseStartTrack) {
	if (startAtLast) {
		tracks.reverse();
	}

	bool directionReversed = reverseStartTrack;
 	FieldTrackListIterator iter;
 	for (iter = tracks.begin(); iter != tracks.end(); iter++) {
 		(*iter).reverse(directionReversed);
 		directionReversed ^= 1; // flip direction
 	}
}

/** 
 * Solves the CPP problem using the Boustrophedon algorithm.
 * @param[in] field a coverage field to generate waypoints over
 * @param[in] C initial configuration of the vehicle
 * @param[in] turnRadius of the vehicle
 * @param[in] returnToInitial whether to include return edge and cost
 * @param[out] G a graph to populate with nodes for coverage
 * @param[out] GA attributes of the graph
 * @param[out] Tour an ordered list of nodes representing the path
 * @param[out] Edges an ordered list of edges representing the path
 * @param[out] Headings headings used along the path at each node
 * @param[out] cost overall cost (path length) for covering the field
 * @return SUCCESS or FAILURE of the algorithm
 */
int BoustrophedonCpp::run(const Field &field, VehicleConfiguration C, double turnRadius,
	bool returnToInitial, Graph &G, GraphAttributes &GA, List<node> &Tour,
	List<edge> &Edges, NodeArray<double> &Headings, double &cost) {

    // Check arguments
    if (Headings.graphOf() != &G) {
        throw std::domain_error("Headings should be for G.");
    }

    // Generate field tracks
    FieldTrackList tracks;
    if (field.generateFieldTracks(tracks) < 1) {
    	throw std::runtime_error("Failed to generate field tracks.");
    }

    // Find the closest endpoint among first or last track from initial config
    bool startAtLast = false;
    bool reverseStartTrack = false;

    FieldTrack firstTrack = tracks.front();
    FieldTrack lastTrack = tracks.back();
    VehicleConfiguration
    	CfirstStart(firstTrack.start().m_x, firstTrack.start().m_y, firstTrack.angle()),
    	CfirstEnd(firstTrack.end().m_x, firstTrack.end().m_y, firstTrack.angle(true)),
    	ClastStart(lastTrack.start().m_x, lastTrack.start().m_y, lastTrack.angle()),
    	ClastEnd(lastTrack.end().m_x, lastTrack.end().m_y, lastTrack.angle(true));

    double d_fs = dubinsPathLength(C, CfirstStart, turnRadius),
    	d_fe = dubinsPathLength(C, CfirstEnd, turnRadius),
    	d_ls = dubinsPathLength(C, ClastStart, turnRadius),
    	d_le = dubinsPathLength(C, ClastEnd, turnRadius);
    double d_min = d_fs;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Comparing distances: " << std::endl
    	<< "First track: " << d_fs << " and " << d_fe << std::endl
    	<< "Last track: " << d_ls << " and " << d_le << std::endl;

    if (d_fe < d_min) {
    	d_min = d_fe;
    	reverseStartTrack = true;
    	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Chose first track end: " << firstTrack.end() << std::endl;
    }
    else if (d_ls < d_min) {
    	d_min = d_ls;
    	reverseStartTrack = false;
    	startAtLast = true;
    	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Chose last track start: " << lastTrack.start() << std::endl;
    }
    else if (d_le < d_min) {
    	d_min = d_le;
    	reverseStartTrack = true;
    	startAtLast = true;
    	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Chose last track end: " << lastTrack.end() << std::endl;
    }
    else {
    	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Chose first track start: " << firstTrack.start() << std::endl;
    }

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Decomposing " << tracks.size()
    	<< " field tracks into nodes with coverage width:" << field.coverageWidth()
    	<< std::endl;

    setFieldTrackOrder(tracks, startAtLast, reverseStartTrack);
    int n = tracks.addNodesToGraph(G, GA, field.coverageWidth(), Tour, Headings);

	Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Added " << n
		<< " nodes to cover entire field." << std::endl;

	// Create edges
    cost = createDubinsTourEdges(G, GA, Tour, Headings, turnRadius, Edges, returnToInitial);

    return SUCCESS;
}

} // namespace dpp