/*
 * Path planner for a Dubins vehicle. Considers the 2D turning radius of the vehicle.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <dpp/basic/FileIO.h>

#include <dpp/planner/DubinsVehiclePathPlanner.h>

namespace dpp {

/* 
 * Load graph from GML files.
 */
void DubinsVehiclePathPlanner::addWaypoints(std::string gmlFilename) {
    DPP_ASSERT(readGraphFromGmlFile(gmlFilename, m_G, m_GA) == SUCCESS);
    m_Headings.init(m_G);
    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Read " << gmlFilename << " with " << waypointCount()
        << " nodes." << std::endl;
}

/*
 * Copy the given Graph and attributes to add waypoints.
 * @note Existing waypoints are deleted.
 */
void DubinsVehiclePathPlanner::addWaypoints(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    DPP_ASSERT(G.numberOfEdges() == 0); // TODO just clear edges
    m_G.clear();
    // Do we need to worry about resetting attributes too?
    //m_GA.init(m_G, m_GA.attributes());

    // Copy graph and attributes
    int n = copyGraph(G, GA, m_G, m_GA);
    m_Headings.init(m_G);

    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Copied " << n << " nodes from graph " << &G << " with " << waypointCount()
        << " nodes into graph " << &m_G << std::endl;
}

/*
 * Run the algorithm to solve the planning problem.
 */
bool DubinsVehiclePathPlanner::solve(void) {
    DPP_ASSERT(waypointCount() > 1);
    m_haveSolution = false;

    // Set the initial heading
    m_Headings(m_G.firstNode()) = m_initialHeading;

    Logger::logInfo(DPP_LOGGER_VERBOSE_1) << "Solving " << waypointCount() << " node problem with "
        << m_algorithm->name() << " " << m_algorithm->typeText() << " algorithm." << std::endl;
    try {
        AlgorithmDtsp *alg = dynamic_cast<AlgorithmDtsp*>(m_algorithm.get());
       
        if (alg->run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour,
            m_Edges, m_Headings, m_cost, m_returnToInitial) == SUCCESS) {
            m_haveSolution = true;
        }
    } catch(std::exception &e) {
        Logger::logError() << "An exception occured: " << e.what() << std::endl;
        m_haveSolution = false;
    }

    return m_haveSolution;
}

/*
 * Sets the DTSP algorithm for solving.
 */
void DubinsVehiclePathPlanner::algorithm(DtspPlanningAlgorithm algId) {
    switch (algId) {
        case NEAREST_NEIGHBOR:
            m_algorithm.reset(new NearestNeighborDtsp());
            Logger::logDebug() << "Set DTSP algorithm to NEAREST_NEIGHBOR." << std::endl;
            break;
        case ALTERNATING:
            m_algorithm.reset(new AlternatingDtsp());
            Logger::logDebug() << "Set DTSP algorithm to ALTERNATING." << std::endl;
            break;
        case RANDOMIZED:
            m_algorithm.reset(new RandomizedDtsp());
            Logger::logDebug() << "Set DTSP algorithm to RANDOMIZED." << std::endl;
            break;
        default:
            DPP_ASSERT(0 && "Unknown planning algorithm");
            Logger::logError() << "Unknown planning algorithm: " << algId << std::endl;
            return;
    } // switch (algId)
}


} // namespace dpp 