/*
 * Path planner for a Dubins vehicle. Considers the 2D turning radius of the vehicle.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <ogdf/fileformats/GraphIO.h>

#include <dpp/basic/basic.h>
#include <dpp/planner/PathPlanner.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

namespace dpp {

/* 
 * Load graph from GML files.
 */
void DubinsVehiclePathPlanner::addWaypoints(std::string gmlFilename) {
    Logger::logDebug() << "Before mem=" << (&m_G);
    DPP_ASSERT(ogdf::GraphIO::readGML(m_GA, m_G, gmlFilename));
    m_Headings.init(m_G);
    Logger::logDebug() << ", after mem=" << (&m_G) << std::endl;
    Logger::logDebug() << "Read " << gmlFilename << " with " << waypointCount()
        << " nodes." << std::endl;
}

/*
 * Copy the graph.
 */
void DubinsVehiclePathPlanner::addWaypoints(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    m_G = G;
    m_GA = GA;
    m_Headings.init(m_G);
    Logger::logDebug() << "Copied graph with " << waypointCount() << " nodes." << std::endl;
}

/*
 * Run the algorithm to solve the planning problem.
 */
bool DubinsVehiclePathPlanner::solve(void) {
    DPP_ASSERT(waypointCount() > 1);
    m_haveSolution = false;

    Logger::logInfo() << "Solving " << waypointCount() << " node problem with "
        << m_algorithm->name() << " " << m_algorithm->typeText() << " algorithm." << std::endl;
    try {
        AlgorithmDTSP *alg = dynamic_cast<AlgorithmDTSP*>(m_algorithm.get());
        // TODO add checking
        alg->run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour,
            m_Edges, m_Headings, m_cost, m_returnToInitial);
        m_haveSolution = true;
    } catch(std::exception &e) {
        Logger::logError() << "An exception occured: " << e.what() << std::endl;
        m_haveSolution = false;
    }

    return m_haveSolution;
}

/**
 * Copy the solution results into the given objects.
 */
void DubinsVehiclePathPlanner::copySolution(ogdf::Graph &G, ogdf::GraphAttributes &GA,
        ogdf::List<ogdf::node> &Tour, ogdf::List<ogdf::edge> &Edges,
        NodeArray<double> &Headings, double &cost) {
    DPP_ASSERT(m_haveSolution);

    G = m_G;
    GA = m_GA;
    Tour = m_Tour;
    Edges = m_Edges;
    Headings = m_Headings;
    cost = m_cost;
}

/*
 * Sets the DTSP algorithm for solving.
 */
void DubinsVehiclePathPlanner::algorithm(PlanningAlgorithm algId) {
    switch (algId) {
        /*
        case NEAREST_NEIGHBOR:
            m_algorithm = new NearestNeighborDTSP();
            Logger::logDebug() << "Set DTSP algorithm to NEAREST_NEIGHBOR." << std::endl;
            break;
        case ALTERNATING:
            m_algorithm = new AlternatingDTSP();
            Logger::logDebug() << "Set DTSP algorithm to ALTERNATING." << std::endl;
            break;
        */
        case RANDOMIZED:
            m_algorithm.reset(new RandomizedDTSP());
            Logger::logDebug() << "Set DTSP algorithm to RANDOMIZED." << std::endl;
            break;
        default:
            Logger::logError() << "Unknown planning algorithm: " << algId << std::endl;
            return;
    } // switch (algId)
}


} // namespace dpp 