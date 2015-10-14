/*
 * Path planner for a Dubins vehicle. Considers the 2D turning radius of the vehicle.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <ogdf/fileformats/GraphIO.h>
#include <ogdf/basic/EdgeArray.h>

using ogdf::EdgeArray;

#include <dpp/basic/basic.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/PathPlanner.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

namespace dpp {

/* 
 * Load graph from GML files.
 */
void DubinsVehiclePathPlanner::addWaypoints(std::string gmlFilename) {
    DPP_ASSERT(ogdf::GraphIO::readGML(m_GA, m_G, gmlFilename));
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

    Logger::logInfo(DPP_LOGGER_VERBOSE_1) << "Solving " << waypointCount() << " node problem with "
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
 * Copy the solution (Graph, Tour, Edges, and Headings) into the ones given.
 * @note Existing nodes and edges are cleared.
 */
void DubinsVehiclePathPlanner::copySolution(ogdf::Graph &G, ogdf::GraphAttributes &GA,
        ogdf::List<ogdf::node> &Tour, ogdf::List<ogdf::edge> &Edges,
        NodeArray<double> &Headings, double &cost) {
    DPP_ASSERT(m_haveSolution);

    // Copy the graph and attributes
    NodeArray<node> nodeCopyTable(m_G);
    EdgeArray<edge> edgeCopyTable(m_G);
    int n = copyGraph(m_G, m_GA, G, GA, nodeCopyTable, edgeCopyTable);

    // Copy the tour
    ogdf::ListIterator<ogdf::node> tourIter;
    for ( tourIter = m_Tour.begin(); tourIter != m_Tour.end(); tourIter++ ) {
        node u = *tourIter;
        node ucopy = nodeCopyTable(u);
        Tour.pushBack(ucopy);
    }

    // Copy the edge list
    ogdf::ListIterator<ogdf::edge> edgeIter;
    for ( edgeIter = m_Edges.begin(); edgeIter != m_Edges.end(); edgeIter++ ) {
        edge e = *edgeIter;
        edge ecopy = edgeCopyTable(e);
        Edges.pushBack(ecopy);
    }

    // Copy the headings
    Headings.init(G);
    node u;
    forall_nodes(u,m_G) {
        node ucopy = nodeCopyTable(u);
        Headings(ucopy) = m_Headings(u);
    }

    // Copy the cost
    cost = m_cost;

    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Copied " << n << " nodes from graph " << &m_G << " with " << waypointCount()
        << " nodes into graph " << &G << std::endl;
}

/*
 * Sets the DTSP algorithm for solving.
 */
void DubinsVehiclePathPlanner::algorithm(PlanningAlgorithm algId) {
    switch (algId) {
        case NEAREST_NEIGHBOR:
            m_algorithm.reset(new NearestNeighborDTSP());
            Logger::logDebug() << "Set DTSP algorithm to NEAREST_NEIGHBOR." << std::endl;
            break;
        case ALTERNATING:
            m_algorithm.reset(new AlternatingDTSP());
            Logger::logDebug() << "Set DTSP algorithm to ALTERNATING." << std::endl;
            break;
        case RANDOMIZED:
            m_algorithm.reset(new RandomizedDTSP());
            Logger::logDebug() << "Set DTSP algorithm to RANDOMIZED." << std::endl;
            break;
        default:
            DPP_ASSERT(0 && "Unknown planning algorithm");
            Logger::logError() << "Unknown planning algorithm: " << algId << std::endl;
            return;
    } // switch (algId)
}


} // namespace dpp 