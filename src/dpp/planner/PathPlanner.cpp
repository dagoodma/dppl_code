/*
 * Implementation of DPP::PathPlanner abstract class.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <dpp/planner/PathPlanner.h>

namespace dpp {

PathPlanner::~PathPlanner() {
}

/**
 * Copy the solution (Graph, Tour, Edges, and Headings) into those given.
 * @note Existing nodes and edges are cleared.
 */
void DubinsPathPlanner::copySolution(ogdf::Graph &G, ogdf::GraphAttributes &GA,
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

DubinsPathPlanner::~DubinsPathPlanner() {
}

} // namespace dpp 