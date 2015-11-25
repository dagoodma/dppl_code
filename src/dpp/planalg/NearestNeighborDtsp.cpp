/*
 * Implementation of DPP::NearestNeighbor class for solving DTSP problems
 * with the nearest neighbor algorithm.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <cmath>

#include <dpp/basic/basic.h>
#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Path.h>
#include <dpp/planalg/NearestNeighborDtsp.h>

using ogdf::GraphCopy;

namespace dpp {
/**
 * Find the copied node vC nearest to C using the shortest Dubins path distance metric.
 */
double findNearestNode(GraphCopy &GC, GraphAttributes &GA, VehicleConfiguration &C,
    node &vC, double r) {
    double minDist = -1.0f;
    node iC;
    forall_nodes(iC,GC) {
        node i = GC.original(iC);

        VehicleConfiguration Ci(GA.x(i), GA.y(i));
        //double dist = C.euclideanDistance(Ci);
        Ci.heading(headingBetween(C.asVector(), Ci.asVector()) );
        double dist = dubinsPathLength(C, Ci, r);
        if (minDist < 0.0f || dist < minDist) {
            vC = iC;
            minDist = dist;
        }
    }

    return minDist;
}

/**
 * Solves the Dtsp with the nearest neighbor algorithm. The tour, headings, and total
 * cost are saved into their respective parameters.
 * @param G         a graph of the problem
 * @param GA        attributes of graph
 * @param x         a starting heading in radians [0,2*pi)
 * @param r         a turning radius in radians
 * @param Tour      a list of nodes to hold the result
 * @param Edges     a list of edges to hold the result
 * @param Headings  a node array of headings to hold the result
 * @param cost      holds the total cost result
 * @return An exit code (0==SUCCESS)
 */
int NearestNeighborDtsp::run(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &Tour, List<edge> &Edges, NodeArray<double> &Headings, double &cost,
    bool returnToInitial) {
    // Check arguments
    if (x < 0.0 || x >= M_PI*2.0) {
        throw std::out_of_range("Expected x to be between 0 and 2*PI");
    }

    if (Headings.graphOf() != &G) {
        throw std::domain_error("Headings should be for G");
    }

    if (G.numberOfNodes() < 2) {
        throw std::out_of_range("Expected G to have at least 2 nodes");
    }
    if (G.numberOfEdges() > 0) {
        throw std::out_of_range("Expected G to have 0 edges");
    }

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found " << n << " nodes, and " << m << " edges." << std::endl;

    // Make a graph copy and remove nodes until none are left
    double total_cost = 0.0f;
    GraphCopy GC(G); // unvisited nodes
    node nodeStart = G.firstNode();

    // Start at the first node
    node u = nodeStart;
    VehicleConfiguration Cs(GA.x(nodeStart), GA.y(nodeStart), x);
    VehicleConfiguration C(Cs);

    // Remove the first node (add return edge afterwards)
    GC.delNode(GC.copy(nodeStart));
    Tour.pushBack(nodeStart);
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << printGraph(G,GA) << std::endl;

    Logger::logDebug(DPP_LOGGER_VERBOSE_1)
        << "Running nearest neighbor solver for Greedy TSP." << std::endl;
    Timer *t1 = new Timer();
    while (!GC.empty()) {
        // Find the closest node, and remove it from the graph
        node vC;
        double cost = findNearestNode(GC, GA, C, vC, r);
        total_cost += cost;
        node v = GC.original(vC);
        GC.delNode(vC);

        // Create an edge, and add it to the tour
        edge e = G.newEdge(u, v);
        GA.doubleWeight(e) = cost;
        Edges.pushBack(e);
        Tour.pushBack(v);

        // Use the heading of line segment between two nodes
        Vector2d uv = C.asVector();
        Vector2d vv(GA.x(v), GA.y(v));
        double x_v = headingBetween(uv,vv);
        Headings(v) = x_v;
        u = v;

        // Update our current position
        C.position(GA.x(v), GA.y(v));
        C.heading(x_v);
    }

    // Return to start configuration
    if (returnToInitial) {
        double cost_return = dubinsPathLength(C, Cs, r);
        cost = total_cost + cost_return;
        edge e = G.newEdge(u, nodeStart);
        GA.doubleWeight(e) = cost_return;
        Edges.pushBack(e);
        Tour.pushBack(nodeStart);
    }

    // Create edges
    cost = dubinsTourCost(G, GA, Tour, Headings, r, returnToInitial);

    // Debug print info
    float elapsedTime = t1->diffMs();
    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Solved " << G.numberOfNodes()
        << " point tour with cost " << cost << " (" << elapsedTime << " ms)." << std::endl;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << dpp::printHeadings(G, GA, Headings);
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << dpp::printTour(G, GA, Tour);
    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << dpp::printEdges(G, GA, Edges);

    return SUCCESS;
} // NearestNeighborDtsp::run()

} // namespace dpp