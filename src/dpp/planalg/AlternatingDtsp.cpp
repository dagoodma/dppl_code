/*
 * Implementation of dpp::AlternatingDtsp class for solving Dtsp problems
 * with the alternating algorithm.
 *
 * Derived from the Alternating Algorithm (Savla et al. 2005).
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <random>
#include <cmath>

#include <ogdf/fileformats/GraphIO.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Path.h>
#include <dpp/planalg/AlternatingDtsp.h>
#include <dpp/basic/FileIO.h>
#include <dpp/basic/Util.h>

#define USE_INITIAL_HEADING // don't re-assign initial heading

namespace dpp {

/**
 * Applies the alternating algorithm to the given tour by finding heading orientations
 * given the Dubins vehicle turning radius r. Modifies heading.
 */
void alternatingAlgorithm(Graph &G, GraphAttributes &GA, List<node> &Tour,
    NodeArray<double> &Headings, double x, double r) {
    ListIterator<node> iter;
    int i = 1;
    node nodeStart = G.firstNode();

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Tour: " << std::endl;

    // We don't want to iterate over the last point in the tour, since we're
    // looking at each node's successor. Hence: i < tour.size()
    node u_last;
    for ( iter = Tour.begin(); (i < Tour.size() && iter != Tour.end()); iter++ ) {
        node u = *iter, v = *(iter.succ());

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "   Node " << GA.idNode(u)
            << " (" << GA.x(u) << ", " << GA.y(u) << "): ";

        #ifdef USE_INITIAL_HEADING
        // Use initial heading if we're at the starting node
        if (u == nodeStart) {
            Headings[u] = x;
        }
        else {
        #endif

        // If odd, find the heading to the next node
        if (fmod(i,2) != 0) {
            Vector2d uv(GA.x(u), GA.y(u));
            Vector2d vv(GA.x(v), GA.y(v));
            Headings[u] = headingBetween(uv, vv);
        }
        // If even,
        else {
            node w = *(iter.pred());
            #ifdef USE_INITIAL_HEADING
            if (u_last == nodeStart)
            {
                // Use heading from initial position to current
                Vector2d uv(GA.x(nodeStart), GA.y(nodeStart));
                Vector2d vv(GA.x(u), GA.y(u));
                Headings[u] = headingBetween(uv, vv);
            }
            else 
            #endif
            {
                // Use heading of previous node
                Headings[u] = Headings[u_last];
            }
        }

        #ifdef USE_INITIAL_HEADING
        }   
        #endif

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << Headings[u] << " rad" << std::endl;
        i++;
        u_last = u;
    }

    // Set heading of the last node in the tour to it's predecessor's heading
    // if it's not the first node.
    if (Tour.back() != Tour.front()) {
        iter = Tour.rbegin();
        node u = *iter;
        iter--; // reverse iterator decrements
        node v = *iter;
        Headings[u] = Headings[v];
    }
}

/**
 * Solves the Dtsp with the alternating algorithm. The tour, headings, and total
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
int AlternatingDtsp::run(Graph &G, GraphAttributes &GA, double x, double r,
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

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found " << n << " nodes, and " << m << " edges." << std::endl;

    Logger::logDebug(DPP_LOGGER_VERBOSE_3) << printGraph(G, GA) << std::endl;

    // Generate temporary TSP and PAR files
    Headings(G.firstNode()) = x;
    string problemComment("Euclidean TSP problem with ");
    problemComment += to_string(n) + " nodes.";
    string problemName("prDubinsScenario");

    string parFilename = (std::tmpnam(nullptr) + string(PAR_FILE_EXTENSION)),
        tspFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION)),
        tourFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION));

    // Find the best configuration over many iterations
    Timer *t1 = new Timer();

    if (writeParFile(parFilename,tspFilename, tourFilename) != SUCCESS
        || writeEtspFile(tspFilename, problemName, problemComment, G, GA) != SUCCESS) {
        throw std::runtime_error("Failed creating TSP files.");
    }

    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Wrote " << parFilename << " and "
        << tspFilename << "." << std::endl;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Running LKH solver for Euclidean TSP."
        << std::endl;

    runLKHSolver(parFilename);
     
    float elapsedTime = t1->diffMs();

    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Finished (" <<  elapsedTime << "ms)."
        << " ETSP tour in " << tourFilename << "." << std::endl;

    // Read LKH solution from tour file
    if (readTspTourFile(tourFilename, G, GA, Tour) != SUCCESS) {
        std::runtime_error("Could not read solution from LKH tour file!");
    }

    // Remove return node in tour if necessary
    if (!returnToInitial && Tour.size() > G.numberOfNodes()) {
        Tour.popBack();
    }

    // Set the headings by applying the alternating algorithm
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Applying the alternating algorithm."
        << std::endl;
    alternatingAlgorithm(G, GA, Tour, Headings, x, r);
    cost = createDubinsTourEdges(G, GA, Tour, Headings, r, Edges, returnToInitial);

    // Debug print info
    float elapsedTimeTotal = t1->diffMs();
    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Solved " << G.numberOfNodes()
        << " point tour with cost " << cost << " (" << elapsedTimeTotal<< " ms)." << std::endl;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << dpp::printHeadings(G, GA, Headings);
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << dpp::printTour(G, GA, Tour);
    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << dpp::printEdges(G, GA, Edges);

    // Cleanup
    if (std::remove(tspFilename.c_str()) != 0 
        || std::remove(parFilename.c_str()) != 0
        || std::remove(tourFilename.c_str()) != 0) {
        throw std::runtime_error("Failed to delete temporary files.");
    }

    return SUCCESS;
} // AlternatingDtsp::run()

} // namespace dpp