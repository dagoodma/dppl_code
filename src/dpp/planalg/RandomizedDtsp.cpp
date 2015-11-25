/*
 * Implementation of DPP::RandomizedDtsp class for solving Dtsp problems
 * with the randomized heading algorithm.
 *
 * Derived from the Randomized Algorithm (Le Ny et al. 2005 and 2011).
 * 
 * TODO: Implement discretization levels from 2011 paper.
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
#include <dpp/planalg/RandomizedDtsp.h>
#include <dpp/basic/FileIO.h>

#define ALGORITHM_ITERATIONS            10 // number of iterations for best-of

#define TWO_PI    ((double)2.0 * M_PI)

namespace dpp {

/**
 * Generates a random heading in radians from [0, 2PI) with the uniform distribution.
 * @return uniform random number between [0, 2PI)
 */
double randomHeading(void) {
    // Random number generator initilization
    static std::random_device rd;
    static std::default_random_engine e1(rd());
    static std::uniform_real_distribution<double> uniform_dist(0, TWO_PI);

    // Generate the uniform random number
    float x = fmod(uniform_dist(e1), TWO_PI);
    DPP_ASSERT(0.0 <= x && x < TWO_PI);

    return x;
}

/**
 * Randomizes all headings for nodes in the graph. Skips the first node (origin)
 * by default.
 * @param G         A graph with nodes.
 * @param GA        Attributes for G.
 * @param Headings  a node array of headings to hold the result
 */
void randomizeHeadings(Graph &G, GraphAttributes &GA, NodeArray<double> &Headings,
    bool skipOrigin=true) {
    ListIterator<node> iter;
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Randomizing headings: " << std::endl;
    node i;
    forall_nodes(i, G) {
        if (skipOrigin && i == G.firstNode()) continue; // skip the origin
        double x = randomHeading();
        Headings[i] = x;

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "   Node " << GA.idNode(i) << ": "
            << x << " rad" << std::endl;
    }
}

/**
 * Solves the Dtsp with the randomized algorithm. The tour, headings, and total
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
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
#pragma GCC diagnostic push 
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
int RandomizedDtsp::run(Graph &G, GraphAttributes &GA, double x, double r,
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

    // Generate temporary TSP and PAR files
    string problemComment("Asymmetric TSP problem with ");
    problemComment += to_string(n) + " nodes.";
    string problemName("prDubinsScenario");

    string parFilename = (std::tmpnam(nullptr) + string(PAR_FILE_EXTENSION)),
        tspFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION)),
        tourFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION));

    // Find the best configuration over many iterations
    List<node> BestTour;
    NodeArray<double> BestHeadings;
    double bestCost = -1;
    Timer *tTotal = new Timer();
    for (int i = 0; i < ALGORITHM_ITERATIONS; i++) {
        // Generate weighted adjacency matrix from random headings
        randomizeHeadings(G, GA, Headings);
        NodeMatrix<double> A(G);
        buildDubinsAdjacencyMatrix(G, GA, A, Headings, r);


        if (writeParFile(parFilename,tspFilename, tourFilename) != SUCCESS
            || writeAtspFile(tspFilename, problemName, problemComment, G, A) != SUCCESS) {
            throw std::runtime_error("Failed creating TSP files.");
        }

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Wrote " << parFilename << " and "
            << tspFilename << "." << std::endl;
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Running LKH solver for Asymmetric TSP."
            << std::endl;

        // Find ATSP solution with LKH. Saves into tourFilename
        Timer *t1 = new Timer();
        //try {
            runLKHSolver(parFilename);
        //} catch(const std::exception& e) { 
        //    cerr << e.what() << endl;
        //    FILE_LOG(logDEBUG) << e.what();
        //    return FAILURE;
        //}
        float elapsedTime = t1->diffMs();

        Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Finished (" <<  elapsedTime << "ms)."
            << " ATSP tour " << i << " in " << tourFilename << "." << std::endl;

        // Read LKH solution from tour file
        if (readTspTourFile(tourFilename, G, GA, Tour) != SUCCESS) {
            std::runtime_error("Could not read solution from LKH tour file!");
        }

        double cost_i = dubinsTourCost(G, GA, Tour, Headings, r, returnToInitial);

        // Save the best scenario
        if (cost_i < bestCost || bestCost < 0) {
            bestCost = cost_i;
            BestHeadings = Headings;
            BestTour = Tour;
        }
        Tour.clear();
            
    } // for (int i = 0; i < ALGORITHM_ITERATIONS; i++) 

    float elapsedTimeTotal = tTotal->diffMs();

    // Use the best scenario
    cost = bestCost;
    Headings = BestHeadings;
    Tour = BestTour;

    // Remove return node in tour if necessary
    if (!returnToInitial && Tour.size() > G.numberOfNodes()) {
        Tour.popBack();
    }

    // Create edges
    cost = createDubinsTourEdges(G, GA, Tour, Headings, r, Edges, returnToInitial);

    // Debug print info
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
} // RandomizedDtsp::run()
#pragma clang diagnostic pop
#pragma GCC diagnostic pop

} // namespace dpp