/*
 * Given a graph of vertices, this program generates random headings four each vertex,
 * and then solves the Asymmetric Traveling Salesperson Problem (ATSP) for a solution to
 * the Dubins Traveling Salesperson Problem (DTSP).
 *
 * Derived from the Randomized Algorithm (Le Ny et al. 2005).
 *
 * Copyright (C) 2014-2015 DubinsAreaCoverage.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the COPYRIGHT file distributed with DubinsAreaCoverage.
*/
#include <math.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <stdbool.h>

#include <random>
#include <cmath>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>

#include <LKH.h>
#include <cxxopts.h>

#include "Log.h"
#include "Dubins.h"
#include "Configuration.h"
#include "Util.h"
#include "TSPLib.h"

//#define DEBUG

// Enable stack traces in debug mode
#ifdef DEBUG
#include "stacktrace.h"
#endif

using namespace std;

// Have to import specifially, since Configuration clashes
using ogdf::node;
using ogdf::edge;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::GraphCopy;
using ogdf::DPoint;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::NodeArray;

#define TWO_PI    ((double)2.0 * M_PI)

/**
 * Generates a random heading in radians from [0, 2PI) with the uniform distribution.
 */
double randomHeading(void) {
    // Random number generator initilization
    static random_device rd;
    static default_random_engine e1(rd());
    static uniform_real_distribution<double> uniform_dist(0, TWO_PI);

    // Generate the uniform random number
    float x = fmod(uniform_dist(e1), TWO_PI);
    return x;
}

/**
 * Randomizes all headings for nodes in the graph. Skips the first node (origin)
 * by default.
 * @param G         A graph with nodes.
 * @param GA        Attributes for G.
 * @param Headings  
 */
void randomizeHeadings(Graph &G, GraphAttributes &GA, NodeArray<double> &Headings,
    bool skipOrigin=true) {
    ListIterator<node> iter;
    #ifdef DEBUG
    cout << "Randomizing headings: " << endl;
    #endif
    node i;
    forall_nodes(i, G) {
        if (skipOrigin && i == G.firstNode()) continue; // skip the origin
        double x = randomHeading();
        Headings[i] = x;

        #ifdef DEBUG
        cout << "   Node " << GA.idNode(u) << ": " << x << endl;
        #endif
    }
}

/**
 * Solves the DTSP with the randomized algorithm. The tour, headings, and total
 * cost are saved into their respective parameters.
 * @param G         a graph of the problem
 * @param GA        attributes of graph
 * @param x         a starting heading in radians [0,2*pi)
 * @param r         a turning radius in radians
 * @param tour      a list of nodes to hold the result
 * @param edges     a list of edges to hold the result
 * @param Headings  a node array of headings to hold the result
 * @param cost      holds the total cost result
 * @return An exit code (0==SUCCESS)
 */
int solveRandomizedDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &tour, List<edge> &edges, NodeArray<double> &Headings, double &cost) {

    if (x < 0.0 || x >= M_PI*2.0) {
        cerr << "Expected x to be between 0 and 2*PI." << endl;
        return 1;
    }

    if (Headings.graphOf() != &G) {
        cerr << "Headings should be for G." << endl;
        return 1;
    }

    if (G.numberOfNodes() < 2) {
        cerr << "Expected G to have atleast 2 nodes." << endl;
        return 1;
    }

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();

    FILE_LOG(logDEBUG) << "Found " << n << " nodes, and " << m << " edges.";

    // Weighted adjacency matrix from random headings
    // TODO add loop for multiple runs
    randomizeHeadings(G, GA, Headings);
    NodeMatrix<double> A(G);
    buildDubinsAdjacencyMatrix(G, GA, A, Headings, r);

    // Generate temporary TSP and PAR files
    string problemComment("Asymmetric TSP problem with ");
    problemComment += to_string(n) + " nodes.";
    string problemName("prDubinsScenario");

    string parFilename = (std::tmpnam(nullptr) + string(PAR_FILE_EXTENSION)),
        tspFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION)),
        tourFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION));
    if (writePARFile(parFilename,tspFilename, tourFilename) != SUCCESS
        || writeATSPFile(tspFilename, problemName, problemComment, G, A) != SUCCESS) {
        cerr << "Failed creating TSP files." << endl;
        return 1;
    }

    FILE_LOG(logDEBUG) << "Wrote " << parFilename << " and " << tspFilename << ".";
    FILE_LOG(logDEBUG) << "Running LKH solver for Asymmetric TSP.";

    // Find ATSP solution with LKH. Saves into tourFilename
    Timer *t1 = new Timer();
    try { 
        LKH::runSolver(const_cast<char*>(parFilename.c_str()));
    } catch(const std::exception& e) { 
        cerr << "LKH solver failed with an exception: " << endl << e.what() << endl;
        FILE_LOG(logDEBUG) << "LKH exception: " << e.what();
        return 1;
    }
    float elapsedTime = t1->diffMs();

    FILE_LOG(logDEBUG) << "Finished (" <<  elapsedTime << "ms)."
        << " Optimal euclidean tour in " << tourFilename << ".";
    cout << "Computed Euclidean TSP solution in " <<  elapsedTime << "ms."
        << endl << "Tour saved in " << tourFilename << "." << endl;

    // Read LKH solution from tour file
    if (readTSPTourFile(tourFilename, G, GA, tour) != SUCCESS) {
        return 1;
    }

    // Create edges
    cost = createDubinsTourEdges(G, GA, tour, Headings, r, edges, true); // TODO add return cost parameter  to main
    
    // Print headings
    #ifdef DEBUG
    node u;
    cout << "Solved " << n << " point tour with cost " << cost << "." << endl;
    cout << "Headings: " << endl;
    forall_nodes(u,G) {
        cout << "   Node " << GA.idNode(u) << ": " << Headings[u] << " rad." << endl;
    }
    #endif

    // Cleanup
    if (std::remove(tspFilename.c_str()) != 0 
        && std::remove(parFilename.c_str()) != 0
        && std::remove(tourFilename.c_str()) != 0) {
        cerr << "Failed to delete temporary files " << tspFilename << ", " << parFilename << "." << endl;
        FILE_LOG(logDEBUG) << "Failed deleting temporary files " << tspFilename << ", "
            << parFilename << ", " << tourFilename << ".";
        return 1;
    }
    FILE_LOG(logDEBUG) << "Removed temporary files " << tspFilename << ", "
        << parFilename << ", " << tourFilename << ".";
    return 0;
}

/**
 * Constructor that doesn't take a tour argument.
 */
int solveRandomizedDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<edge> &edges, NodeArray<double> &Headings, double &cost) {

    List<node> tour;
    return solveRandomizedDTSP(G, GA, x, r, tour, edges, Headings, cost);
}

/**
 * Constructor that doesn't take an edge list argument.
 */
int solveRandomizedDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &tour, NodeArray<double> &Headings, double &cost) {

    List<edge> edges;
    return solveRandomizedDTSP(G, GA, x, r, tour, edges, Headings, cost);
}


/** Main Entry Point
 * The solution is a tour, which is an ordered list of waypoints,
 * saved inside as a TSPlib file. The total cost is printed.
 * 
 * usage: randomizedDTSP <inputGMLFile> <startHeading> <turnRadius>
 *
 * @param inputGMLFile  input GML file to read the problem from
 * @param startHeading  a starting heading in radians [0,2*pi)
 * @param turnRadius    a turning radius in radians
 * @return An exit code (0==SUCCESS)
 */
#if !defined(DUBINS_IS_LIBRARY)
int main(int argc, char *argv[]) {
    // Setup stack traces for debugging
    char const *program_name = argv[0];
    #ifdef DEBUG
    set_signal_handler(program_name);
    #endif

    // Initialize logging
    FILELog::ReportingLevel() = logDEBUG3;
    FILE* log_fd = fopen( "logfile.txt", "w" );
    Output2FILE::Stream() = log_fd;
    FILE_LOG(logDEBUG) << "Started.";

    // Input arguments
    string inputFilename;
    double x, r;
    bool debug = false;

    // Option parsing
    cxxopts::Options options(program_name,
        " gml_file initial_heading turn_radius");
    try {
        options.add_options()
            ("d,debug", "Enable debugging messages", cxxopts::value<bool>(debug))
            ("h,help", "Print this message");
            //("", "x is the initial heading")
            //("", "r is the turn radius of the Dubins vehicle");
            //("inputGMLFile", "Input GML file to read graph from", cxxopts::value<string>(), "INPUT_GML_FILE")
            //("initialHeading", "Initial heading orientation", cxxopts::value<string>(), "INITIAL_HEADING")
            //("turnRadius", "Dubins vehicle turning radius", cxxopts::value<string>(), "TURN_RADIUS");

        //options.parse_positional({"inputGMLFile"});
        options.parse(argc, argv);

        if (options.count("help")) {
            std::cout << options.help() << std::endl;
            return 0;
        }

        if (argc != 4) {
            std::cout << program_name << ": " << " Expected 3 arguments." << std::endl;
            std::cout << options.help();
            exit(1);
        }

    } catch (const cxxopts::OptionException& e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }

    inputFilename = argv[1];
    x = atof(argv[2]);
    r = atof(argv[3]);
   
    // Load the graph (from GML)
    Graph G;
    GraphAttributes GA(G,
      GraphAttributes::nodeGraphics |
      GraphAttributes::edgeGraphics |
      GraphAttributes::nodeLabel |
      GraphAttributes::edgeStyle |
      GraphAttributes::edgeDoubleWeight |
      GraphAttributes::nodeStyle |
      GraphAttributes::nodeTemplate |
      GraphAttributes::nodeId); 

     if (!ogdf::GraphIO::readGML(GA, G, inputFilename)) {
        cerr << "Could not open " << inputFilename << endl;
        return 1;
    }
    FILE_LOG(logDEBUG) << "Opened " << inputFilename << "." << endl;

    List<node> tour;
    List<edge> edges;
    double cost;
    NodeArray<double> Headings(G);

    if (solveRandomizedDTSP(G, GA, x, r, tour, edges, Headings, cost) != SUCCESS) {
        cerr << "Randomized algorithm failed" << endl;
        return 1;
    }

    // Results
    cout << "Solved " << G.numberOfNodes() << " point tour with cost " << cost << "." << endl;

    // Print edge list
    printEdges(G, GA, edges);

    return 0;

}
#endif
