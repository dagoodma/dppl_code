/*
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

/**
 * Applies the alternating algorithm to the given tour by finding heading orientations
 * given the Dubins vehicle turning radius r. Modifies heading.
 */
void alternatingAlgorithm(Graph &G, GraphAttributes &GA, List<node> &tour,
    NodeArray<double> &headings, double r) {
    ListIterator<node> iter;
    int i = 1;
    #ifdef DEBUG
    cout << "Tour: " << endl;
    #endif
    // We don't want to iterate over the last point in the tour, since we're
    // looking at each node's successor. Hence: i < tour.size()
    for ( iter = tour.begin(); (i < tour.size() && iter != tour.end()); iter++ ) {
        node u = *iter, v = *(iter.succ());

        #ifdef DEBUG
        cout << "   Node " << GA.idNode(u) << endl;
        #endif

        // If odd
        if (fmod(i,2) != 0) {
            Vector2d uv(GA.x(u), GA.y(u));
            Vector2d vv(GA.x(v), GA.y(v));
            headings[u] = headingBetween(uv, vv);
        }
        // If even
        else {
            v = *(iter.pred());
            headings[u] = headings[v];
        }
        i++;
    }
}

/**
 * Solves the DTSP with the alternating algorithm. The tour, headings, and total
 * cost are saved into their respective parameters.
 * @param G         a graph of the problem
 * @param GA        attributes of graph
 * @param x         a starting heading in radians [0,2*pi)
 * @param r         a turning radius in radians
 * @param tour      save into a list of nodes representing the tour
 * @param edges     save into a list of edges representing the tour
 * @param headings  save into a NodeArray of headings
 * @param cost      save the total cost
 * @return An exit code (0==SUCCESS)
 */
int solveAlternatingDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &tour, List<edge> &edges, NodeArray<double> &headings, double &cost) {

    if (x < 0.0 || x >= M_PI*2.0) {
        cerr << "Expected x to be between 0 and 2*PI." << endl;
        return 1;
    }

    if (headings.graphOf() != &G) {
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

    // Generate temporary TSP and PAR files
    string problemComment("Euclidean TSP problem with ");
    problemComment += to_string(n) + " nodes.";
    string problemName("prDubinsScenario");

    string parFilename = (std::tmpnam(nullptr) + string(PAR_FILE_EXTENSION)),
        tspFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION)),
        tourFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION));
    if (writePARFile(parFilename,tspFilename, tourFilename) != SUCCESS
        || writeETSPFile(tspFilename, problemName, problemComment, G, GA) != SUCCESS) {
        cerr << "Failed creating TSP files." << endl;
        return 1;
    }

    FILE_LOG(logDEBUG) << "Wrote " << parFilename << " and " << tspFilename << ".";
    FILE_LOG(logDEBUG) << "Running LKH solver for Euclidean TSP.";

    // Find Euclidean TSP solution with LKH. Saves into tourFilename
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

    // Apply alternating algorithm
    alternatingAlgorithm(G, GA, tour, headings, r);
    cost = createDubinsTourEdges(G, GA, tour, headings, r, edges, true); // TODO add return cost input
   
    // Print headings
    #ifdef DEBUG
    node u;
    cout << "Solved " << n << " point tour with cost " << cost << "." << endl;
    cout << "Headings: " << endl;
    forall_nodes(u,G) {
        cout << "   Node " << GA.idNode(u) << ": " << headings[u] << " rad." << endl;
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

int solveAlternatingDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<edge> &edges, NodeArray<double> &headings, double &cost) {

    List<node> tour;
    return solveAlternatingDTSP(G, GA, x, r, tour, edges, headings, cost);
}

int solveAlternatingDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &tour, NodeArray<double> &headings, double &cost) {

    List<edge> edges;
    return solveAlternatingDTSP(G, GA, x, r, tour, edges, headings, cost);
}


/** Main Entry Point
 * Given the graph in the input GML file, this program solves the Euclidean Traveling
 * Salesperson Problem (ETSP), and then applies the alternating algorithm to solve the 
 * Dubins Traveling Salesperson Problem (DTSP). The solution is output as a tour inside
 * a TSPlib file. The total cost is printed at tht end.
 * 
 * usage: alternatingDTSP <inputGMLFile> <startHeading> <turnRadius>
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
    //string inputFilename(argv[1]);
    //double x = atof(argv[2]);
    //double r = atof(argv[3]);
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
    NodeArray<double> headings(G);

    if (solveAlternatingDTSP(G, GA, x, r, tour, edges, headings, cost) != SUCCESS) {
        cerr << "Alternating algorithm failed" << endl;
        return 1;
    }

    // Results
    cout << "Solved " << G.numberOfNodes() << " point tour with cost " << cost << "." << endl;

    // Print edge list
    printEdges(G, GA, edges);

    return 0;

}
#endif
