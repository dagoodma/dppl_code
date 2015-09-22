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

#include "Log.h"
#include "Dubins.h"
#include "Configuration.h"
#include "Util.h"
#include "TSPLib.h"

#define DEBUG

// Enable stack traces in debug mode
#ifdef DEBUG
#include "stacktrace.h"
#endif

using namespace std;

// Have to import specifially, since Configuration clashes
using ogdf::node;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::GraphCopy;
using ogdf::DPoint;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::NodeArray;

/**
 * Find the copied node vC nearest to C using the shortest Dubins path distance metric.
 */
double findNearestNode(GraphCopy &GC, GraphAttributes &GA, Configuration &C,
    node &vC, double r) {
    double minDist = -1.0f;
    node iC;
    forall_nodes(iC,GC) {
        node i = GC.original(iC);

        Configuration Ci(GA.x(i), GA.y(i));
        //double dist = C.euclideanDistance(Ci);
        Ci.setHeading(headingBetween(C.asVector(), Ci.asVector()) );
        double dist = dubinsPathLength(C, Ci, r);
        if (minDist < 0.0f || dist < minDist) {
            vC = iC;
            minDist = dist;
        }
    }

    return minDist;
}

/**
 * Solves the Euclidean Traveling Salesperson problem using the Nearest Neighbor
 * algorithm, and then applies the alternating algorithm to generate a tour.
 */
int solveNearestNeighborDTSP(Graph &G, GraphAttributes &GA, double x, double r,
    List<node> &tour, NodeArray<double> &heading, double &cost) {

    if (x < 0.0 || x >= M_PI*2.0) {
        cerr << "Expected x to be between 0 and 2*PI." << endl;
        return 1;
    }

    if (heading.graphOf() != &G) {
        cerr << "Heading should be for G." << endl;
        return 1;
    }

    if (G.numberOfNodes() < 2) {
        cerr << "Expected G to have atleast 2 nodes." << endl;
        return 1;
    }

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();
    FILE_LOG(logDEBUG) << "Found " << n << " nodes, and " << m << " edges." << endl;


    cost = 0.0f;
    GraphCopy GC(G); // unvisited nodes
    node nodeStart = G.firstNode();
    Configuration Cs(GA.x(nodeStart), GA.y(nodeStart), x);
    Configuration C(Cs);
    GC.delNode(GC.copy(nodeStart));
    tour.pushBack(nodeStart);
    heading(nodeStart) = x;

    #ifdef DEBUG
    printGraph(G, GA);
    #endif

    FILE_LOG(logDEBUG) << "Running nearest neighbor solver for Euclidean TSP.";
    Timer *t1 = new Timer();
    while (!GC.empty()) {
        node vC;
        cost += findNearestNode(GC, GA, C, vC, r);
        node v = GC.original(vC);
        GC.delNode(vC);

        tour.pushBack(v);
        Vector2d uv = C.asVector();
        Vector2d vv(GA.x(v), GA.y(v));
        double x_v = headingBetween(uv,vv);
        heading(v) = x_v;
        // Update configuration
        C.setPosition(GA.x(v), GA.y(v));
        C.setHeading(x_v);
    }
    float elapsedTime = t1->diffMs();

    // Return to start configuration
    // TODO make optional
    tour.pushBack(nodeStart);
    cost += dubinsPathLength(C, Cs, r);
    //cost += C.m_position.distance(Cs.m_position);

    FILE_LOG(logDEBUG) << "Finished solving with cost " << cost << ".";

    FILE_LOG(logDEBUG) << "Finished (" <<  elapsedTime << "ms).";
    cout << "Computed TSP solution in " <<  elapsedTime << "ms." << endl;
   
    // Print headings
    #ifdef DEBUG
    node u;
    cout << "Headings: " << endl;
    forall_nodes(u,G) {
        cout << "   Node " << GA.idNode(u) << ": " << heading[u] << " rad." << endl;
    }
    #endif

    return SUCCESS;
}


/** Main Entry Point
 * Given the graph in the input GML file, this program solves the Euclidean Traveling
 * Salesperson Problem (ETSP) using the sub-optimal nearest neighbor algorithm with
 * Dubins shortest path as the distance metric, and then applies the alternating
 * algorithm to solve the Dubins Traveling Salesperson Problem (DTSP). The solution
 * is output as a tour inside a TSPlib file. The total cost is printed at the end.
 * 
 * usage: nearestNeighborDTSP <inputGMLFile> <startHeading> <turnRadius>
 *
 * @param inputGMLFile  input GML file to read the problem from
 * @param startHeading  a starting heading in radians [0,2*pi)
 * @param turnRadius    a turning radius in radians
 * @return An exit code (0==SUCCESS)
 */
int main(int argc, char *argv[]) {
    // Setup stack traces for debugging
    char const *program_name = argv[0];
    #ifdef DEBUG
    set_signal_handler(program_name); // stack trace on signals
    #endif

    // Initialize logging
    FILELog::ReportingLevel() = logDEBUG3;
    FILE* log_fd = fopen( "logfile.txt", "w" );
    Output2FILE::Stream() = log_fd;
    FILE_LOG(logDEBUG) << "Started.";

    // Read arguments
    if (argc != 4) {
        cerr << "Expected 3 arguments." << endl;
        return 1;
    }
    string inputFilename(argv[1]);
    double x = atof(argv[2]);
    double r = atof(argv[3]);

    // Read input gml file
    Graph G;
    GraphAttributes GA(G,
      GraphAttributes::nodeGraphics |
      GraphAttributes::edgeGraphics |
      GraphAttributes::nodeLabel |
      GraphAttributes::edgeStyle |
      GraphAttributes::nodeStyle |
      GraphAttributes::nodeTemplate |
      GraphAttributes::nodeId); 
    
    if (!ogdf::GraphIO::readGML(GA, G, inputFilename)) {
        cerr << "Could not open " << inputFilename << endl;
        return 1;
    }

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();
    FILE_LOG(logDEBUG) << "Opened " << inputFilename << "." << endl;

    // Find nearest neighbor solution
    List<node> tour;
    NodeArray<double> heading(G);
    double cost = 0.0;
    int result = solveNearestNeighborDTSP(G,GA,x,r,tour,heading,cost);

    if (result != SUCCESS) {
        cerr << "Nearest neighbor algorithm failed" << endl;
        return 1;
    }

    cout << "Solved " << n << " point tour with cost " << cost << "." << endl;

    // Print heading
    ListIterator<node> tourIter;
    cout << "Tour: ";
    for ( tourIter = tour.begin(); tourIter != tour.end(); tourIter++ ) {
        if (tourIter != tour.begin())
            cout << " -> ";
        cout << GA.idNode(*tourIter);
    }
    
    return 0;
}

