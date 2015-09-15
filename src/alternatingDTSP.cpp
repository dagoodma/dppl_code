/*
The MIT License
Copyright (c) 2015 UCSC Autonomous Systems Lab
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <math.h>
#include <chrono>
#include <iostream>
#include <fstream>
#include <stdio.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>

#include <LKH.h>

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
 * Given the graph in the input GML file, this program solves the Euclidean Traveling
 * Salesperson Problem(ETSP), then applies the alternating algorithm to solve the 
 * Dubins Traveling Salesperson Problem (DTSP). The solution is output as a tour inside,
 * a TSPlib file. The total cost is printed at tht end.
 *
 * If called from a MEX file, returns the tour as a list of nodes, a set of headings for
 * each node, and the total cost.
 *
 * As CPP:
 * @param inputGMLFile  input GML file to read the problem from
 * @param x             a starting heading in radians [0,2*pi)
 * @param r             a turning radius in radians
 * @return An exit code (0==SUCCESS)
 *
 * In MEX MODE:
 * @param G         a graph of the problem
 * @param GA        attributes of graph
 * @param x         a starting heading in radians [0,2*pi)
 * @param r         a turning radius in radians
 * @param pTour     a pointer to list of nodes representing the tour
 * @param pheading  a pointer to NodeArray of headings
 * @param pCost     a pointer to the total cost
 * @return An exit code (0==SUCCESS)
 */
int solveAlternatingDTSP(Graph &G, GraphAttributes &GA, double x, double r,
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

    // Read tour file.
    if (readTSPTourFile(tourFilename, G, GA, tour) != SUCCESS) {
        return 1;
    }

    // Apply alternating algorithm
    ListIterator<node> iter;
    cost = 0.0;
    int i = 1;
    #ifdef DEBUG
    cout << "Tour: " << endl;
    #endif
    for ( iter = tour.begin(); (i < tour.size() && iter != tour.end()); iter++ ) {
        node u = *iter, v = *(iter.succ());

        #ifdef DEBUG
        cout << "   Node " << GA.idNode(u) << endl;
        #endif

        // If odd
        if (fmod(i,2) != 0) {
            Vector2d uv(GA.x(u), GA.y(u));
            Vector2d vv(GA.x(v), GA.y(v));
            heading[u] = headingBetween(uv, vv);
        }
        // If even
        else {
            v = *(iter.pred());
            heading[u] = heading[v];
        }

        i++;
    }

    // Print headings
    #ifdef DEBUG
    node u;
    cout << "Headings: " << endl;
    forall_nodes(u,G) {
        cout << "   Node " << GA.idNode(u) << ": " << heading[u] << " rad." << endl;
    }
    #endif


// TODO enable this in MEX mode
#ifndef USE_MEX_MODE  
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
        
#endif
    return 0;
}


/* Main Entry Point
 * If this isn't being called from MEX, and we're not compiled as a library, then 
 * this defines the main entry point.
 *
 * usage: alternatingDTSP <inputGMLFile> <startHeading> <turnRadius>
 */
#if !defined(USE_MEX_MODE) && !defined(DUBINS_IS_LIBRARY)
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

    // TODO Set option parsing
    //cxxopts::Options opts(program_name, " - computes a weighted adjacency matrix with Dubins' edge costs");
    
    // Read arguments
    if (argc != 4) {
        cerr << "Expected 3 arguments." << endl;
        return 1;
    }
    string inputFilename(argv[1]);
    double x = atof(argv[2]);
    double r = atof(argv[3]);

    List<node> tour;
    double cost;

    // Load the graph (from GML)
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

    NodeArray<double> heading(G);

    FILE_LOG(logDEBUG) << "Opened " << inputFilename << "." << endl;

    return 
#endif


