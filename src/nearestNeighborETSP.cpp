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

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>

#include "Log.h"
#include "Dubins.h"
#include "Configuration.h"
#include "Util.h"

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


// Prototypes
double euclideanDistanceToNode(GraphAttributes &GA, Configuration &C, node &node);
double solveETSPNearestNeighbor(Graph &G, GraphAttributes &GA, Configuration &C_start,
    Configuration &C_end, List<node> &tour);

/**
 * Find the copied node vC nearest to C using the Euclidean distance metric.
 */
double findNearestNode(GraphCopy &GC, GraphAttributes &GA, Configuration &C, node &vC) {
    double minDist = -1.0f;
    node iC;
    forall_nodes(iC,GC) {
        node i = GC.original(iC);
        double dist = euclideanDistanceToNode(GA, C, i);
        if (minDist < 0.0f || dist < minDist) {
            vC = iC;
            minDist = dist;
        }
    }

    return minDist;
}

/**
 * Calculate euclidean distance from C to the node.
 */
double euclideanDistanceToNode(GraphAttributes &GA, Configuration &C, node &node) {
    DPoint uPoint = DPoint(GA.x(node), GA.y(node));
    return C.m_position.distance(uPoint);
    //return sqrt(pow(u_x - v_x,2) + pow(u_y - v_y,2));
}    

/**
 * nearestNeighbor solution to ETSP.
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
    if (argc <= 1 || argc > 2) { 
        cerr << "Expected only 1 argument." << endl;
        return 1;
    }
    char *pFilename = argv[1];

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
    
    if (!ogdf::GraphIO::readGML(GA, G, pFilename)) {
        cerr << "Could not open " << pFilename << endl;
        return 1;
    }

    int m = G.numberOfEdges();
    int n = G.numberOfNodes();
    FILE_LOG(logDEBUG) << "Opened " << pFilename << ". Found " << m << " edges, and "
        << n << " nodes." << endl;

    // Set start and end positions
    Configuration C_start(0.0f, 0.0f, 0.0f), C_end;
    C_end = C_start;
    
    // Find nearest neighbor solution
    List<node> tour;
    FILE_LOG(logDEBUG) << "Starting solver.";
    double cost = solveETSPNearestNeighbor(G,GA,C_start,C_end,tour);
    FILE_LOG(logDEBUG) << "Finished solving with cost " << cost << ".";

    cout << "Solved " << n << " point tour with cost " << cost << "." << endl;

    // Write solution to GML file
    ListIterator<node> tourIter;
    cout << "Tour: ";
    for ( tourIter = tour.begin(); tourIter != tour.end(); tourIter++ ) {
        if (tourIter != tour.begin())
            cout << " -> ";
        cout << GA.idNode(*tourIter);
    }
    cout << "." << endl;
    
    return 0;
}


/**
 * Solves the Euclidean Traveling Salesperson problem using the Nearest Neighbor algorithm.
 */
double solveETSPNearestNeighbor(Graph &G, GraphAttributes &GA, Configuration &C_start, Configuration &C_end, List<node> &tour) {
    double minCost = 0.0f;
    GraphCopy GC(G); // unvisited nodes

    Configuration C(C_start);

    #ifdef DEBUG
    printGraph(G, GA);
    #endif

    while (!GC.empty()) {
        node vC;
        minCost += findNearestNode(GC, GA, C, vC);
        node v = GC.original(vC);
        tour.pushBack(v);
        GC.delNode(vC);

        // Update configuration
        C.setPosition(GA.x(v), GA.y(v));
    }

    // Return to start configuration
    minCost += C.m_position.distance(C_end.m_position);

    return minCost;
}
