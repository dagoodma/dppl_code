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
#include <vector>
#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/fileformats/GraphIO.h>

#include "Log.h"
#include "Dubins.h"
 
using namespace ogdf;
using namespace std;


// Prototypes
double solveETSPNearestNeighbor(Graph &G, GraphAttributes &GA, configuration_t C_sart, configuration_t C_end, List<node> &tour);

/**
 * Find the node nearest to the configuration using the Euclidean distance metric.
 */
double findNearestNode(Graph &G, GraphAttributes &GA, configuration_t &C, node *v) {
    double minDist = -1.0f;

    node u;
    forall_nodes(u,G) {
        double dist = euclideanDistanceToNode(G, GA, C, u);
        if (minDist < 0.0f || dist < minDist) {
            v = &u;
            minDist = dist;
        }
    }

    return minDist;
}
    
 
int main(int argc, char *argv[]) {
    // Initialize logging
    FILELog::ReportingLevel() = logDEBUG3;
    FILE* log_fd = fopen( "logfile.txt", "w" );
    Output2FILE::Stream() = log_fd;
    FILE_LOG(logDEBUG) << "Started.";

    // Read arguments
    if (argc <= 1 || argc > 2) { 
        cerr << "Expected only 1 argument.\n";
        return 1;
    }
    char *pFilename = argv[1];

    // Read input gml file
    Graph G;
    GraphAttributes GA(G, GraphAttributes::nodeGraphics | GraphAttributes::edgeGraphics );
    
    if (!GraphIO::readGML(GA, G, pFilename)) {
        cerr << "Could not open " << pFilename << endl;
        return 1;
    }
    int m = G.numberOfEdges();
    int n = G.numberOfNodes();
    FILE_LOG(logDEBUG) << "Opened " << pFilename << ". Found " << m << " edges, and " << n << " nodes." << endl;

    // Set start and end positions
    configuration_t C_start, C_end;
    C_start.position.m_x = 0.0f;
    C_start.position.m_y = 0.0f;
    C_start.heading = 0.0f;
    copyConfiguration(C_start,C_end);
    
    // Find nearest neighbor solution
    List<node> tour;
    FILE_LOG(logDEBUG) << "Starting solver.";
    double cost = solveETSPNearestNeighbor(G,GA,C_start,C_end,tour);
    FILE_LOG(logDEBUG) << "Finished solving with cost " << cost << ".";

    cout << "Solved " << n << " point tour with cost " << cost << ".\n";

    // Write solution to GML file
    ListIterator<node> tourIter;
    cout << "Tour: ";
    for ( tourIter = tour.begin(); tourIter != tour.end(); tourIter++ ) {
        cout << GA.idNode(*tourIter) << " -> ";
    }
    cout << ".\n";
    
    return 0;
}


//double solveETSPArora()

/**
 * Solves the Euclidean Traveling Salesperson problem using the Nearest Neighbor algorithm.
 */
double solveETSPNearestNeighbor(Graph &G, GraphAttributes &GA, configuration_t C_start, configuration_t C_end, List<node> &tour) {
    double minCost = 0.0f;
    GraphCopy G_v = GraphCopy(G); // unvisited nodes

    configuration_t C;
    copyConfiguration(C_start, C);
    
    while (!G_v.empty()) {
        node v;
        minCost += findNearestNode(G_v, GA, C, &v);
        tour.insertAfter(v, tour.end());
        G_v.delNode(v);

        // Update configuration
        C.position.m_x = GA.x(v);
        C.position.m_y = GA.y(v);
    }

    // Return to start configuration
    minCost += C.position.distance(C_end.position);

    return minCost;
}
 
