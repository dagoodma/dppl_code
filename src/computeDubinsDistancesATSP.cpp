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

//#include <cxxopts.h>

#include "Log.h"
#include "Dubins.h"
#include "Configuration.h"
#include "Util.h"
#include "NodeMatrix.h"
#include "TSPLib.h"


//#define DEBUG

// Enable stack traces in debug mode
#ifdef DEBUG
#include "stacktrace.h"
#endif

#define DEFAULT_TURN_RADIUS    10.0 // [m]

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
 * Writes Dubins edge distances to a TSPLIB-style problem file (.tsp).
 */
int main(int argc, char *argv[]) {
    // Setup stack traces for debugging
    char const *program_name = argv[0];
    #ifdef DEBUG
    set_signal_handler(program_name);
    #endif

    // Set option parsing
    //cxxopts::Options opts(program_name, " - computes a weighted adjacency matrix with Dubins' edge costs");

    // Initialize logging
    FILELog::ReportingLevel() = logDEBUG3;
    FILE* log_fd = fopen( "logfile.txt", "w" );
    Output2FILE::Stream() = log_fd;
    FILE_LOG(logDEBUG) << "Started.";

    // Read arguments
    if (argc != 3) {
        cerr << "Expected 2 arguments." << endl;
        return 1;
    }
    string inputFilename(argv[1]), outputFilename(argv[2]);

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
    FILE_LOG(logDEBUG) << "Opened " << inputFilename << ". Found " << m << " edges, and "
        << n << " nodes." << endl;
 
    // Build Dubins' weighted adjacency matrix
    //cout << "Here at line " << __LINE__ << " in file " << __FILE__ << "." << endl;
    NodeMatrix<double> A(G);
    NodeArray<double> heading(G,0.0); // (TODO: randomize) 
    buildDubinsAdjacencyMatrix(G, GA, A, heading, DEFAULT_TURN_RADIUS);

    node v = G.firstNode();
    //cout << "At node v: " << A[v][v] << "." << endl;

    // Find nearest neighbor solution
    FILE_LOG(logDEBUG) << "Starting Dubins' adjacency matrix computation." << endl;
    FILE_LOG(logDEBUG) << "Finished." << endl;
    cout << "Computed weighted adjacency matrix." << endl;

    // Write solution to .tsp file
    size_t pos = outputFilename.find(".tsp");
    string problemName(outputFilename,0,pos);
    string problemComment("Dubins' path for ");
    problemComment += to_string(n) + " point scenario.";
    writeATSPFile(outputFilename, problemName, problemComment, G, A);
        
    return 0;
}

