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


//#define DEBUG

#define TSP_FILE_EXTENSION ".tsp"
#define PAR_FILE_EXTENSION ".par"

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
 * Given the graph in the input GML file, this program solves the Symmetric Traveling
 * Salesperson Problem (ETSP) using euclidean distances as a metric. The solution is 
 * saved as a TSPLIB tour.
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
        << n << " nodes.";

    // Generate temporary TSP and PAR files
    string problemComment("Euclidean TSP problem with ");
    problemComment += to_string(n) + " nodes.";
    size_t pos = outputFilename.find(".tsp");
    string problemName(outputFilename,0,pos);

    string parFilename = (std::tmpnam(nullptr) + string(PAR_FILE_EXTENSION)),
        tspFilename = (std::tmpnam(nullptr) + string(TSP_FILE_EXTENSION));
    if (writePARFile(parFilename,tspFilename, outputFilename) != SUCCESS
        || writeETSPFile(tspFilename, problemName, problemComment, G, GA) != SUCCESS);
        return 1;

    FILE_LOG(logDEBUG) << "Wrote " << parFilename << " and " << tspFilename << ".";
    FILE_LOG(logDEBUG) << "Running LKH solver for Euclidean TSP.";

    // Find Euclidean TSP solution with LKH into outputFilename
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
        << " Optimal euclidean tour in " << outputFilename << ".";
    cout << "Computed Euclidean TSP solution in " <<  elapsedTime << "ms."
        << endl << "Tour saved in " << outputFilename << "." << endl;

    // Cleanup
    if (std::remove(tspFilename.c_str()) != 0 
        && std::remove(parFilename.c_str()) != 0) {
        cerr << "Failed to delete temporary files " << tspFilename << ", " << parFilename << "." << endl;
        FILE_LOG(logDEBUG) << "Failed deleting temporary files " << tspFilename << ", " << parFilename << ".";
        return 1;
    }
    FILE_LOG(logDEBUG) << "Removed temporary files " << tspFilename << ", " << parFilename << ".";
        
    return 0;
}


