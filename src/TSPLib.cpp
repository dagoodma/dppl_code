#include <stdio.h>

#include "TSPLib.h"

int writeATSPFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A) {
    ofstream tspFile;
    tspFile.open(filename);
    if (!tspFile.is_open()) {
        cerr << "Could not open " << filename << std::endl;
        return 1;
    }

    // Find largest number of digits
    int nDigits = 12; // eg. 3.14159e+000, 2.00600e+003
    int n = A.numberOfNodes();
    //cout << "HERE at line " << __LINE__ << " in file " << __FILE__ << "." << std::endl;

    tspFile << 
        "NAME: " << name << std::endl << 
        "TYPE: ATSP" << std::endl <<
        "COMMENT:" << comment << std::endl <<
        "DIMENSION: " << n <<  std::endl <<
        "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl <<
        "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl <<
        "EDGE_WEIGHT_SECTION" << std::endl << std::fixed;
    tspFile.precision(0); 

    ogdf::node i, j;
    //Graph G = *A.graphOf();
    forall_nodes(i, G) {
        forall_nodes(j, G) {
            tspFile << " " << A[i][j];
        }
        tspFile << std::endl;
    }

    tspFile << "EOF";
    tspFile.close();
    return 0;
}



