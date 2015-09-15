#ifndef _TSPLIB_H_
#define _TSPLIB_H_

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include "NodeMatrix.h"

#define TSP_FILE_EXTENSION ".tsp"
#define PAR_FILE_EXTENSION ".par"

class TSPFile {
    public:
        enum ProblemType { ETSP, ATSP, HCP };
        static const char *ProblemTypeText[];
};

/* TODO create classes
    - PARFile
    - TSPFile abstract
    - ATSPFile : public TSPFile
    - ETSPFile : public TSPFile
*/

/*
class TSPFile {
public:
    int writeFile();

};
*/
int writeATSPFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A);

int writeETSPFile(std::string filename, std::string name, std::string comment,
    ogdf::Graph &G, ogdf::GraphAttributes &GA);

int writePARFile(std::string filename, std::string tspFilename, std::string outputFilename, int runs=10);

int readTSPTourFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour);

#endif // _TSPLIB_H_
