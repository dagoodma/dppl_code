#ifndef _DPP_FILE_IO_H_
#define _DPP_FILE_IO_H_

#include <ogdf/fileformats/GraphIO.h>

#include <dpp/basic/NodeMatrix.h>

#define TSP_FILE_EXTENSION ".tsp"
#define PAR_FILE_EXTENSION ".par"

namespace dpp {

/**
 * Class for TSPlib files.
 */
class TspFile {
public:
    enum ProblemType { ETSP, ATSP, HCP };
    static const char *ProblemTypeText[];
};

/**
 * Class for GML files.
 */
class GmlFile {
public:
    enum Type { GRAPH, DPOLYGON };
    static const char *TypeText[];
};

/**
 * For read/write of a DLM (CSV) file. Reads List of DPoints.
class PolygonFile {
public:
    enum Type {}
}; // class DlmFile
*/

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

// Static function to eventually add into classes
int writeAtspFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A);

int writeEtspFile(std::string filename, std::string name, std::string comment,
    ogdf::Graph &G, ogdf::GraphAttributes &GA);

int writeParFile(std::string filename, std::string tspFilename, std::string outputFilename, int runs=10);

int readTspTourFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour, bool returnToInitial=true);


int readPointsFromGmlFile(std::string filename, ogdf::List<ogdf::DPoint> &Points);
 
int readGraphFromGmlFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA);

int readPolygonFromGmlFile(std::string filename, ogdf::DPolygon &poly);

} // namespace dpp

#endif // _DPP_FILE_IO_H_
