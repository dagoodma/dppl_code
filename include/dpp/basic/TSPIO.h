#ifndef _DPP_TSPIO_H_
#define _DPP_TSPIO_H_

#include <dpp/basic/NodeMatrix.h>

#define TSP_FILE_EXTENSION ".tsp"
#define PAR_FILE_EXTENSION ".par"

namespace dpp {

class TSPIO {
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
    ogdf::List<ogdf::node> &tour, bool returnToInitial=true);

} // namespace dpp

#endif // _DPP_TSPIO_H_
