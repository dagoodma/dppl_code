#ifndef _DPP_ALGORITHM_NEAREST_NEIGHBOR_DTSP_H_
#define _DPP_ALGORITHM_NEAREST_NEIGHBOR_DTSP_H_

#include <ogdf/basic/GraphCopy.h>

#include <dpp/planalg/Algorithm.h>

namespace dpp {
/*
 * Nearest neighbor algorithm for the DTSP.
 */
class NearestNeighborDTSP : public AlgorithmDTSP {
public:
    NearestNeighborDTSP()
        : AlgorithmDTSP(std::string("NearestNeighbor"))
    { }

    ~NearestNeighborDTSP()
    { }

    int run(Graph &G, GraphAttributes &GA, double x, double r, List<node> &Tour,
        List<edge> &Edges, NodeArray<double> &Headings, double &cost,
        bool returnToInitial=true);
};

} // namespace dpp
#endif // _DPP_ALGORITHM_NEAREST_NEIGHBOR_DTSP_H_