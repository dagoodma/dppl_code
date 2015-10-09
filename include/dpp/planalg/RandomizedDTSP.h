#ifndef _DPP_RANDOMIZED_DTSP_H_
#define _DPP_RANDOMIZED_DTSP_H_

#include <dpp/planalg/Algorithm.h>

namespace dpp {

/*
 * Randomized heading algorithm for the DTSP.
 */
class RandomizedDTSP : public AlgorithmDTSP {
public:
    RandomizedDTSP()
        : AlgorithmDTSP(std::string("Randomized"))
    { }

    ~RandomizedDTSP()
    { }

    int run(Graph &G, GraphAttributes &GA, double x, double r, List<node> &Tour,
        List<edge> &Edges, NodeArray<double> &Headings, double &cost,
        bool returnToInitial=true);

};

} // namespace dpp
#endif // _DPP_ALGORITHM_RANDOMIZED_DTSP_H_