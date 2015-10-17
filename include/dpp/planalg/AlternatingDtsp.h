#ifndef _DPP_ALGORITHM_ALTERNATING_DTSP_H_
#define _DPP_ALGORITHM_ALTERNATING_DTSP_H_

#include <ogdf/basic/GraphCopy.h>

#include <dpp/planalg/Algorithm.h>

namespace dpp {
/*
 * Alternating algorithm for the Dtsp.
 */
class AlternatingDtsp : public AlgorithmDtsp {
public:
    AlternatingDtsp()
        : AlgorithmDtsp(std::string("Alternating"))
    { }

    ~AlternatingDtsp()
    { }

    int run(Graph &G, GraphAttributes &GA, double x, double r, List<node> &Tour,
        List<edge> &Edges, NodeArray<double> &Headings, double &cost,
        bool returnToInitial=true);
};

} // namespace dpp
#endif // _DPP_ALGORITHM_ALTERNATING_DTSP_H_