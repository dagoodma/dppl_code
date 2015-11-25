#ifndef _DPP_BOUTSTROPHEDON_CPP_H_
#define _DPP_BOUTSTROPHEDON_CPP_H_

#include <dpp/planalg/Algorithm.h>
#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/basic/Field.h>
#include <dpp/basic/Util.h>

namespace dpp {

/*
 * Boustrophedon algorithm for CPP.
 */
class BoustrophedonCpp : public AlgorithmCpp {
public:
    BoustrophedonCpp()
        : AlgorithmCpp(std::string("Boustrophedon"))
    { }

    ~BoustrophedonCpp()
    { }

    int run(const Field &field, VehicleConfiguration C, double turnRadius,
    bool returnToInitial, Graph &G, GraphAttributes &GA, List<node> &Tour,
    List<edge> &Edges, NodeArray<double> &Headings, double &cost);

};

} // namespace dpp

#endif // _DPP_BOUTSTROPHEDON_CPP_H_