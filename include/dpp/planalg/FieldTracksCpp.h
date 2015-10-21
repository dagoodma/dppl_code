#ifndef _DPP_FIELD_TRACKS_CPP_H_
#define _DPP_FIELD_TRACKS_CPP_H_

#include <dpp/planalg/Algorithm.h>

using ogdf::DPolygon;

namespace dpp {

/*
 * FieldTracks heading algorithm for the CPP.
 */
class FieldTracksCpp : public AlgorithmCpp {
public:
    FieldTracksCpp()
        : AlgorithmCpp(std::string("FieldTracks"))
    { }

    ~FieldTracksCpp()
    { }

    int run(void);

};

int addNodesFromPolygonGrid(Graph &G, GraphAttributes &GA, DPolygon &poly, double e);


} // namespace dpp

#endif // _DPP_FIELD_TRACKS_CPP_H_