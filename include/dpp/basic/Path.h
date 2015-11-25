#ifndef _DPP_PATH_H_
#define _DPP_PATH_H_

#include <stdbool.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/basic/Path.h>
#include <dpp/basic/NodeMatrix.h>


#define DPP_MAX_EDGE_COST         999999.0 // FIXME scale this based on edge costs

namespace dpp {


// Public Prototypes
double dubinsPathLength(VehicleConfiguration &Cs, VehicleConfiguration &Ce,
	double turnRadius);

double dubinsTourCost(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &Tour, ogdf::NodeArray<double> &Headings,
    double turnRadius, bool returnCost = false);

double createDubinsTourEdges(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &Tour, ogdf::NodeArray<double> &Headings,
    double turnRadius, ogdf::List<ogdf::edge> &Edges, bool returnEdge = false);

void buildDubinsAdjacencyMatrix(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    NodeMatrix<double> &A, ogdf::NodeArray<double> &Headings, double turnRadius);

} // namespace dpp

#endif // _DPP_PATH_H_
