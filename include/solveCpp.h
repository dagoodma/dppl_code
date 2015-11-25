#ifndef _DPP_SOLVE_CPP_H_
#define _DPP_SOLVE_CPP_H_

#include <memory>
#include <dpp/basic/basic.h>
 
#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

#include <dpp/planner/DubinsSensorPathPlanner.h>
#include <dpp/basic/VehicleConfiguration.h>

 int solveCpp(ogdf::DPolygon poly, dpp::VehicleConfiguration C, ogdf::Graph &G,
    ogdf::GraphAttributes &GA, double r, double e, ogdf::List<ogdf::node> &Tour,
    ogdf::List<ogdf::edge> &Edges, NodeArray<double> &Headings, double &cost,
    bool returnToInitial = true,
    dpp::CppPlanningAlgorithm alg=dpp::CppPlanningAlgorithm::BOUSTROPHEDON);

#endif // _DPP_SOLVE_CPP_H_