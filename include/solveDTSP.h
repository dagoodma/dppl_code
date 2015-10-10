#ifndef _DPP_SOLVE_DTSP_H_
#define _DPP_SOLVE_DTSP_H_

#include <memory>
#include <dpp/basic/basic.h>
 
#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

typedef dpp::DubinsVehiclePathPlanner::PlanningAlgorithm PlanningAlgorithm;

int solveDTSP(ogdf::Graph &G, ogdf::GraphAttributes &GA, double x, double r,
    ogdf::List<ogdf::node> &Tour, ogdf::List<ogdf::edge> &Edges, NodeArray<double> &Headings,
    double &cost, bool returnToInitial = true, PlanningAlgorithm alg=PlanningAlgorithm::RANDOMIZED);

#endif // _DPP_SOLVE_DTSP_H_