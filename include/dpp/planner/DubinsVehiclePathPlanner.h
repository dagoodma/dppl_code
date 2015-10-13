#ifndef _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_
#define _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <dpp/planner/PathPlanner.h>
#include <dpp/planalg/NearestNeighborDTSP.h>
#include <dpp/planalg/AlternatingDTSP.h>
#include <dpp/planalg/RandomizedDTSP.h>

namespace dpp {

class DubinsVehiclePathPlanner : public PathPlanner {
public:
    enum PlanningAlgorithm {NEAREST_NEIGHBOR, ALTERNATING, RANDOMIZED};

    DubinsVehiclePathPlanner(double turnRadius=1.0)
        : m_turnRadius(turnRadius),
        PathPlanner(new RandomizedDTSP)
    { }

    ~DubinsVehiclePathPlanner()
    { }

    void addWaypoints(std::string gmlFilename);

    void addWaypoints(ogdf::Graph &G, ogdf::GraphAttributes &GA);

    double turnRadius(void) {
        return m_turnRadius;
    }

    void turnRadius(double r) {
        m_turnRadius = r;
    }

    void algorithm(PlanningAlgorithm algId);

    const std::string algorithm(void) {
        return m_algorithm->name();
    }

    bool solve(void);

    void copySolution(ogdf::Graph &G, ogdf::GraphAttributes &GA,
        ogdf::List<ogdf::node> &Tour, ogdf::List<ogdf::edge> &Edges,
        NodeArray<double> &Headings, double &cost);

private:
    double m_turnRadius;

};

} // namespace dpp

#endif // _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_