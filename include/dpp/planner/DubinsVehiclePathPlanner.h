#ifndef _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_
#define _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_

#include <dpp/planner/PathPlanner.h>
#include <dpp/planalg/NearestNeighborDtsp.h>
#include <dpp/planalg/AlternatingDtsp.h>
#include <dpp/planalg/RandomizedDtsp.h>

namespace dpp {

class DubinsVehiclePathPlanner : public DubinsPathPlanner {
public:
    enum DtspPlanningAlgorithm {NEAREST_NEIGHBOR, ALTERNATING, RANDOMIZED};

    DubinsVehiclePathPlanner(double turnRadius = 1.0,
        Algorithm *alg = new AlternatingDtsp)
        : DubinsPathPlanner(turnRadius, alg)
    { }

    ~DubinsVehiclePathPlanner()
    { }

    void addWaypoints(std::string gmlFilename);

    void addWaypoints(ogdf::Graph &G, ogdf::GraphAttributes &GA);

    void algorithm(DtspPlanningAlgorithm algId);

    bool solve(void);

private:

};

typedef DubinsVehiclePathPlanner::DtspPlanningAlgorithm DtspPlanningAlgorithm;


} // namespace dpp


#endif // _DPP_DUBINS_VEHICLE_PATH_PLANNER_H_