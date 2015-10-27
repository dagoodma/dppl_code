#ifndef _DPP_DUBINS_VEHICLE_PATH_PLANNER_SIMPLE_H_
#define _DPP_DUBINS_VEHICLE_PATH_PLANNER_SIMPLE_H_

#include <dpp/planner/DubinsVehiclePathPlanner.h>

namespace dpp {

#define DPP_SEQUENCE_ID_NOT_SET     -1

typedef struct Waypoint {
    double x;
    double y;
} Waypoint;

typedef struct WaypointSequenceTransform {
    int oldIndex;
    int newIndex;
} WaypointSequenceTransform;

typedef std::vector<Waypoint> WaypointList;

class WaypointSequencePlanner : private DubinsVehiclePathPlanner {
public:
    using DubinsVehiclePathPlanner::DtspPlanningAlgorithm;
    using DubinsVehiclePathPlanner::algorithm;
    using PathPlanner::algorithmName;
    using DubinsPathPlanner::initialHeading;
    using DubinsPathPlanner::turnRadius;
    using PathPlanner::waypointCount;
    using PathPlanner::cost;
    using PathPlanner::haveSolution;

    WaypointSequencePlanner(double turnRadius = 1.0, double initialHeading = 0.0,
        Algorithm *alg = new AlternatingDtsp)
        : DubinsVehiclePathPlanner(turnRadius, alg),
           m_sequenceTransformList(m_G)
    {
        m_initialHeading = initialHeading;
    }


    ~WaypointSequencePlanner()
    { }

    bool solve(void) {
        return planWaypointSequence();
    }

    bool planWaypointSequence(void);

    int newWaypointSequenceId(int oldIndex);

    std::vector<int> newWaypointSequenceList(void);

    // FIXME move some of these functions up to the PathPlanner abstract class and overload them?
    void addWaypoints(const WaypointList list);

    int addWaypoint(const Waypoint waypoint);

    bool containsWaypoint(double x, double y);

    bool containsWaypoint(Waypoint waypoint) {
        return containsWaypoint(waypoint.x, waypoint.y);
    }

private:
    ogdf::NodeArray<WaypointSequenceTransform> m_sequenceTransformList;
    std::vector<ogdf::node> m_originalNodeList, m_newNodeList;
};

} // dpp

#endif // _SIMPLE