#ifndef _DPP_COVERAGE_WAYPOINT_PLANNER_H_
#define _DPP_COVERAGE_WAYPOINT_PLANNER_H_

#include <dpp/planner/DubinsSensorPathPlanner.h>
#include <dpp/planner/WaypointSequencePlanner.h>

namespace dpp {

typedef Waypoint Vertex;

class CoverageWaypointPlanner : private DubinsSensorPathPlanner {
public:
    using DubinsSensorPathPlanner::initialConfiguration;
    using DubinsSensorPathPlanner::sensorWidth;
    using DubinsPathPlanner::turnRadius;
    using PathPlanner::algorithmName;
    using PathPlanner::waypointCount;
    using PathPlanner::cost;
    using PathPlanner::haveSolution;

    CoverageWaypointPlanner(double turnRadius = 1.0, double sensorWidth = 0.0)
        : DubinsSensorPathPlanner(turnRadius, sensorWidth)
    { }


    ~CoverageWaypointPlanner()
    { }

    bool solve(void) {
        return planCoverageWaypoints();
    }

    bool planCoverageWaypoints(void);

    WaypointList waypointList(void) {
        return m_waypointList;
    }

    void initialConfiguration(double x, double y, double heading) {
        VehicleConfiguration C(x, y, heading);
        DubinsSensorPathPlanner::initialConfiguration(C);
    }

    // FIXME move some of these functions up to the DubinsSensorPathPlanner
    void addPolygonVertex(Vertex v);

    int vertexCount(void) {
        return m_polygon.size();
    }

private:
    WaypointList m_waypointList;
};

} // dpp

#endif // _DPP_COVERAGE_WAYPOINT_PLANNER_H_