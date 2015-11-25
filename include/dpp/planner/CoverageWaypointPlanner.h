#ifndef _DPP_COVERAGE_WAYPOINT_PLANNER_H_
#define _DPP_COVERAGE_WAYPOINT_PLANNER_H_

#include <dpp/planner/DubinsSensorPathPlanner.h>
#include <dpp/planner/WaypointSequencePlanner.h>

namespace dpp {

typedef Waypoint Vertex;
typedef WaypointList VertexList;

class CoverageWaypointPlanner : private DubinsSensorPathPlanner {
public:
    using DubinsSensorPathPlanner::initialConfiguration;
    using DubinsSensorPathPlanner::sensorWidth;
    using DubinsPathPlanner::turnRadius;
    using DubinsSensorPathPlanner::algorithm;
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
        DPP_ASSERT(haveSolution());
        return m_waypointList;
    }

    // FIXME move some of these functions up to the DubinsSensorPathPlanner
    void addPolygonVertex(Vertex v);

    int addPolygonVertices(VertexList list);

    int vertexCount(void) {
        return m_polygon.size();
    }

private:
    WaypointList m_waypointList;
};

} // dpp

#endif // _DPP_COVERAGE_WAYPOINT_PLANNER_H_