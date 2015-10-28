#ifndef _DPP_DUBINS_SENSOR_PATH_PLANNER_H_
#define _DPP_DUBINS_SENSOR_PATH_PLANNER_H_

#include <dpp/basic/VehicleConfiguration.h>
#include <dpp/planner/PathPlanner.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>
#include <dpp/planalg/FieldTracksCpp.h>

using ogdf::DPoint;
using ogdf::DPolygon;

namespace dpp {

class DubinsSensorPathPlanner : public DubinsPathPlanner {
public:
    enum CppPlanningAlgorithm { FIELD_TRACKS };

    DubinsSensorPathPlanner(double turnRadius=1.0, double sensorWidth=1.0)
        : m_sensorWidth(sensorWidth),
        DubinsPathPlanner(turnRadius, new FieldTracksCpp)
    { }

    ~DubinsSensorPathPlanner()
    { }

    void polygon(std::string gmlFilename);

    void polygon(DPolygon polygon);

    void polygon(List<DPoint> points);

    DPolygon& polygon(void) {
        return m_polygon;
    }

    /// Set the sensor coverage width.
    void sensorWidth(double e) {
        m_sensorWidth = e;
    }

    /// Get the sensor coverage width.
    double sensorWidth(void) {
        return m_sensorWidth;
    }

    /// Get the initial vehicle configuration
    VehicleConfiguration initialConfiguration(void)  {
        return m_initialConfig;
    }

    /// Set the initial vehicle configuration.
    void initialConfiguration(VehicleConfiguration C)  {
        m_initialConfig = C;
        m_initialHeading = m_initialConfig.heading();
    }

    /// Override to update heading of vehicle config.
    void initialHeading(double x) {
        m_initialConfig.heading(x);
        m_initialHeading = x;
    }

    void algorithm(CppPlanningAlgorithm algId);

    bool solveAsDtsp(DtspPlanningAlgorithm algId = DtspPlanningAlgorithm::ALTERNATING);

    bool solve(void);

protected:
    DPolygon m_polygon;
    double m_sensorWidth;
    VehicleConfiguration m_initialConfig; 

}; // DubinsSensorPathPlanner

typedef DubinsSensorPathPlanner::CppPlanningAlgorithm CppPlanningAlgorithm;

} // namespace dpp

#endif // _DPP_DUBINS_SENSOR_PATH_PLANNER_H_