#ifndef _DPP_VEHICLE_CONFIGURATION_H_
#define _DPP_VEHICLE_CONFIGURATION_H_

#include <math.h>
#include <stdexcept>      // std::out_of_range

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <dpp/basic/basic.h>

#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Vector2d;

namespace dpp {

class VehicleConfiguration 
{
public:
    // Public attributes
    ogdf::DPoint m_position;
    double m_heading; // [rad[

    // Public methods
    VehicleConfiguration() {
        m_position.m_x = 0.0;
        m_position.m_y = 0.0;
        m_heading = 0.0;
    }

    VehicleConfiguration(double x, double y, double heading=0.0) {
        m_position.m_x = x;
        m_position.m_y = y;
        m_heading = heading;
    }

    VehicleConfiguration(const VehicleConfiguration &C) {
        m_position = C.m_position;
        m_heading = C.m_heading;
    }

    // Operator overloading
    VehicleConfiguration & operator=(const VehicleConfiguration &C);
    friend bool operator==(VehicleConfiguration &C1, VehicleConfiguration &C2);
    friend bool operator!=(VehicleConfiguration &C1, VehicleConfiguration &C2);
    friend std::ostream & operator<<(std::ostream & stream, VehicleConfiguration const & v);

    // Accessors
    double x() {
        return m_position.m_x;
    }

    double y() {
        return m_position.m_y;
    }

    double heading() {
        return m_heading;
    }

    // Modifiers
    void setPosition(double x, double y) {
        m_position.m_x = x;
        m_position.m_y = y;
    }

    void setHeading(double x) {
        DPP_ASSERT(x >= 0 && x < 2.0*M_PI);
        m_heading = x;
    }

    void x(double x) {
        m_position.m_x = x;
    }

    void y(double y) {
        m_position.m_y = y;
    }

    // Methods
    void asArray(double **q) {
        *q = new double[3];
        (*q)[0] = m_position.m_x;
        (*q)[1] = m_position.m_y;
        (*q)[2] = m_heading;
    }

    Vector2d asVector() {
        return Vector2d(m_position.m_x, m_position.m_y);
    }

    double euclideanDistance(VehicleConfiguration &C) {
        return m_position.distance(C.m_position);
    }
};

inline VehicleConfiguration & VehicleConfiguration::operator=(const VehicleConfiguration &C) {
    m_position = C.m_position;
    m_heading = C.m_heading;

    return *this;
}

inline bool operator==(VehicleConfiguration &C1, VehicleConfiguration &C2) {
    return (C1.m_position.m_x == C2.m_position.m_x &&
            C1.m_position.m_y == C2.m_position.m_y &&
            C1.m_heading == C2.m_heading);
}

inline bool operator!=(VehicleConfiguration &C1, VehicleConfiguration &C2) {
    return !(C1 == C2);
}

inline std::ostream & operator<<(std::ostream & stream, const VehicleConfiguration & v) {
    stream << "(" << v.m_position.m_x << ", " << v.m_position.m_y << ", " << v.m_heading << ")";
}

} // namespace dpp

#endif // _DPP_VEHICLE_CONFIGURATION_H_
