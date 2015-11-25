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
        this->heading(heading); // for debug assertion on bounds
    }

    VehicleConfiguration(const VehicleConfiguration &C) {
        m_position = C.m_position;
        m_heading = C.m_heading;
    }

    // Operator overloading
    /// Copy constructor
    VehicleConfiguration & operator=(const VehicleConfiguration &C) {
        if (C != *this) {
            m_position = C.m_position;
            m_heading = C.m_heading;
        }
        return *this;
    }

    bool operator==(const VehicleConfiguration &C) const {
        return (m_position == C.m_position &&
                m_heading == C.m_heading);
    }

    bool operator!=(VehicleConfiguration &C) const {
        return !(*this == C);
    }

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

    ogdf::DPoint& position(void) {
        return m_position;
    }

    // Modifiers
    void position(double x, double y) {
        m_position.m_x = x;
        m_position.m_y = y;
    }

    void heading(double x) {
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

inline std::ostream & operator<<(std::ostream & stream, const VehicleConfiguration & v) {
    stream << "(" << v.m_position.m_x << ", " << v.m_position.m_y << ", " << v.m_heading << ")";
    return stream;
}

} // namespace dpp

#endif // _DPP_VEHICLE_CONFIGURATION_H_
