#ifndef _CONFIGURATION_H_
#define _CONFIGURATION_H_

#include <math.h>
#include <stdexcept>      // std::out_of_range

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Vector2d;

class Configuration 
{
public:
    // Public attributes
    ogdf::DPoint m_position;
    double m_heading; // [rad[

    // Public methods
    Configuration() {
        m_position.m_x = 0.0;
        m_position.m_y = 0.0;
        m_heading = 0.0;
    }

    Configuration(double x, double y, double heading=0.0) {
        m_position.m_x = x;
        m_position.m_y = y;
        m_heading = heading;
    }

    Configuration(const Configuration &C) {
        m_position = C.m_position;
        m_heading = C.m_heading;
    }

    // Operator overloading
    Configuration & operator=(const Configuration &C);
    friend bool operator==(Configuration &C1, Configuration &C2);
    friend bool operator!=(Configuration &C1, Configuration &C2);
    friend std::ostream & operator<<(std::ostream & stream, Configuration const & v);

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

    double euclideanDistance(Configuration &C) {
        return m_position.distance(C.m_position);
    }
};

inline Configuration & Configuration::operator=(const Configuration &C) {
    m_position = C.m_position;
    m_heading = C.m_heading;

    return *this;
}

inline bool operator==(Configuration &C1, Configuration &C2) {
    return (C1.m_position.m_x == C2.m_position.m_x &&
            C1.m_position.m_y == C2.m_position.m_y &&
            C1.m_heading == C2.m_heading);
}

inline bool operator!=(Configuration &C1, Configuration &C2) {
    return !(C1 == C2);
}

inline std::ostream & operator<<(std::ostream & stream, const Configuration & v) {
    stream << "(" << v.m_position.m_x << ", " << v.m_position.m_y << ", " << v.m_heading << ")";
}


#endif // _CONFIGURATION_H_H
