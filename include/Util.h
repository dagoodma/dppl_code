
#ifndef _UTIL_H
#define _UTIL_H

#include <stdio.h>
#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <Eigen/Dense>

using Eigen::Vector3d;

/*
**
 * Configuration (x,y,heading).
typedef struct _configuration_t {
    ogdf::DPoint position; // (x,y) position [m]
    double heading;        // heading from North (positive y-axis)  [rad]
} configuration_t;


**
 * Copies configuration u to v.
inline void copyConfiguration(configuration_t &u, configuration_t &v) {
    v.position = ogdf::DPoint(u.position);
    v.heading = u.heading;
}
*/

/**
 * Wraps an angle to [0, 2*pi).
 */
inline double wrapAngle(double x) {
    return fmod(x, 2.0*M_PI);
}

/**
 * Find the 2D heading angle between the vectors (ignores extra dimensions).
 */
inline double angleBetween(Vector3d u, Vector3d v) {
        double x1 = u[0];
        double x2 = v[0];
        double y1 = u[1];
        double y2 = v[1];

        double psi = 0.0;

        // Check quadrant
        if (x1 <= x2 && y1 < y2)
            psi = atan((x2 - x1)/(y2 - y1));
        else if (x1 < x2 && y1 >= y2)
            psi = atan((y1 - y2)/(x2 - x1) + M_PI/2.0);
        else if (x2 <= x1 && y2 < y1)
            psi = atan((x1 - x2)/(y1 - y2)) + M_PI;
        else if (x2 < x1 && y1 <= y2)
            psi = atan((y2 - y1)/(x1 - x2)) + 3.0*M_PI/2.0;
        else
            throw std::out_of_range ("uncovered quadrant");

        return psi;
    }
/**
 * Returns L2-Norm of a vector.
 */
/*
inline double l2_norm(Vector3d const& u) {
    double accum = 0.0;
    for (double x : u) {
        accum += x * x;
    }
    return sqrt(accum);
} 
*/

/**
 * Prints a list of all nodes and their (x,y) positions.
 */
inline void printGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    cout << "Graph 0x" << &G << " with " << G.numberOfNodes() << " nodes and "
         << G.numberOfEdges() << " edges:" << endl;

    ogdf::node u;
    forall_nodes(u,G) {
        cout << "   Node " << GA.idNode(u) << " at (" << GA.x(u) << ", " << GA.y(u) << ")." << endl;
    }
}

#endif // _UTIL_H
 
