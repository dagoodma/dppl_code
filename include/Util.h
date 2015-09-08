
#ifndef _UTIL_H
#define _UTIL_H

#include <stdio.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

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
 
