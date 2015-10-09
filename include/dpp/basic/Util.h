#ifndef _DPP_UTIL_H
#define _DPP_UTIL_H

#include <stdlib.h>   
#include <stdio.h>
#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include <Eigen/Dense>

#include <dpp/basic/basic.h>

using Eigen::Vector3d;
using Eigen::Vector2d;

namespace dpp {

/**
 * A modulo function that works properly with negatives.
 */
inline double myMod(double x, double y) {
    double n = floor(x/y);
    return x - n*y;
}

/**
 * Converts a heading angle x to a circular angle theta.
 */
inline double headingToAngle(double x) {
    return myMod(-(x - M_PI/2.0), 2.0 * M_PI);
}

/**
 * Wraps an angle to [0, 2*pi).
 */
inline double wrapAngle(double x) {
    return myMod(x, 2.0*M_PI);
}

/** 
 * Converts an angle in degrees to radians.
 */
inline double degToRad(double x) {
    return x * (M_PI/180.0);
}

/** 
 * Converts an angle in radians to degrees.
 */
inline double radToDeg(double x) {
    return x * (180.0/M_PI);
}


/**
 * Find the 2D heading angle between the vectors. Heading angle is in radians, and
 * increases clockwise where 0 is the positive y-axis.
 */
inline double headingBetween(Vector2d u, Vector2d v) {
    double x1 = u[0];
    double x2 = v[0];
    double y1 = u[1];
    double y2 = v[1];

    double psi = 0.0;

    // Check quadrant
    if (x1 <= x2 && y1 < y2) {
        psi = atan((x2 - x1)/(y2 - y1));
    }
    else if (x1 < x2 && y1 >= y2) {
        psi = atan((y1 - y2)/(x2 - x1)) + M_PI/2.0;
    }
    else if (x2 <= x1 && y2 < y1) {
        psi = atan((x1 - x2)/(y1 - y2)) + M_PI;
    }
    else if (x2 < x1 && y1 <= y2) {
        psi = atan((y2 - y1)/(x1 - x2)) + 3.0*M_PI/2.0;
    }
    else {
        throw std::out_of_range ("uncovered quadrant"); 
    }

    return psi;
}
/**
 * Calls the 2D version of the function with the same name by ignoring the 3rd
 * dimension.
 */
inline double headingBetween(Vector3d u, Vector3d v) {
    Vector2d u2(u[0], u[1]),
        v2(v[0], v[1]);
    return headingBetween(u2,v2);
}

/*
 * Clears all edges in the graph.
 */
inline void clearEdges(ogdf::Graph &G) {
    ogdf::edge e, next;
    int i = 1;
    for (e = G.firstEdge(); e; e = next) {
        next = e->succ();
        G.delEdge(e);
    }
}



/**
 * Prints a list of all nodes and their (x,y) positions.
 */
inline void printGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA, std::ostream &out=std::cout) {
    out << "Graph 0x" << &G << " with " << G.numberOfNodes() << " nodes and "
         << G.numberOfEdges() << " edges:" << endl;

    ogdf::node u;
    forall_nodes(u,G) {
        out << "   Node " << GA.idNode(u) << " at (" << GA.x(u) << ", " << GA.y(u) << ")." << endl;
    }

    ogdf::edge e;
    forall_edges(e,G) {
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        out << "   Edge " << e->index() << " from node " << GA.idNode(u)
            << " (" << GA.x(u) << ", " << GA.y(u) << ") to node " << GA.idNode(v)
            << " (" << GA.x(v) << ", " << GA.y(v) << ")." << std::endl;
    }
}

/**
 * Prints all edges in the graph.
 */
inline void printEdges(ogdf::Graph &G, ogdf::GraphAttributes &GA, std::ostream &out=std::cout) {
    ogdf::edge e;
    out << "Edges: ";
    forall_edges(e, G) {
    //for ( edgeIter = edges.begin(); edgeIter != edges.end(); edgeIter++ ) {
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        int uid = GA.idNode(u);
        int vid = GA.idNode(v);
        double cost = GA.doubleWeight(e);
        out <<"   " << uid << " -> " << vid << " : " << cost << endl;
    }
}

/**
 * Prints edge list.
 */
inline void printEdges(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::edge> &edges, std::ostream &out=std::cout) {
    ogdf::ListIterator<ogdf::edge> edgeIter;
    out << "Edges: " << endl;
    for ( edgeIter = edges.begin(); edgeIter != edges.end(); edgeIter++ ) {
        ogdf::edge e = *edgeIter;
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        int uid = GA.idNode(u);
        int vid = GA.idNode(v);
        double cost = GA.doubleWeight(e);
        out <<"   " << uid << " -> " << vid << " : " << cost << endl;
    }
}


/**
 * Performance timer class (c++11 only).
 */
#include <chrono>

class Timer {
public:
    Timer() {
        reset();
    }
    void reset() {
        m_timestamp = std::chrono::high_resolution_clock::now();
    }
    float diffSec() {
        std::chrono::duration<float> fs = std::chrono::high_resolution_clock::now() - m_timestamp;
        return fs.count();
    }
    float diffMs() {
        return diffSec() * 1000.0;
    }
    /*
    double rate() {
        return (double)(std::chrono::high_resolution_clock::period::num / 
            std::chrono::high_resolution_clock::period::den);
    }
    */
private:
    std::chrono::high_resolution_clock::time_point m_timestamp;
};

} // namespace dpp

#endif // _DPP_UTIL_H
 
