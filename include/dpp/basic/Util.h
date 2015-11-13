#ifndef _DPP_UTIL_H
#define _DPP_UTIL_H

#include <stdlib.h>   
#include <stdio.h>
#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/NodeArray.h>

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
 * Calculates the angle between two vectors.
 * @remark This is technically not the same as heading between two vectors.
 */
inline double angleBetween(Vector2d v, Vector2d w) {
    return acos((v.dot(w)) / (v.norm() * w.norm()));
}


/**
 * Find the 2D heading angle between the vectors. Heading angle is in radians, and
 * increases clockwise where 0 is the positive y-axis.
 * @note Could have used atan2(y,x)
 */
inline double headingBetween(Vector2d v, Vector2d w) {
    double x1 = v[0];
    double x2 = w[0];
    double y1 = v[1];
    double y2 = w[1];

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

/**
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
 * Copies all nodes and attributes from one graph to another.
 * @param G     Graph to copy from.
 * @param GA    Attributes for G.
 * @param Gcopy Graph to copy to.
 * @param GAcopy Attributes for Gcopy.
 * @param nodeCopyTable Optional argument for saving the node translation table. 
 * @param edgeCopyTable Optional argument for saving the edge translation table. 
 * @return Number of nodes copied or FAILURE.
 * @note The copy graph will be cleared.
 */
inline int copyGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::Graph &Gcopy, ogdf::GraphAttributes &GAcopy,
    ogdf::NodeArray<ogdf::node> &nodeCopyTable,
    ogdf::EdgeArray<ogdf::edge> &edgeCopyTable) {
    DPP_ASSERT(&(GA.constGraph()) == const_cast<const ogdf::Graph*>(&G));
    DPP_ASSERT(&(GAcopy.constGraph()) == const_cast<const ogdf::Graph*>(&Gcopy));
    Gcopy.clear();

    // Copy graph nodes and attributes (x, y, and id)
    ogdf::node u;
    int i = 0;
    forall_nodes(u,G) {
        ogdf::node v = Gcopy.newNode();
        GAcopy.x(v) = GA.x(u);
        GAcopy.y(v) = GA.y(u);
        GAcopy.idNode(v) = GA.idNode(u);
        nodeCopyTable(u) = v;
        i++;
    }

    // Copy edges and attributes (weight)
    ogdf::edge e;
    forall_edges(e,G) {
        ogdf::node ucopy = nodeCopyTable(e->source());
        ogdf::node vcopy = nodeCopyTable(e->target());
        ogdf::edge f = Gcopy.newEdge(ucopy, vcopy);
        GAcopy.doubleWeight(f) = GA.doubleWeight(e);
        edgeCopyTable(e) = f;
    }

    return i;
}

inline int copyGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::Graph &Gcopy, ogdf::GraphAttributes &GAcopy) {
    ogdf::NodeArray<ogdf::node> nodeCopyTable(G);
    ogdf::EdgeArray<ogdf::edge> edgeCopyTable(G);
    return copyGraph(G, GA, Gcopy, GAcopy, nodeCopyTable, edgeCopyTable);
}

/**
 * Compares the Graphs' layout (x,y,id) and edges (doubleWeight) for equivalence.
 * @return true if the graphs are equivalent
 */
inline bool graphsAreEquivalent(ogdf::Graph &G, ogdf::GraphAttributes &GA, ogdf::Graph &Gcopy,
    ogdf::GraphAttributes &GAcopy) {
    if (!(G.numberOfNodes() == Gcopy.numberOfNodes()
        && G.numberOfEdges() == Gcopy.numberOfEdges())) {
        return false;
    }
    ogdf::NodeArray<ogdf::node> nodeCopyTable(G);
    ogdf::node u;
    ogdf::node ucopy = Gcopy.firstNode();
    forall_nodes(u,G) {
        if (!(GA.x(u) == GAcopy.x(ucopy)
            && GA.y(u) == GAcopy.y(ucopy)
            && GA.idNode(u) == GAcopy.idNode(ucopy))) {
            return false;
        }
        nodeCopyTable(u) = ucopy;
        ucopy = ucopy->succ();
    }

    ogdf::edge e;
    ogdf::edge ecopy = Gcopy.firstEdge();
    forall_edges(e,G) {
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        double w = GA.doubleWeight(e);

        ogdf::node ucopy = ecopy->source();
        ogdf::node vcopy = ecopy->target();
        double wcopy = GA.doubleWeight(e);
        if (!(ucopy == nodeCopyTable(u)
            && vcopy == nodeCopyTable(v)
            && wcopy == w)) {
            return false;
        }
        ecopy = ecopy->succ();
    }

    return true;
}

/**
 * Prints a list of all nodes and their (x,y) positions to a string.
 * @param G     A graph with edges.
 * @param GA    Attributes of the graph.
 * @return A string with the printed graph.
 */
inline std::string printGraph(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    std::stringstream sout;

    sout << "Graph 0x" << &G << " with " << G.numberOfNodes() << " nodes and "
         << G.numberOfEdges() << " edges:" << endl;

    ogdf::node u;
    forall_nodes(u,G) {
        sout << "   Node " << GA.idNode(u) << " at (" << GA.x(u) << ", " << GA.y(u) << ")." << endl;
    }

    ogdf::edge e;
    forall_edges(e,G) {
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        sout << "   Edge " << e->index() << " from node " << GA.idNode(u)
            << " (" << GA.x(u) << ", " << GA.y(u) << ") to node " << GA.idNode(v)
            << " (" << GA.x(v) << ", " << GA.y(v) << ")." << std::endl;
    }
    return sout.str();
}

/**
 * Prints all edges in the list to a string.
 * @param G     A graph with edges.
 * @param GA    Attributes of the graph.
 * @param Edges An edge list to print.
 * @return A string with the edge list.
 */
inline std::string printEdges(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::edge> &Edges) {
    std::stringstream sout;
    ogdf::ListIterator<ogdf::edge> iter;
    sout << "Edges: " << std::endl;
    for ( iter = Edges.begin(); iter != Edges.end(); iter++ ) {
        ogdf::edge e = *iter;
        ogdf::node u = e->source();
        ogdf::node v = e->target();
        int uid = GA.idNode(u);
        int vid = GA.idNode(v);
        double cost = GA.doubleWeight(e);
        sout << "   " << uid << " -> " << vid << " : " << cost << std::endl;
    }
    return sout.str();
}

/**
 * Prints the tour to a string.
 * @param G     A graph with edges.
 * @param GA    Attributes of the graph.
 * @param Tour  A list of nodes representing the tour.
 * @return A string with the tour.
 */
inline std::string printTour(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &Tour) {
    std::stringstream sout;
    ogdf::ListIterator<ogdf::node> iter;
    sout << "Tour: " << std::endl;
    for ( iter = Tour.begin(); iter != Tour.end(); iter++ ) {
        ogdf::node u = *iter;
        int id = GA.idNode(u);
        double x = GA.x(u);
        double y = GA.y(u);
        sout << "   " << id << " (" << x << ", " << y << ")" << std::endl;
    }
    return sout.str();
}

/**
 * Prints all headings in the node array.
 * @param G     A graph with edges.
 * @param GA    Attributes of the graph.
 * @param Headings A node array of headings for each node.
 * @return A string with the headings.
 */
inline std::string printHeadings(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::NodeArray<double> &Headings) {
    std::stringstream sout;
    ogdf::node u;
    sout << "Headings: " << std::endl;
    forall_nodes(u,G) {
        sout << "   Node " << GA.idNode(u) << ": " << Headings[u] << " rad." << std::endl;
    }

    return sout.str();
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
 
