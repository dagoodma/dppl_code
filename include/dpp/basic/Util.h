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
#include <dpp/basic/Logger.h>

using Eigen::Vector3d;
using Eigen::Vector2d;

using ogdf::DPoint;
using ogdf::DPolygon;
using ogdf::DSegment;
using ogdf::DLine;
using ogdf::DIsEqual;

// FIXME move this all to basic.h and basic.cpp, remove Util.h.

namespace dpp {

/**
 * A modulo function that works properly with negatives.
 */
inline double myMod(double x, double y) {
    double n = floor(x/y);
    return x - n*y;
}

/**
 * Wraps an angle to [0, 2*pi).
 */
inline double wrapAngle(double x) {
    return myMod(x, 2.0*M_PI);
}

/**
 * Converts a heading angle to a circular angle.
 */
inline double headingToAngle(double psi) {
    return wrapAngle(-(psi - M_PI/2.0));
}


/**
 * Converts a circular angle to a heading from North.
 */
inline double angleToHeading(double theta) {
    return headingToAngle(theta); // same result both ways
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
 * Computes the angle between two vectors.
 * @remark This is technically not the same as heading between two vectors.
 */
inline double angleBetween(Vector2d v, Vector2d w) {
    return acos((v.dot(w)) / (v.norm() * w.norm()));
}

/**
 * Computes the angle of a DLine.
 * @return Angle of the line from [0, 2*pi)
 * @remark Angle is taken between the line and the positive x-axis in the
 *   counter-clockwise direction.
 */
inline double angleOfLine(DLine l) {
    return wrapAngle(atan2(l.dy(), l.dx()));
}

/**
 * Computes the angle of a DSegment.
 */
inline double angleOfSegment(DSegment s) {
    return angleOfLine(s);
}

/**
 * Find the 2D heading angle between points on the plane. Heading angle is in
 * radians, and it increases clockwise where 0 is the positive y-axis.
 * @note Consider using atan2(y,x)
 */
inline double headingBetween(DPoint p, DPoint q) {
    double x1 = p.m_x;
    double x2 = q.m_x;
    double y1 = p.m_y;
    double y2 = q.m_y;

    double psi = 0.0;

    // Identical points have no heading between them
    if (p == q) {
        return psi;
    }

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
 * Finds the heading angle between the points pointed at by vectors v and w.
 */
inline double headingBetween(Vector2d v, Vector2d w) {
    return headingBetween(DPoint(v[0], v[1]), DPoint(w[0], w[1]));
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
 * Infinite line in normal form that uses DLine and has polar translation.
 * @remark This is used for sweep-line algorithms in CPP.
 */
 // TODO look into Eigen::ParametrizedLine, esp. for higher dimensions
class Line2d : private DLine
{
public:
    using DLine::isVertical;
    using DLine::isHorizontal;
    using DLine::slope;

    double m_a, m_b, m_c;

    /**
     * Construct the line from a Dline.
     */
    Line2d(DLine line)
        : DLine(line)
    {
        DPoint p1 = start();
        DPoint p2 = end();

        if (!isVertical()) {
            m_a = - (p2.m_y - p1.m_y)/(p2.m_x - p1.m_x);
            m_b = 1;
            m_c = p1.m_y + m_a*p1.m_x;
        }
        else {
            m_a = p1.m_x;
            m_b = 1;
            m_c = std::numeric_limits<double>::max(); // inf
        }
    }

    /**
     * Copy constructor.
     */
    Line2d & operator= (const Line2d &line) {
        if (this != &line) {
            m_start = line.m_start;
            m_end = line.m_end;
            m_a = line.m_a;
            m_b = line.m_b;
            m_c = line.m_c;
        }
        return *this;
    }


    // Equality
    bool operator== (const Line2d &line) const {
        if (isVertical() && line.isVertical()) {
            return DIsEqual(m_a, line.m_a);
        }

        return DIsEqual(m_a, line.m_a) && DIsEqual(m_b, line.m_b)
            && DIsEqual(m_c, line.m_c);
    }

    // Not equals
    bool operator!= (const Line2d &line) const {
        return !(*this == line);
    }

    friend ostream& operator<<(ostream& os, const Line2d& line);

    // Length of a line is infinite.
    double length(void) {
        return std::numeric_limits<double>::max();
    }

    /**
     * Returns the angle of this line.
     * @return Angle of the line, where 0 <= theta < pi
     * @remark Different than the angle of a DLine, which has endpoints.
     */
     double angle(void) {
        if (isVertical()) {
            return M_PI/2; // slope is infinite
        }

        return myMod(atan(slope()), M_PI);
     }

    /**
     * Returns true if the point lies on the line.
     */
    bool contains(DPoint p) {
        if (!isVertical()) {
            return DIsEqual(m_c, m_a * p.m_x + m_b * p.m_y);
        }
        else {
            return DIsEqual(m_a, p.m_x);
        }
    }

    /**
     * Translates the line by polar coordinates (d, theta).
     * @param d     Distance to translate in the direction of theta.
     * @param theta Angle direction to translate the line in. Must satisfy 0 <= theta < 2pi
     * @remark This does not rotate the line (no change in slope).
     * @remark If theta is the angle of this line, translating has no effect.
     */
    void translatePolar(double d, double theta) {
        DPP_ASSERT(0 <= theta && theta < 2*M_PI);

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Translating line: " << *this << " by "
            << d << " at " << radToDeg(theta) << " degrees." << std::endl;

        if (myMod(theta, M_PI) == angle()) {
            Logger::logWarn(DPP_LOGGER_VERBOSE_1) << "Translating line by same angle as line."
                << std::endl;
        }

        //theta = wrapAngle(theta);
        double dx = 0, dy = 0;

        if (theta <= M_PI/2) {
            dx = d*cos(theta);
            dy = d*sin(theta);
        }
        else if (theta <= M_PI) {
            dx = -d*sin(theta - M_PI/2);
            dy = d*cos(theta - M_PI/2);
        }
        else if (theta <= 3*M_PI/2) {
            dx = -d*cos(theta - M_PI);
            dy = -d*sin(theta - M_PI);
        }
        else {
            dx = d*sin(theta - 3*M_PI/2);
            dy = -d*cos(theta - 3*M_PI/2);
        }

        if (!isVertical()) {
            m_c += dy + m_a*dx;
        }
        else {
            m_a += dx;
        }

        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "New line: " << *this << std::endl;
    }

    /**
     * Find the point of intersection with another line.
     * @param[in] line Another line to find intersection with.
     * @param[out] inters Point to hold intersection of lines.
     * @return True if they intersect and are not identical.
     */
    bool intersection(Line2d line, DPoint &inters) {
        bool result = false;
        // Will this work if either is infinite?
        if (!DIsEqual(slope(), line.slope())) {
            if (!isVertical() && !line.isVertical()) {
                inters.m_x = (m_c * line.m_b - m_b * line.m_c)/(m_a * line.m_b - m_b * line.m_a);
                inters.m_y = (m_a  * line.m_c - m_c * line.m_a)/(m_a *line.m_b - m_b * line.m_a);
                result = true;
            }
            else if (!isVertical()) {
                inters.m_x = line.m_a;
                inters.m_y = (m_c - m_a * line.m_a)/m_b;
                result = true;
            }
            else if (!line.isVertical()) {
                inters.m_x = m_a;
                inters.m_y = (line.m_c - line.m_a * m_a)/line.m_b;
                result = true;
            }
        }

        return result;
    }

    /**
     * Find the point of intersection with a line segment.
     * @param[in] s A line segment to find intersection with.
     * @param[out] inters Point to hold intersection.
     * @return True if line segment intersects this line, and the segment is not
     *      completey on-top of this line (infinite intersections).
     */
    bool intersection(DSegment s, DPoint &inters) {
        Line2d line(s);
        DPoint p;
        if (intersection(line, p)) {
            double minX = min({s.start().m_x, s.end().m_x});
            double maxX = max({s.start().m_x, s.end().m_x});
            double minY = min({s.start().m_y, s.end().m_y});
            double maxY = max({s.start().m_y, s.end().m_y});
            if (p.m_x >= minX && p.m_x <= maxX
                && p.m_y >= minY && p.m_y <= maxY) {
                inters = p;
                return true;
            }
        }
        return false;
    }

private:
}; // class Line2d


inline ostream& operator<<(ostream& os, const Line2d& line) {
    if (!line.isVertical()) {
        os << line.m_a << "*x + " << line.m_b << "*y = " << line.m_c;
    }
    else {
        os << "x = " << line.m_a;
    }

    return os;
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
 
