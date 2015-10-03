/*
The MIT License
Copyright (c) 2015 UCSC Autonomous Systems Lab
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/
#include <math.h>
#include <stdio.h>
#include <algorithm>
#include <Eigen/Dense>

using Eigen::Vector3d;
using Eigen::Vector2d;

#include "Dubins.h"
#include "Util.h"

//#define DUBINS_DEBUG

#define HEADING_TOLERANCE          1E-10 // upperbound for Euclidean metric, radius safe?

#define MAX_EDGE_COST           999999.0

/**
 * Calculate the shortest Dubins' path distance to the node. Note that all angles
 * used in this function are heading angles from 0 at the y-axis, and
 * counter-clockwise is positive. 
 */
double dubinsPathLength(Configuration &Cs, Configuration &Ce, double r) {
    Vector2d Ps = Vector2d(Cs.x(), Cs.y()),
        Pe = Vector2d(Ce.x(), Ce.y());
    double Xs = Cs.m_heading,
        Xe = Ce.m_heading;
    double dist = (Ps - Pe).norm();

    #ifdef DUBINS_DEBUG
    cout << "Given Cs=" << Cs << ", Ce=" << Ce << ", r=" << r << endl;
    cout << "Got dist=" << dist << " compared to r=" << r << "." << endl;
    #endif
    /*
    printf("Given Cs=(\n    %+E,\n    %+E,\n    %+.25E)\n",
        Cs.x(), Cs.y(), Cs.heading());

    printf("Given Ce=(\n    %+E,\n    %+E,\n    %+.25E)\n",
        Ce.x(), Ce.y(), Ce.heading());
    */

    if (dist < 3.0 * r) {
        std::cerr << "distance must be larger than 3*r" << endl;
        return -1.0;
    }

    // Added tolerance to avoid numerical instability for paths with no curviture
    // TODO compute curviture as percent safe for any turn radius?
    if (fabs(Xs - Xe) <= HEADING_TOLERANCE) {
        #ifdef DUBINS_DEBUG
        printf("Using straight line L=%+E. |Xs - Xe| = %+E <= %+E\n",(Pe - Ps).norm(), fabs(Xs - Xe), HEADING_TOLERANCE);
        #endif
        return (Pe - Ps).norm();
    }

    double alpha = headingToAngle(Xs),
           beta = headingToAngle(Xe);

    // Find circle center points for each case
    Vector3d R_rs(cos(alpha - M_PI/2.0), sin(alpha - M_PI/2.0), 0.0),
        R_ls(cos(alpha + M_PI/2.0), sin(alpha + M_PI/2.0), 0.0),
        R_re(cos(beta - M_PI/2.0), sin(beta - M_PI/2.0), 0.0),
        R_le(cos(beta + M_PI/2.0), sin(beta + M_PI/2.0), 0.0);

    Vector3d PC_rs(Cs.x(), Cs.y(), 0.0),
        PC_ls(Cs.x(), Cs.y(), 0.0),
        PC_re(Ce.x(), Ce.y(), 0.0),
        PC_le(Ce.x(), Ce.y(), 0.0);

    PC_rs = PC_rs.transpose() + r*R_rs.transpose();
    PC_ls = PC_ls.transpose() + r*R_ls.transpose();
    PC_re = PC_re.transpose() + r*R_re.transpose();
    PC_le = PC_le.transpose() + r*R_le.transpose();

    #ifdef DUBINS_DEBUG
    cout << "PC_rs " << PC_rs << "." << endl;
    cout << "PC_ls " << PC_ls << "." << endl;
    cout << "PC_re " << PC_re << "." << endl;
    cout << "PC_le " << PC_le << "." << endl;
    #endif

    // Case I, R-S-R
    double x = headingBetween(PC_rs, PC_re);
    //printf("norm(c_rs - c_re)=%+E \nr*wrap1=%+E\n r*wrap2=%0.9f\n",(PC_rs - PC_re).norm(),
    //    r*wrapAngle(2.0 * M_PI + wrapAngle(x - M_PI/2.0) - wrapAngle(Xs - M_PI/2.0)),
    //    r*wrapAngle(2.0*M_PI + wrapAngle(Xe - M_PI/2.0) - wrapAngle(x - M_PI/2.0)));
    // printf("r*wrap2: wrapAngle(Xe - M_PI/2.0)=%+E, wrapAngle(x - M_PI/2.0)=%+E\n",
    //    wrapAngle(Xe - M_PI/2.0), wrapAngle(x - M_PI/2.0));
    /*
    printf("CW: Xe - X = %+E\n", wrapAngle(Xe - M_PI/2.0) - wrapAngle(x - M_PI/2.0));
    printf("CW: X - Xe = %+E\n", wrapAngle(x - M_PI/2.0) - wrapAngle(Xe - M_PI/2.0));
    printf("CW: Xe = %+E\n", wrapAngle(Xe - M_PI/2.0));
    printf("CW: X = %+E\n", wrapAngle(x - M_PI/2.0));
    printf("Norm = %+E\n",(PC_rs - PC_re).norm() );
    */

    double L1 = (PC_rs - PC_re).norm() 
        + r*wrapAngle(2.0 * M_PI + wrapAngle(x - M_PI/2.0) - wrapAngle(Xs - M_PI/2.0))
        + r*wrapAngle(2.0 * M_PI + wrapAngle(Xe - M_PI/2.0) - wrapAngle(x - M_PI/2.0));

/*
    printf("L1 = %+E\n    + %+E\n    +%+E\n\n = %+E\n",
        (PC_rs - PC_re).norm() ,
         r*wrapAngle(2.0 * M_PI + wrapAngle(x - M_PI/2.0) - wrapAngle(Xs - M_PI/2.0)),
         r*wrapAngle(2.0 * M_PI + wrapAngle(Xe - M_PI/2.0) - wrapAngle(x - M_PI/2.0)),
        L1);
        */

    #ifdef DUBINS_DEBUG
    std::cout << "L1: " << L1 << ", with x=" << x << endl;
    #endif

    // Case II, R-S-L
    double ls = (PC_le - PC_rs).norm();
    x = headingBetween(PC_rs, PC_le);
    double x2 = x - M_PI/2.0 + asin(2.0*r/ls);
    double L2 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(x2)
        - wrapAngle(Xs - M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x2 + M_PI)
        - wrapAngle(Xe + M_PI/2.0));
    #ifdef DUBINS_DEBUG
    std::cout << "L2: " << L2 << " with ls=" << ls << ", x=" << x << ", x2=" << x2 << endl;
    #endif

    // Case III, L-S-R
    ls = (PC_re - PC_ls).norm();
    x = headingBetween(PC_ls, PC_re);
    x2 = acos(2.0*r/ls);
    if (2.0*r/ls > 1.0 || 2.0*r/ls < -1.0) {
        std::cerr << "angle out of range in case III" << endl;
        return -1.0;
    }
    double L3 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(Xs + M_PI/2.0) 
        - wrapAngle(x + x2)) + r*wrapAngle(2.0*M_PI + wrapAngle(Xe - M_PI/2.0)
        - wrapAngle(x + x2 - M_PI));
    #ifdef DUBINS_DEBUG
    std::cout << "L3: " << L3 << " with ls=" << ls << ", x=" << x << ", x2=" << x2 << endl;
    #endif

    // Case IV, L-S-L
    x = headingBetween(PC_ls, PC_le);
    double L4 = (PC_ls - PC_le).norm() + r*wrapAngle(2.0*M_PI + wrapAngle(Xs + M_PI/2.0)
        - wrapAngle(x + M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x + M_PI/2.0)
        - wrapAngle(Xe + M_PI/2.0));
    #ifdef DUBINS_DEBUG
    std::cout << "L4: " << L4 << ", with x=" << x << endl;

    std::cout << "Comparing L1=" << L1 << " L2=" << L2 << " L3=" << L3 << " L4=" << L4 << endl;
    #endif

    return std::min({L1, L2, L3, L4});
}

/**
 * Finds the cost of the shortest dubins path through the given tour using the
 * headings X. If returnCost is true, the cost of returning back to the first
 * node in the tour will be included.
 */
double dubinsTourCost(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour, ogdf::NodeArray<double> &X,
    double r, bool returnCost) {
    ogdf::ListIterator<ogdf::node> iter;
    double cost = 0.0;

    if (tour.size() < 2) return 0.0;

    // Add the return edge if necessary
    ogdf::List<ogdf::node> modTour(tour);
    ogdf::node lastNode = modTour.back();
    if (returnCost && lastNode != *(modTour.begin())) {
        modTour.pushBack(*(tour.begin()));
    }
    else if (!returnCost && lastNode == *(modTour.begin())) {
        modTour.popBack();
    }

    int m = modTour.size() - 1;
    int i = 0; // edge index
    for ( iter = modTour.begin(); (i < m && iter != modTour.end()); iter++ ) {
        ogdf::node u = *iter, v = *(iter.succ());

        Configuration Cu(GA.x(u), GA.y(u), X(u)),
                      Cv(GA.x(v), GA.y(v), X(v));
        cost += dubinsPathLength(Cu, Cv, r);
        i++;
    }

    return cost;
}


/**
 * Adds edges to the graph with the cost of a Dubins tour. Returns the cost of
 * the shortest dubins path through the given tour. If returnEdge is true, the
 * return edge is added. Edges created are saved into the list of edges.
 */
double createDubinsTourEdges(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour, ogdf::NodeArray<double> &X,
    double r, ogdf::List<ogdf::edge> &edges, bool returnEdge) {
    ogdf::ListIterator<ogdf::node> iter;
    double total_cost = 0.0;

    if (tour.size() < 2) return 0.0;
    if (G.numberOfEdges() > 0) {
        std::cerr << "cannot have existing edges in graph" << endl;
        return -1.0;
    }

    // Add the return edge if necessary
    ogdf::List<ogdf::node> modTour(tour);
    ogdf::node lastNode = modTour.back();
    if (returnEdge && lastNode != *(modTour.begin())) {
        modTour.pushBack(*(tour.begin()));
    }
    else if (!returnEdge && lastNode == *(modTour.begin())) {
        modTour.popBack();
    }
 
    int m = modTour.size() - 1;
    int i = 0; // edge index
    for ( iter = modTour.begin(); (i < m && iter != modTour.end()); iter++ ) {
        ogdf::node u = *iter, v = *(iter.succ());

        Configuration Cu(GA.x(u), GA.y(u), X(u)),
                      Cv(GA.x(v), GA.y(v), X(v));
        double cost = dubinsPathLength(Cu, Cv, r);

        //printf("Found cost %0.1f from node %d->%d, where headings %d: %0.1f, %d: %0.1f\n",
        //    cost, GA.idNode(u), GA.idNode(v), GA.idNode(u), X(u), GA.idNode(v), X(v));

        // Add the edge
        ogdf::edge e = G.newEdge(u,v);
        GA.doubleWeight(e) = cost;
        edges.pushBack(e);
        total_cost += cost;
        i++;
    }

    return total_cost;
}



/**
 * Computes an adjacency matrix of Dubins path lengths between nodes for ATSP.
 */
void buildDubinsAdjacencyMatrix(ogdf::Graph &G, ogdf::GraphAttributes &GA, 
    NodeMatrix<double> &A, ogdf::NodeArray<double> &X, double turnRadius) {
  
    ogdf::node i, j;
    forall_nodes(i, G) {
        Configuration Ci(GA.x(i), GA.y(i), X(i));

        forall_nodes(j, G) {
            if (i == j) {
                A[i][i] = MAX_EDGE_COST;
                continue;
            }
            Configuration Cj(GA.x(j), GA.y(j), X(j));
            
            double w = dubinsPathLength(Ci, Cj, turnRadius);
            A[i][j] = w;
        }
    }
}

