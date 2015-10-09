/*
 * Path-related utility functions.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdio.h>
#include <math.h>
#include <algorithm>

#include <Eigen/Dense>

#include <dpp/basic/Path.h>

using Eigen::Vector3d;
using Eigen::Vector2d;

//#define DPP_PATH_DEBUG

#define HEADING_TOLERANCE          1E-10 // upperbound for Euclidean metric, radius safe?

#define MAX_EDGE_COST           999999.0 // todo: scale this based on edge costs

namespace dpp {

/**
 * Calculate the shortest Dubins' path distance to the node. Note that all angles
 * used in this function are heading angles from 0 at the y-axis, and
 * counter-clockwise is positive. 
 */
double dubinsPathLength(VehicleConfiguration &Cs, VehicleConfiguration &Ce, double r) {
    Vector2d Ps = Vector2d(Cs.x(), Cs.y()),
        Pe = Vector2d(Ce.x(), Ce.y());
    double Xs = Cs.m_heading,
        Xe = Ce.m_heading;
    double dist = (Ps - Pe).norm();

    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "Given Cs=" << Cs << ", Ce=" << Ce << ", r=" << r << std::endl;
    Logger::logDebug() << "Got dist=" << dist << " compared to r=" << r << "." << std::endl;
    #endif

    DPP_ASSERT(dist >= 3.0 * r);
    //if (dist < 3.0 * r) {
    //  std::domain_error("distance must be larger than 3*r");
    //}

    // Added tolerance to avoid numerical instability for paths with no curviture
    // TODO compute curviture as ratio w.r.t. turn radius?
    if (fabs(Xs - Xe) <= HEADING_TOLERANCE) {
        #ifdef DPP_PATH_DEBUG
        Logger::logDebug() << "Using straight line L=" << (Pe - Ps).norm() << ". |Xs - Xe| = "
            << fabs(Xs - Xe) << " <= " << HEADING_TOLERANCE << std::endl;
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

    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "PC_rs " << PC_rs << "." << std::endl;
    Logger::logDebug() << "PC_ls " << PC_ls << "." << std::endl;
    Logger::logDebug() << "PC_re " << PC_re << "." << std::endl;
    Logger::logDebug() << "PC_le " << PC_le << "." << std::endl;
    #endif

    // Case I, R-S-R
    double x = headingBetween(PC_rs, PC_re);
    double L1 = (PC_rs - PC_re).norm() 
        + r*wrapAngle(2.0 * M_PI + wrapAngle(x - M_PI/2.0) - wrapAngle(Xs - M_PI/2.0))
        + r*wrapAngle(2.0 * M_PI + wrapAngle(Xe - M_PI/2.0) - wrapAngle(x - M_PI/2.0));

    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "L1: " << L1 << ", with x=" << x << std::endl;
    #endif

    // Case II, R-S-L
    double ls = (PC_le - PC_rs).norm();
    x = headingBetween(PC_rs, PC_le);
    double x2 = x - M_PI/2.0 + asin(2.0*r/ls);
    double L2 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(x2)
        - wrapAngle(Xs - M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x2 + M_PI)
        - wrapAngle(Xe + M_PI/2.0));
    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "L2: " << L2 << " with ls=" << ls << ", x=" << x << ", x2=" << x2 << std::endl;
    #endif

    // Case III, L-S-R
    ls = (PC_re - PC_ls).norm();
    x = headingBetween(PC_ls, PC_re);
    double ratioOA = 2.0*r/ls;
    // Bound the ratio from -1 to 1
    ratioOA = std::min<double> (-1.0, ratioOA);
    ratioOA = std::max<double> (1.0, ratioOA);
    DPP_ASSERT(2.0*r/ls <= 1.0 && 2.0*r/ls >= -1.0);
    //    std::cerr << "angle out of range in case III" << endl;
    //    return -1.0;
    //}
    x2 = acos(ratioOA);
    double L3 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(Xs + M_PI/2.0) 
        - wrapAngle(x + x2)) + r*wrapAngle(2.0*M_PI + wrapAngle(Xe - M_PI/2.0)
        - wrapAngle(x + x2 - M_PI));

    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "L3: " << L3 << " with ls=" << ls << ", x=" << x << ", x2="
        << x2 << std::endl;
    #endif

    // Case IV, L-S-L
    x = headingBetween(PC_ls, PC_le);
    double L4 = (PC_ls - PC_le).norm() + r*wrapAngle(2.0*M_PI + wrapAngle(Xs + M_PI/2.0)
        - wrapAngle(x + M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x + M_PI/2.0)
        - wrapAngle(Xe + M_PI/2.0));

    #ifdef DPP_PATH_DEBUG
    Logger::logDebug() << "L4: " << L4 << ", with x=" << x << std::endl;
    Logger::logDebug() << "Comparing L1=" << L1 << " L2=" << L2 << " L3=" << L3 << " L4="
        << L4 << std::endl;
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

    if (tour.size() < 2) {
        Logger::logWarn() << "Zero cost for an empty tour." << std::endl;
        return 0.0;
    }

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

        VehicleConfiguration Cu(GA.x(u), GA.y(u), X(u)),
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
    DPP_ASSERT(G.numberOfEdges() < 1);
    //    std::range_error("Cannot have existing edges in graph");
    //}

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

        VehicleConfiguration Cu(GA.x(u), GA.y(u), X(u)),
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
 * Computes an adjacency matrix of Dubins path lengths between nodes with the
 * given headings (for ATSP).
 */
void buildDubinsAdjacencyMatrix(ogdf::Graph &G, ogdf::GraphAttributes &GA, 
    NodeMatrix<double> &A, ogdf::NodeArray<double> &X, double turnRadius) {
  
    ogdf::node i, j;
    forall_nodes(i, G) {
        VehicleConfiguration Ci(GA.x(i), GA.y(i), X(i));

        forall_nodes(j, G) {
            if (i == j) {
                A[i][i] = MAX_EDGE_COST;
                continue;
            }
            VehicleConfiguration Cj(GA.x(j), GA.y(j), X(j));
            
            double w = dubinsPathLength(Ci, Cj, turnRadius);
            A[i][j] = w;
        }
    }
}

} // namespace DPP

