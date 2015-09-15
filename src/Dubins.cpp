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

#define MAX_EDGE_COST           999999.0

/**
 * Calculate the shortest Dubins' path distance to the node.
 */
double dubinsPathLength(Configuration &Cs, Configuration &Ce, double r) {
    //cout << "Checking Cs=" << Cs << ", Ce=" << Ce << "." << endl;
    Vector2d Ps = Vector2d(Cs.x(), Cs.y()),
        Pe = Vector2d(Ce.x(), Ce.y());
    double Xs = Cs.m_heading,
        Xe = Ce.m_heading;
    double alpha = headingToAngle(Xs),
        beta = headingToAngle(Xe);
    double dist = (Ps - Pe).norm();

    //cout << "Got dist=" << dist << " compared to r=" << r << "." << endl;

    if (dist < 3.0 * r) {
        std::cerr << "distance must be larger than 3*r" << endl;
        return -1.0;
    }

    // Find circle center points for each case
    Vector3d DC_sr(cos(alpha - M_PI/2.0), sin(alpha - M_PI/2.0), 0.0),
        DC_sl(cos(alpha + M_PI/2.0), sin(alpha - M_PI/2.0), 0.0),
        DC_er(cos(beta - M_PI/2.0), sin(beta - M_PI/2.0), 0.0),
        DC_el(cos(beta + M_PI/2.0), sin(beta + M_PI/2.0), 0.0);

    Vector3d PC_sr(Cs.x() + r * DC_sr[0], Cs.y() + r * DC_sr[1], 0.0),
        PC_sl(Cs.x() + r * DC_sl[0], Cs.y() + r * DC_sl[1], 0.0),
        PC_er(Ce.x() + r * DC_sr[0], Ce.y() + r * DC_sr[1], 0.0),
        PC_el(Ce.x() + r * DC_sl[0], Ce.y() + r * DC_sl[1], 0.0);

    // Case I, R-S-R
    double x = headingToAngle(headingBetween(PC_sr, PC_er));
    double L1 = (PC_sr - PC_er).norm() + r*wrapAngle(2.0 * M_PI + wrapAngle(x - M_PI/2.0)
        - wrapAngle(alpha - M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(beta - M_PI/2.0)
        - wrapAngle(x - M_PI/2.0));

    // Case II, R-S-L
    double ls = (PC_el - PC_sr).norm();
    x = headingToAngle(headingBetween(PC_sr, PC_el));
    double x2 = x - M_PI/2.0 + asin(2.0*r/ls);
    double L2 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(x2)
        - wrapAngle(alpha - M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x2 + M_PI)
        - wrapAngle(beta + M_PI/2.0));

    // Case III, L-S-R
    ls = (PC_er - PC_sl).norm();
    x = headingToAngle(headingBetween(PC_sl, PC_er));
    x2 = acos(2.0*r/ls);
    if (2.0*r/ls > 1.0 || 2.0*r/ls < -1.0) {
        std::cerr << "angle out of range in case III" << endl;
        return -1.0;
    }
    double L3 = sqrt(ls*ls - 4*r*r) + r*wrapAngle(2.0*M_PI + wrapAngle(alpha + M_PI/2.0) 
        - wrapAngle(x + x2)) + r*wrapAngle(2.0*M_PI + wrapAngle(beta - M_PI/2.0)
        - wrapAngle(x + x2 - M_PI));

    // Case IV, L-S-L
    x = headingToAngle(headingBetween(PC_sl, PC_el));
    double L4 = (PC_sl - PC_el).norm() + r*wrapAngle(2.0*M_PI + wrapAngle(alpha + M_PI/2.0)
        - wrapAngle(x + M_PI/2.0)) + r*wrapAngle(2.0*M_PI + wrapAngle(x + M_PI/2.0)
        - wrapAngle(beta + M_PI/2.0));


    return std::min({L1, L2, L3, L4});
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

