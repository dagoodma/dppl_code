/*
 * Example showing how to build a path planner for a Dubins Traveling Salesperson
 * Problem (DTSP). The solution's tour, headings, and edges are printed.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdio.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

using namespace std;

/**
 * This example code solves an 8 waypoint triangle pattern with a turn radius
 * of 10 meters, and an initial heading of 0 radians (north). The initial position
 * is the first node in the loaded .gml graph.
 */
int main(int argc, char *argv[]) {

    // Read input arguments (this compiles into: build_dir/examples/)
    string inputFilename("../../data/triangle-8wp.gml");
    double x = 0.0;
    double r = 10.0;

    // Build path planner and find a solution
    dpp::DubinsVehiclePathPlanner p;
    
    p.addWaypoints(inputFilename);
    p.initialHeading(0.0);
    p.turnRadius(1.0);
    p.returnToInitial(true);

    p.solve();

    // Results. Must use graph pointer.
    ogdf::Graph *G = p.graphPtr();
    ogdf::GraphAttributes GA = p.graphAttributes();
    ogdf::List<ogdf::edge> E = p.edges();
    ogdf::List<ogdf::node> Tour = p.tour();
    ogdf::NodeArray<double> Headings = p.headings();
    double cost = p.cost();

    // Print headings and edge list
    ogdf::node u;
    std::cout << "Solved " << G->numberOfNodes() << " point tour with cost " << cost << "." << std::endl;
    std::cout << dpp::printHeadings(*G, GA, Headings);
    std::cout << dpp::printEdges(*G, GA, E);

    return 0;
}