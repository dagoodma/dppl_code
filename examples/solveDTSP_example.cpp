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

/** Main Entry Point
 * The solution is a tour, which is an ordered list of waypoints,
 * saved inside as a TSPlib file. The total cost is printed.
 * 
 * usage: solveDTSP [<inputGMLFile> <startHeading> <turnRadius>
 * @param inputGMLFile  input GML file to read the problem from
 * @param startHeading  a starting heading in radians [0,2*pi)
 * @param turnRadius    a turning radius in radians
 * @return An exit code (0==SUCCESS)
 */
int main(int argc, char *argv[]) {
    // Check input arguments
    if (argc != 4) {
        cerr << "Expected 3 arguments!" << endl;
        exit(1);
    }
    // Read input arguments
    string inputFilename = argv[1];
    double x = atof(argv[2]);
    double r = atof(argv[3]);

    // Build path planner and find a solution
    dpp::DubinsVehiclePathPlanner p;
    
    p.addWaypoints(inputFilename);
    p.initialHeading(0.0);
    p.turnRadius(1.0);
    p.returnToInitial(true);

    p.solve();

    // Results
    ogdf::Graph G = p.graph();
    ogdf::GraphAttributes GA = p.graphAttributes();
    ogdf::List<ogdf::edge> E = p.edges();
    ogdf::List<ogdf::node> Tour = p.tour();
    double cost = p.cost();

    cout << "Solved " << G.numberOfNodes() << " point tour with cost " << cost << "." << endl;

    // Print edge list
    dpp::printEdges(G, GA, E);

    return SUCCESS;

}