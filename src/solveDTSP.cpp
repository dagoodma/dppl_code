/*
 * DubinsPathPlanner tool for solving the Dubins Traveling Salesperson problem.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <cxxopts.h>
#include "stacktrace.h" // todo put into lib

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

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
    // Setup stack traces for debugging
    char const *program_name = argv[0];
    #ifdef DPP_DEBUG
    set_signal_handler(program_name);
    #endif

    // Input arguments
    string inputFilename;
    double x, r;
    bool debug = false;

    // Option parsing
    cxxopts::Options options(program_name,
        " gml_file initial_heading turn_radius");
    try {
        options.add_options()
            ("d,debug", "Enable debugging messages", cxxopts::value<bool>(debug))
            ("h,help", "Print this message")
            ("a,algorithm", "Algorithm for DTSP (nearest,alternating,randomized =default)",
                cxxopts::value<std::string>()->default_value("randomized"), "DTSP_ALGORITHM")
            //("", "x is the initial heading")
            //("", "r is the turn radius of the Dubins vehicle");
            // TODO hide these:
            ("inputGMLFile", "Input GML file to read graph from", cxxopts::value<std::string>(), "INPUT_GML_FILE")
            ("initialHeading", "Initial heading orientation", cxxopts::value<double>(), "INITIAL_HEADING")
            ("turnRadius", "Dubins vehicle turning radius", cxxopts::value<double>(), "TURN_RADIUS");

        options.parse_positional({"inputGMLFile", "initialHeading", "turnRadius"});
        options.parse(argc, argv);

        inputFilename =  options["inputGMLFile"].as<std::string>();
        x = options["initialHeading"].as<double>();
        r = options["turnRadius"].as<double>();

        // Check algorithm
        if (!options.count("algorithm")) {
            options["algo"]
        }



        if (options.count("help")) {
            std::cout << options.help() << std::endl;
            return 0;
        }

        if (argc != 4) {
            std::cout << program_name << ": " << " Expected 3 arguments." << std::endl;
            std::cout << options.help();
            exit(1);
        }

    } catch (const cxxopts::OptionException& e) {
        std::cout << "error parsing options: " << e.what() << std::endl;
        exit(1);
    }
/*
    inputFilename = argv[1];
    x = atof(argv[2]);
    r = atof(argv[3]);
*/
    //dpp::Logger *log = dpp::Logger::Instance();
    //log->level(dpp::Logger::Level::LL_DEBUG); // set debug mode

    // Build path planner
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