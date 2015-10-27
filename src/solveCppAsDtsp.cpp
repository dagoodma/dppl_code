/*
 * DubinsPathPlanner tool for solving the Coverage Path Planning for a sensor-equipped
 * Dubins vehicle, using the Dubins Traveling Salesperson problem.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */

#include <memory>
#include <cxxopts.hpp>
#include <stacktrace.h>

#include <solveCppAsDtsp.h>

#include <dpp/basic/Logger.h>
#include <dpp/basic/Util.h>
#include <dpp/planner/DubinsSensorPathPlanner.h>

/** 
 * Solve a CPP scenario by converting the problem to a Dtsp.
 * @param poly  Polygon to cover.
 * @param C     Vehicle starting configuration.
 * @param G     An empty graph to hold the solution.
 * @param GA    Attributes of solution graph.
 * @param r     Vehicle turn radius in radians.
 * @param e     Sensor width in meters.
 * @param Tour  List of nodes to hold result
 * @param Edges List of edges to hold result
 * @param Headings Node array of headings to hold result
 * @param returnToInitial Return to the initial configuration at the end of tour.
 * @param algId Algorithm for solving Dtsp. See dpp::DubinsVehiclePathPlanner::DtspPlanningAlgorithm
 * @return Success or failure
 */
 int solveCppAsDtsp(ogdf::DPolygon poly, dpp::VehicleConfiguration C, ogdf::Graph &G,
    ogdf::GraphAttributes &GA, double r, double e, ogdf::List<ogdf::node> &Tour,
    ogdf::List<ogdf::edge> &Edges, NodeArray<double> &Headings, double &cost,
    bool returnToInitial, dpp::DtspPlanningAlgorithm algId) {
    DPP_ASSERT(G.empty());

    dpp::DubinsSensorPathPlanner p;
    p.polygon(poly);
    p.initialConfiguration(C);
    p.turnRadius(r);
    p.sensorWidth(e);
    p.returnToInitial(returnToInitial);

    if (!p.solveAsDtsp(algId)) {
        return FAILURE;
    }

    // Get the results
    p.copySolution(G, GA, Tour, Edges, Headings, cost);
    
    return SUCCESS;
}

/** Main Entry Point
 * The solution is a tour, which is an ordered list of waypoints,
 * saved inside a TSPlib file. The total cost, and edges are printed.
 * 
 * usage: solveCppAsDtsp [OPTIONS] <inputGMLFile> <startX> <startY> <startHeading> <turnRadius> <sensorWidth>
 * @param inputGMLFile  input GML file to read the polygon vertices from
 * @param startX        Starting x position. 
 * @param startY        Starting y position.
 * @param startHeading  a starting heading in radians [0,2*pi)
 * @param turnRadius    a turning radius in radians
 * @param sensorWdith   width of sensor coverage in meters
 * @return An exit code (0==SUCCESS)
 *
 * @options
 * -d, --debug
 *     Enables debug mode in logging module for extra information.
 * -h, --help
 *     Prints a help message and exits.
 * -a, --dtspalgorithm=<DtspPlanningAlgorithmName>
 *     Sets the planning algorithm ("nearest", "alternating", "randomized").
 * -r,--return
 *     Whether to return to initial configuration.
 */
int main(int argc, char *argv[]) {

    // Setup stack traces for debugging
    char const *program_name = argv[0];
    #ifdef DPP_DEBUG
    set_signal_handler(program_name);
    #endif

    // Input arguments
    string inputFilename;
    double r, e;
    dpp::VehicleConfiguration initialConfig;
    int verbose = 0;
    //bool verbose = false;
    bool noReturn = false;
    bool debug = false;

    dpp::DubinsSensorPathPlanner p;

    dpp::DtspPlanningAlgorithm algId; // set below

    // Option parsing
    cxxopts::Options options(program_name,
        " gml_file initial_x initial_y initial_heading turn_radius sensor_width");
    try {
        options.add_options()
            ("d,debug", "Enable debugging messages",cxxopts::value<bool>(debug))
            ("h,help", "Print this message")
            ("a,dtspalgorithm", "Algorithm for DTSP (nearest,alternating,randomized =default)",
                cxxopts::value<std::string>()->default_value("alternating"), "DTSP_ALGORITHM")
            ("r,noreturn", "Disables returning to initial configuration", cxxopts::value<bool>(noReturn))
            ("v,verbose", "Prints increasingly verbose messages", cxxopts::value<int>(verbose));

        // Positional arguments
        options.add_options("Positional")
            ("inputGMLFile", "Input GML file to read polygon points", cxxopts::value<std::string>(), "INPUT_GML_FILE")
            ("initialX", "Initial x position", cxxopts::value<double>(), "INITIAL_X")
            ("initialY", "Initial y position", cxxopts::value<double>(), "INITIAL_Y")
            ("initialHeading", "Initial heading orientation", cxxopts::value<double>(), "INITIAL_HEADING")
            ("turnRadius", "Dubins vehicle turning radius", cxxopts::value<double>(), "TURN_RADIUS")
            ("sensorWidth", "Sensor coverage width", cxxopts::value<double>(), "SENSOR_WIDTH");


        // Exit with help message if they didn't provide enough arguments
        if (argc < 7) {
            std::cout << program_name << ": " << " Expected at least 5 arguments." << std::endl;
            std::cout << options.help({""});
            return FAILURE;
        }

        options.parse_positional({"inputGMLFile", "initialX" , "initialY", "initialHeading",
            "turnRadius", "sensorWidth"});
        options.parse(argc, argv);

        // Read arguments
        inputFilename =  options["inputGMLFile"].as<std::string>();
        initialConfig.x(options["initialX"].as<double>());
        initialConfig.y(options["initialY"].as<double>());
        initialConfig.heading(options["initialHeading"].as<double>());
        r = options["turnRadius"].as<double>();
        e = options["sensorWidth"].as<double>();

        // Setup logger
        dpp::Logger *log = dpp::Logger::Instance();

        // Set debug mode if given in options
        if (debug) {
            log->level(dpp::Logger::Level::LL_DEBUG);
        }

        // Set verbosity if given in options
        if (verbose) {
        //if (options.count("verbose")) {
        //    verbosity = options["verbose"].as<bool>();
            log->verbose(verbose);
        }

        // Set desired algorithm
        if (options.count("dtspalgorithm")) {
            std::string algName = options["dtspalgorithm"].as<std::string>();
            if (algName.compare("nearest") == 0) {
                algId = dpp::DtspPlanningAlgorithm::NEAREST_NEIGHBOR;
            }
            else if (algName.compare("alternating") == 0) {
                algId = dpp::DtspPlanningAlgorithm::ALTERNATING;
            }
            else if (algName.compare("randomized") == 0) {
                algId = dpp::DtspPlanningAlgorithm::RANDOMIZED;
            }
            else {
                std::cout << program_name << ": " << "Invalid DTSP algorithm \'" << algName << "\'." << std::endl;
                std::cout << options.help();
                return FAILURE;
                //exit(1);
            }
        } // (options.count("algorithm"))

        // Print help and exit if help option was set
        if (options.count("help")) {
            std::cout << options.help() << std::endl;
            return SUCCESS;
            //exit(SUCCESS);
        }

    } catch (const cxxopts::OptionException& e) {
        std::cout << program_name << " error parsing options: " << e.what() << std::endl;
        return FAILURE;
    } // try

    // Set up planner
    p.polygon(inputFilename);
    p.initialConfiguration(initialConfig);
    p.turnRadius(r);
    p.sensorWidth(e);
    p.returnToInitial(!noReturn);

    if (!p.solveAsDtsp(algId)) {
        return FAILURE;
    }

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

    return SUCCESS;
}