/*
 * Path planner for a Dubins vehicle. Considers the 2D turning radius of the vehicle.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <ogdf/fileformats/GraphIO.h>

#include <dpp/basic/basic.h>
#include <dpp/planner/PathPlanner.h>
#include <dpp/planner/DubinsVehiclePathPlanner.h>

namespace dpp {

/* 
 * Load graph from GML files.
 */
void DubinsVehiclePathPlanner::addWaypoints(std::string gmlFilename) {
    DPP_ASSERT(ogdf::GraphIO::readGML(m_GA, m_G, gmlFilename));
    Logger::logDebug() << "Opened " << gmlFilename << " with " << waypointCount()
        << " nodes." << std::endl;
}

/*
 * Copy the graph.
 */
void DubinsVehiclePathPlanner::addWaypoints(ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    PathPlanner::m_G = G;
    PathPlanner::m_GA = GA;
    PathPlanner::m_Headings.init(PathPlanner::m_G);
    Logger::logDebug() << "Copied graph with " << waypointCount() << " nodes." << std::endl;
}

/*
 * Run the algorithm to solve the planning problem.
 */
bool DubinsVehiclePathPlanner::solve(void) {
    DPP_ASSERT(waypointCount() > 1);
    m_haveSolution = false;

    Logger::logInfo() << "Solving " << waypointCount() << " node problem with "
        << m_algorithm->name() << " " << m_algorithm->typeText() << " algorithm." << std::endl;
    try {
        AlgorithmDTSP *alg = dynamic_cast<AlgorithmDTSP*>(m_algorithm.get());
        // TODO add checking
        alg->run(m_G, m_GA, m_initialHeading, m_turnRadius, m_Tour,
            m_Edges, m_Headings, m_cost, m_returnToInitial);
        m_haveSolution = true;
    } catch(std::exception &e) {
        Logger::logError() << "An exception occured: " << e.what() << std::endl;
        m_haveSolution = false;
    }

    return m_haveSolution;
}


} // namespace dpp 