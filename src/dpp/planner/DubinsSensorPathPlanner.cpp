/*
 * Path planner for a Dubins vehicle with a sensor. Considers the 2D turning radius
 * of the vehicle and the coverage width of the sensor.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <dpp/planner/DubinsSensorPathPlanner.h>
#include <dpp/basic/FileIO.h>
#include <dpp/basic/Field.h>

namespace dpp {

void DubinsSensorPathPlanner::polygon(std::string gmlFilename) {
    DPP_ASSERT(readPolygonFromGmlFile(gmlFilename, m_polygon) == SUCCESS);

    polygon(m_polygon); // for logger message
}

void DubinsSensorPathPlanner::polygon(DPolygon polygon) {
    m_polygon = polygon;

    Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Set polygon to " << &polygon <<
        " with n=" << polygon.size() << " points." << std::endl;
}

void DubinsSensorPathPlanner::polygon(List<DPoint> points) {
    Logger::logWarn() << "polygon(List<DPoint>) not implemented!" << std::endl;
}


/**
 * Sets the CPP algorithm for solving.
 */
void DubinsSensorPathPlanner::algorithm(CppPlanningAlgorithm algId) {
    switch (algId) {
        case BOUSTROPHEDON:
            m_algorithm.reset(new BoustrophedonCpp());
            Logger::logDebug() << "Set CPP algorithm to BOUSTROPHEDON." << std::endl;
            break;
        default:
            DPP_ASSERT(0 && "Unknown planning algorithm");
            Logger::logError() << "Unknown planning algorithm: " << algId << std::endl;
            return;
    } // switch (algId)
}

/**
 * Find the DTSP solution to the CPP problem.
 */
bool DubinsSensorPathPlanner::solveAsDtsp(DtspPlanningAlgorithm algId) {
    DPP_ASSERT(m_polygon.size() >= 3);
    m_haveSolution = false;

    try {
        // Create a DTSP path planner and copy settings
        DubinsVehiclePathPlanner p(m_turnRadius);
        p.algorithm(algId);
        p.initialHeading(m_initialHeading);
        p.turnRadius(m_turnRadius);
        p.returnToInitial(m_returnToInitial);

        Logger::logInfo(DPP_LOGGER_VERBOSE_1) << "Solving coverage problem with "
        << p.algorithmName() << " DTSP algorithm." << std::endl;
        
        // Clear and add origin to graph with heading
        m_G.clear();
        ogdf::node nodeStart = m_G.newNode();
        m_GA.x(nodeStart) = m_initialConfig.x();
        m_GA.y(nodeStart) = m_initialConfig.y();
        m_GA.idNode(nodeStart) = 1;
        m_Headings(m_G.firstNode()) = m_initialConfig.heading();

        // Construct field and grid polygon by sensor width,
        // building a graph with centroids as nodes
        Field field(m_polygon, m_sensorWidth);
        int n = field.addNodesFromGrid(m_G, m_GA);
        Logger::logDebug(DPP_LOGGER_VERBOSE_1) << "Added " << n << " nodes "
            << "to the Graph." << std::endl;

        // Solve as DTSP problem
        p.addWaypoints(m_G, m_GA);
        if (p.solve() == true) {
            m_haveSolution = true;
            p.copySolution(m_G, m_GA, m_Tour, m_Edges, m_Headings, m_cost);
        }
    } catch(std::exception &e) {
        Logger::logError() << "An exception occured: " << e.what() << std::endl;
        m_haveSolution = false;
    }
    return m_haveSolution;
}

bool DubinsSensorPathPlanner::solve(void) {
    DPP_ASSERT(m_polygon.size() >= 3);
    m_haveSolution = false;

    // Clear and add origin to graph with heading
    m_G.clear();
    ogdf::node nodeStart = m_G.newNode();
    m_GA.x(nodeStart) = m_initialConfig.x();
    m_GA.y(nodeStart) = m_initialConfig.y();
    m_GA.idNode(nodeStart) = 1;
    m_Headings(m_G.firstNode()) = m_initialConfig.heading();

    Logger::logInfo(DPP_LOGGER_VERBOSE_1) << "Solving coverage problem with "
        << m_algorithm->name() << " " << m_algorithm->typeText() << " algorithm." << std::endl;
    try {
        // Generate the field
        Field field(m_polygon, m_sensorWidth);
        if (!field.isConvex()) {
            throw std::domain_error("Non-convex polygons not yet supported.");
        }

        // Call the algorithm
        AlgorithmCpp *alg = dynamic_cast<AlgorithmCpp*>(m_algorithm.get());
        if (alg->run(field, m_initialConfig, m_turnRadius, m_returnToInitial,
            m_G, m_GA, m_Tour, m_Edges, m_Headings, m_cost) == SUCCESS) {
            m_haveSolution = true;
        }
    } catch(std::exception &e) {
        Logger::logError() << "An exception occured: " << e.what() << std::endl;
        m_haveSolution = false;
    }
    return m_haveSolution;
}


} // namespace dpp 