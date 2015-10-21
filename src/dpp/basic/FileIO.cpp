/*
 * TSPlib file input/output functions for reading and writing TSP and PAR files.
 *
 * Copyright (C) 2014-2015 DubinsPathPlanner.
 * Created by David Goodman <dagoodma@gmail.com>
 * Redistribution and use of this file is allowed according to the terms of the MIT license.
 * For details see the LICENSE file distributed with DubinsPathPlanner.
 */
#include <stdio.h>
#include <regex>
#include <stdbool.h>

#include <dpp/basic/basic.h>
#include <dpp/basic/FileIO.h>
#include <dpp/basic/Logger.h>

namespace dpp {

const char* TspFile::ProblemTypeText[] = { "TSP", "ATSP", "HCP" };

const char* GmlFile::TypeText[] = { "Graph", "Polygon" };

// ---------------- GmlFile Functions -----------------
/**
 * Reads a polygon from the points in a GML file.
 */
int readPolygonFromGmlFile(std::string filename, ogdf::DPolygon &poly) {
    ogdf::List<ogdf::DPoint> Points;
    if (readPointsFromGmlFile(filename, Points) != SUCCESS) {
        return FAILURE;
    }

    // Copy DPoints into a polygon
    ogdf::ListIterator<ogdf::DPoint> iter;
    for ( iter = Points.begin(); iter != Points.end(); iter++ ) {
        ogdf::DPoint p = *iter;
        if (!poly.containsPoint(p)) {
            poly.pushBack(p);
        }
    }

    return SUCCESS;
}

/**
 * Reads a points from a GML file.
 */
int readPointsFromGmlFile(std::string filename, ogdf::List<ogdf::DPoint> &Points) {
    ogdf::Graph G;
    ogdf::GraphAttributes GA(G, DPP_GRAPH_ATTRIBUTES);
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Reading " << filename << " for points." << std::endl;

    if (readGraphFromGmlFile(filename, G, GA) != SUCCESS) {
        return FAILURE;
    }
    Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Found " << G.numberOfNodes() << ". Creating list..." << std::endl;
    ogdf::node u;
    int i = 0;
    forall_nodes(u, G) {
        ogdf::DPoint p(GA.x(u), GA.y(u));
        Points.pushBack(p);
        i++;
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "   Point " << GA.idNode(u) << ": "
            << GA.x(u) << ", " << GA.y(u) << "." << std::endl;
    }
    DPP_ASSERT(i == G.numberOfNodes());
    return SUCCESS;
}

/**
 * Reads a graph and attributes from a GML file.
 */
int readGraphFromGmlFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    if (ogdf::GraphIO::readGML(GA, G, filename)) {
        return SUCCESS;
    }
    return FAILURE;
}

// ---------------- TspFile Functions -----------------

void writeTspHeader(std::ofstream& tspFile, TspFile::ProblemType type, std::string name,
    std::string comment, int dimension) {
    std::string typeStr = TspFile::ProblemTypeText[type];
    tspFile << 
        "NAME: " << name << std::endl << 
        "TYPE: " << typeStr << std::endl <<
        "COMMENT: " << comment << std::endl <<
        "DIMENSION: " << dimension <<  std::endl;
}

/**
 * Write a symmetric TSP file of graph G with Euclidean distance metric.
 */
int writeEtspFile(std::string filename, std::string name, std::string comment,
    ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    std::ofstream tspFile;
    tspFile.open(filename, std::ios_base::trunc);
    if (!tspFile.is_open()) {
        Logger::logError() << "Could not open " << filename << "." << std::endl;
        return FAILURE;
    }

    // Write the header, then the data section
    int n = G.numberOfNodes();
    writeTspHeader(tspFile, TspFile::ProblemType::ETSP, name, comment, n);
    tspFile <<
        "EDGE_WEIGHT_TYPE: EUC_2D" << std::endl <<
        "NODE_COORD_SECTION" << std::endl << std::scientific;
    tspFile.precision(5); 

    ogdf::node i;
    int i_offset = 0;
    if (G.firstNode()->index() < 1) 
        i_offset++;
    
    forall_nodes(i, G) {
        tspFile << (i->index() + i_offset) << " " << GA.x(i) << " " << GA.y(i) << std::endl;
    }

    tspFile << "EOF";
    tspFile.close();
    return SUCCESS;
}


/**
 * Write an asymmetrical TSP file using A.
 */
int writeAtspFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A) {
    std::ofstream tspFile;
    tspFile.open(filename, std::ios_base::trunc);
    if (!tspFile.is_open()) {
        Logger::logError() << "Could not open " << filename << "." << std::endl;
        return FAILURE;
    }

    // Write the header, then the data section
    int n = A.numberOfNodes();
    writeTspHeader(tspFile, TspFile::ProblemType::ATSP, name, comment, n);
    tspFile <<
        "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl <<
        "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl <<
        "EDGE_WEIGHT_SECTION" << std::endl << std::fixed;
    tspFile.precision(0); 

    ogdf::node i, j;
    forall_nodes(i, G) {
        forall_nodes(j, G) {
            tspFile << " " << A[i][j];
        }
        tspFile << std::endl;
    }

    tspFile << "EOF";
    tspFile.close();
    return SUCCESS;
}


int writeParFile(std::string filename, std::string tspFilename, std::string outputFilename,
    int runs) {
    std::ofstream parFile;
    parFile.open(filename, std::ios_base::trunc);
    if (!parFile.is_open()) {
        Logger::logError() << "Could not open " << filename << "." << std::endl;
        return FAILURE;
    }

    parFile <<
        "PROBLEM_FILE = " << tspFilename << std::endl <<
        "OUTPUT_TOUR_FILE = " << outputFilename << std::endl <<
    //    "OPTIMUM = " << optimumCost << std::endl <<
        "MOVE_TYPE = 5" << std::endl <<
        "PATCHING_C = 3" << std::endl <<
        "PATCHING_A = 2" << std::endl <<
        "RUNS = " << runs << std::endl;

    parFile.close();
    return SUCCESS;
}

/**
 * Reads a TSP tour file into G and GA using the reference Graph and attributes.
 */
 int readTspTourFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour, bool returnToInitial) {
    std::ifstream tourFile;
    tourFile.open(filename);
    if (!tourFile.is_open()) {
        Logger::logError() << "Could not open tour " << filename << "." << std::endl;
        return FAILURE;
    }

    if (tour.size() > 0) {
        Logger::logWarn() << "Gave non-empty tour." << std::endl;
    }

    // Build node lookup list
    ogdf::node nodeList[G.maxNodeIndex()], i;
    int maxNodeId = -1;
    forall_nodes(i,G) {
        int id = GA.idNode(i);
        nodeList[id] = i;
        if (id > maxNodeId) maxNodeId = id;
    }

    // Regex patterns for parsing
    std::regex nameSection("NAME\\s*:\\s*(.+)\\s*", std::regex_constants::icase);
    std::regex commentSection("COMMENT\\s*:\\s*(.+)\\s*", std::regex_constants::icase);
    std::regex tourSection("TOUR_SECTION", std::regex_constants::icase);
    std::regex dimensionSection("DIMENSION\\s*:\\s*(\\d+)", std::regex_constants::icase);
    std::regex eofSection("EOF", std::regex_constants::icase);

    std::string line;
    bool foundName, foundComment, foundTour = false;

    std::string name, comment;
    int n = -1;
    int nExpected = G.numberOfNodes();
    std::smatch match;
    try {
        while ( getline (tourFile,line)) {
            // Look for the name section
            if (!foundName && std::regex_search(line, match, nameSection) && match.size() > 1) {
                name = match.str(1);
                foundName = true;
            }
            // Look for the comment section (cost/length)
            else if (!foundComment && std::regex_search(line, match, commentSection) && match.size() > 1) {
                comment = match.str(1);
                foundComment = true;
            }
            // Look for dimension section
            else if (n < 0 && std::regex_search(line, match, dimensionSection) && match.size() > 1) {
                n = std::stoi(match.str(1));
                if (n != nExpected) {
                    Logger::logError() << "Tour contains " << n << " nodes, but graph has " << nExpected << "." << std::endl;
                    return FAILURE;
                }
            }
            // Look for the tour list
            else if (!foundTour && std::regex_search(line, match, tourSection)) {
                foundTour = true;
                // Read the points
                while (getline (tourFile,line) && !std::regex_search(line, match, eofSection)) {
                    //std::cout << "Running stoi on: " << line << std::endl;
                    int nodeId = std::stoi(line); // - 1;
                    //std::cout << "Considering " << nodeId << std::endl;
                    if (nodeId < -1 || nodeId > maxNodeId) {
                        Logger::logError() << "Out of range node id " << nodeId << " in tour file." << std::endl;
                        return FAILURE;
                    }
                    if (nodeId >= 0) {
                        ogdf::node u = nodeList[nodeId];
                        if (!u) {
                            Logger::logError() << "Node id " << nodeId << " in the tour does not exist." << std::endl;
                            return FAILURE;
                        }
                        tour.pushBack(u);
                    }
                }
                int nFound = tour.size();
                if (nFound != n) {
                    Logger::logError() << "Found " << nFound << " nodes in tour, but expected " << n << "." << std::endl;
                    return FAILURE;
                }
                // Return to the initial point
                if (nFound > 0 && returnToInitial) {
                    // Return to start
                    tour.pushBack(*(tour.begin()));
                }
            }
        }
        
        Logger::logDebug(DPP_LOGGER_VERBOSE_2) << "Read tour file: " << std::endl
            << "\tName: " << name << std::endl
            << "\tComment: " << comment << std::endl
            << "\tSize: " << n << std::endl;
    }
    catch (std::regex_error& e) {
        Logger::logError() << "Parser syntax error: " << e.what() << std::endl;
        return FAILURE;
    }
    catch (std::exception& e) {
        Logger::logError() << "Parser error: " << e.what() << std::endl;
        return FAILURE;
    }

    if (!foundTour) {
        Logger::logError() << "Missing tour section." << std::endl;
        return FAILURE;
    }
    if (!foundName) {
        Logger::logWarn() << "Missing name section." << std::endl;
    }
    if (!foundComment) {
        Logger::logWarn() << "Missing comment section." << std::endl;
    }

    return SUCCESS;
}

} // namespace dpp

