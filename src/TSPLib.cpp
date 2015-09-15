#include <stdio.h>
#include <regex>
#include <stdbool.h>

#include "TSPLib.h"
#include "stacktrace.h"

const char* TSPFile::ProblemTypeText[] = { "TSP", "ATSP", "HCP" };

void writeTSPHeader(std::ofstream& tspFile, TSPFile::ProblemType type, std::string name,
    std::string comment, int dimension) {
    std::string typeStr = TSPFile::ProblemTypeText[type];
    tspFile << 
        "NAME: " << name << std::endl << 
        "TYPE: " << typeStr << std::endl <<
        "COMMENT: " << comment << std::endl <<
        "DIMENSION: " << dimension <<  std::endl;
}



/**
 * Write a symmetric TSP file of graph G with Euclidean distance metric.
 */
int writeETSPFile(std::string filename, std::string name, std::string comment,
    ogdf::Graph &G, ogdf::GraphAttributes &GA) {
    std::ofstream tspFile;
    tspFile.open(filename);
    if (!tspFile.is_open()) {
        cerr << "Could not open " << filename << std::endl;
        return 1;
    }

    // Write the header, then the data section
    int n = G.numberOfNodes();
    writeTSPHeader(tspFile, TSPFile::ProblemType::ETSP, name, comment, n);
    tspFile <<
        "EDGE_WEIGHT_TYPE: EUC_2D" << std::endl <<
        "NODE_COORD_SECTION" << std::endl << std::scientific;
    tspFile.precision(5); 

    ogdf::node i;
    int i_offset = 0;
    if (G.firstNode()->index() < 1) i_offset++;
    forall_nodes(i, G) {
        tspFile << (i->index() + i_offset) << " " << GA.x(i) << " " << GA.y(i) << std::endl;
    }

    tspFile << "EOF";
    tspFile.close();
    return 0;
}


/**
 * Write an asymmetrical TSP file using A.
 */
int writeATSPFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A) {
    std::ofstream tspFile;
    tspFile.open(filename);
    if (!tspFile.is_open()) {
        cerr << "Could not open " << filename << std::endl;
        return 1;
    }

    // Write the header, then the data section
    int n = A.numberOfNodes();
    writeTSPHeader(tspFile, TSPFile::ProblemType::ATSP, name, comment, n);
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
    return 0;
}


/**
 * Write an asymmetrical TSP file using A.
 */

/*
int writeATSPFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A) {
    ofstream tspFile;
    tspFile.open(filename);
    if (!tspFile.is_open()) {
        cerr << "Could not open " << filename << std::endl;
        return 1;
    }

    // Find largest number of digits
    int nDigits = 12; // eg. 3.14159e+000, 2.00600e+003
    int n = A.numberOfNodes();
    //cout << "HERE at line " << __LINE__ << " in file " << __FILE__ << "." << std::endl;

    tspFile << 
        "NAME: " << name << std::endl << 
        "TYPE: ATSP" << std::endl <<
        "COMMENT: " << comment << std::endl <<
        "DIMENSION: " << n <<  std::endl <<
        "EDGE_WEIGHT_TYPE: EXPLICIT" << std::endl <<
        "EDGE_WEIGHT_FORMAT: FULL_MATRIX" << std::endl <<
        "EDGE_WEIGHT_SECTION" << std::endl << std::fixed;
    tspFile.precision(0); 

    ogdf::node i, j;
    //Graph G = *A.graphOf();
    forall_nodes(i, G) {
        forall_nodes(j, G) {
            tspFile << " " << A[i][j];
        }
        tspFile << std::endl;
    }

    tspFile << "EOF";
    tspFile.close();
    return 0;
}
*/



int writePARFile(std::string filename, std::string tspFilename, std::string outputFilename,
    int runs) {
    std::ofstream parFile;
    parFile.open(filename);
    if (!parFile.is_open()) {
        std::cerr << "Could not open " << filename << std::endl;
        return 1;
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
    return 0;
}

/**
 * Reads a TSP tour file into G and GA using the reference Graph and attributes.
 */
 int readTSPTourFile(std::string filename, ogdf::Graph &G, ogdf::GraphAttributes &GA,
    ogdf::List<ogdf::node> &tour) {
    std::ifstream tourFile;
    tourFile.open(filename);
    if (!tourFile.is_open()) {
        cerr << "Could not open " << filename << std::endl;
        return 1;
    }

    // Build node lookup list
    ogdf::node nodeList[G.maxNodeIndex()], i;
    forall_nodes(i,G) {
        int id = GA.idNode(i);
        nodeList[id] = i;
        //std::cout << "Added node " << (long int)i << " at " << id << "." << std::endl;
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
            if (!foundName && std::regex_search(line, match, nameSection) && match.size() > 1) {
                name = match.str(1);
                foundName = true;
                //std::cout << "Found name: " << name << std::endl;
            }
            else if (!foundComment && std::regex_search(line, match, commentSection) && match.size() > 1) {
                comment = match.str(1);
                foundComment = true;
                //std::cout << "Found comment: " << comment << std::endl;
            }
            else if (n < 0 && std::regex_search(line, match, dimensionSection) && match.size() > 1) {
                n = std::stoi(match.str(1));
                if (n != nExpected) {
                    std::cerr << "Tour contains " << n << " nodes, but expected " << nExpected << "."
                        << std::endl;
                    return 1;
                }
                //std::cout << "Found dimension: " << n << std::endl;
            }
            else if (!foundTour && std::regex_search(line, match, tourSection)) {
                foundTour = true;
                // Read the points
                while (getline (tourFile,line) && !std::regex_search(line, match, eofSection)) {
                    //std::cout << "Running stoi on: " << line << std::endl;
                    int nodeId = std::stoi(line); // - 1;
                    //std::cout << "Considering " << nodeId << std::endl;
                    if (nodeId >= 0) {
                        ogdf::node u = nodeList[nodeId];
                        if (!u) {
                            std::cerr << "Node id " << nodeId << " does not exist." << std::endl;
                            return 1;
                        }
                        tour.pushBack(u);

                        //std::cout << "Pushed node " << (long int)(u) << ", id = " << nodeId << "." << std::endl;
                        //std::cout << "Pushed node " << nodeId << "." << std::endl;
                    }
                }
                int nFound = tour.size();
                if (nFound != n) {
                    std::cerr << "Read " << nFound << " nodes, but expected " << n << "." << std::endl;
                    return 1;
                }
                if (nFound > 0) {
                    // Return to start
                    tour.pushBack(*(tour.begin()));
                }
            }
        }
        /*
        std::cout << "Read tour file: " << std::endl
            << "\tName: " << name << std::endl
            << "\tComment: " << comment << std::endl
            << "\tSize: " << n << std::endl;
        */
    }
    catch (std::regex_error& e) {
        std::cerr << "Parser syntax error: " << e.what() << std::endl;
        return 1;
    }
    catch (std::exception& e) {
        std::cerr << "Parser error: " << e.what() << std::endl;
        print_stacktrace();
        return 1;
    }

    if (!foundTour) {
        std::cerr << "Missing tour section." << std::endl;
        return 1;
    }
    if (!foundName) {
        std::cerr << "Warning: Missing name." << std::endl;
    }
    if (!foundComment) {
        std::cerr << "Warning: Missing comment." << std::endl;
    }

    return 0;
 }

