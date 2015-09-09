#ifndef _TSPLIB_H_
#define _TSPLIB_H_

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include "NodeMatrix.h"

int writeATSPFile(std::string filename, std::string name, std::string comment, 
    ogdf::Graph &G, NodeMatrix<double> &A);

#endif // _TSPLIB_H_
