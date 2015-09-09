#ifndef _DUBINS_H_
#define _DUBINS_H_

#include <math.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>

#include "Configuration.h"
#include "NodeMatrix.h"


// Prototypes
double dubinsPathLength(Configuration &Cs, Configuration &Ce, double r);

void buildDubinsAdjacencyMatrix(ogdf::Graph &G, ogdf::GraphAttributes &GA,
    NodeMatrix<double> &A, ogdf::NodeArray<double> &X, double turnRadius);

#endif // _DUBINS_H_
