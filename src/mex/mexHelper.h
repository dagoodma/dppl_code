#ifndef _MEX_HELPER_H_
#define _MEX_HELPER_H_

#include <dpp/basic/basic.h>

#include <ogdf/basic/Graph.h>
#include <ogdf/basic/GraphAttributes.h>
#include <ogdf/basic/List.h>
#include <ogdf/basic/NodeArray.h>

#include "mex.h"

using ogdf::node;
using ogdf::edge;
using ogdf::Graph;
using ogdf::GraphAttributes;
using ogdf::List;
using ogdf::ListIterator;
using ogdf::NodeArray;

#define MEX_MODULE_NAME "DubinsPathPlanner" 

#define MEX_MAT(a,i,j,m) a[(i)+(j)*m] 

#define MEX_DEBUG


int unpackNodes(Graph &G, GraphAttributes &GA, double *pV, int n);

int packEdges(Graph &G, GraphAttributes &GA, List<edge> &Edges, double *pE);
 
int packHeadings(Graph &G, GraphAttributes &GA, NodeArray<double> &X, double *pX);


#endif // _MEX_HELPER_H_